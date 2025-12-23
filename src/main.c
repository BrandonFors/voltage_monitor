#include <build_a_board.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"

#define CMD_BUF_LEN 100
#define BUF_LEN 10

static const BaseType_t app_cpu = 0;

//globals

static volatile uint16_t buffer_1[BUF_LEN];
static volatile uint16_t buffer_2[BUF_LEN];
static volatile uint16_t* write_to = buffer_1; // pointer used to tell what buffer ISR writes to
static volatile uint16_t* read_from = buffer_2; // pointer to tell what buffer calc task reads from
static volatile uint8_t buffer_overrun = 0; // tells us if there were ISR readings that were dropped

static float avg;

static TaskHandle_t processing_task = NULL;
//mutexes semaphores and timers

static SemaphoreHandle_t avg_mutex = NULL; // used when editing gloval avg var

static gptimer_handle_t timer = NULL; // timer that triggers isr reading

static SemaphoreHandle_t done_reading_sem = NULL; // tells when the calc task is done reading and we can swap pointers

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED; // spinlock that will be used for critical sections that can't be interrupted

static adc_channel_t adc_chan = ADC_CHANNEL_0; // adc converter channel

static adc_oneshot_unit_handle_t adc1_handle; // handle for adc


static bool IRAM_ATTR read_adc_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx){
  //read from the adc
  int raw = 0;
  static uint16_t idx = 0;
  BaseType_t task_woken = pdFALSE;

  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_chan, &raw));
  
  if(idx<BUF_LEN){
    write_to[idx] = raw;
    idx++;
  }
  
  if(idx >= BUF_LEN){
    if(xSemaphoreTakeFromISR(done_reading_sem, &task_woken) == pdFALSE){
      //no need to enter critical section here as an isr cannot be interrupted
      buffer_overrun = 1;
    }
    //if the below is run, the semaphore for done_reading has been successfully taken
    if(buffer_overrun == 0){ // if ther is no overrun reset idx and swap the read/write pointers
      idx = 0;
      volatile uint16_t* temp_ptr = write_to;
      write_to = read_from;
      read_from = temp_ptr;

      //notify task that a buffer is ready
      vTaskNotifyGiveFromISR(processing_task, &task_woken);
    }
  }
  //return value is whether a high priority task was awakend
  return (task_woken == pdTRUE);
}

void calc_avg(void *parameters){
  float new_avg;


  //Vout = Dout * Vmax / Dmax
  //Dout = average of the 10 samples
  //Dmax = 4095
  while(1){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    new_avg = 0.0;
    for(int i = 0; i<BUF_LEN; i++){
      new_avg += (float)read_from[i];
    }
    new_avg /= BUF_LEN;
    new_avg = new_avg * 3.3 / 4095;

    //take semaphore to edit the new avg
    if(xSemaphoreTake(avg_mutex, portMAX_DELAY) == pdTRUE){
      avg = new_avg;
      xSemaphoreGive(avg_mutex);
    }

    portENTER_CRITICAL(&spinlock);
    buffer_overrun = 0;
    xSemaphoreGive(done_reading_sem);
    portEXIT_CRITICAL(&spinlock);
  }

}


void blink_led(void *parameters) {
  while(1){
    gpio_set_level(LED_BUILTIN, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(LED_BUILTIN, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}

void interface(void *parameters){
  static char cmd_line[CMD_BUF_LEN];
  static int idx = 0;
  char c;
  char avg_string[10];
  while(1){
    //read from avg if recieved message
    // Write data to UART.
    //handler user input

    if(usb_serial_jtag_read_bytes(&c, sizeof(char), 0) > 0){
      
      if (c == '\n' || c == '\r'){
        cmd_line[idx] = '\0';
        usb_serial_jtag_write_bytes("\r\n", 2, 0);
        idx = 0;
        if(strcmp(cmd_line, "avg") == 0){
          if(xSemaphoreTake(avg_mutex, portMAX_DELAY) == pdTRUE){
            sprintf(avg_string,"%.2f", avg); //use mutex here to access this variable
            xSemaphoreGive(avg_mutex);
          }
          usb_serial_jtag_write_bytes("Average V: ", 11, 0);
          usb_serial_jtag_write_bytes(avg_string, strlen(avg_string), 0);
          usb_serial_jtag_write_bytes("\r\n", 2, 0);


        }
      }else if(idx < (CMD_BUF_LEN -1)){
        cmd_line[idx] = c;
        usb_serial_jtag_write_bytes(&c, 1, 0);
        idx++;
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }


}


void app_main() {
  //setup serial monitor 
  usb_serial_jtag_driver_config_t serial_config = {
    .rx_buffer_size = 1024,
    .tx_buffer_size = 1024,
  };
  usb_serial_jtag_driver_install(&serial_config);

  


  //set led pin to output mode
  gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);
  //set adc pin to input mode

  //create mutexes/semaphores
  avg_mutex = xSemaphoreCreateMutex();
  done_reading_sem = xSemaphoreCreateBinary();
  //initialize sem_done_reading to 1 in order for the isr task to switch to a new buffer initially
  xSemaphoreGive(done_reading_sem);


  //create adc reader
  adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
  adc_oneshot_chan_cfg_t chan_config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_chan, &chan_config));

  // create blink task
  xTaskCreatePinnedToCore(
    blink_led,
    "Blink LED",
    1024,
    NULL,
    1,
    NULL,
    app_cpu
  );

  xTaskCreatePinnedToCore(
    calc_avg,
    "Calculate ADC Avg",
    2048,
    NULL,
    3,
    &processing_task,
    app_cpu
  );

  xTaskCreatePinnedToCore(
    interface,
    "User Interface",
    4096,
    NULL,
    2,
    NULL,
    app_cpu
  );

  //setup timer
  gptimer_config_t timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000,
    .intr_priority = 0,
  };
  //create timer instance
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));

  gptimer_alarm_config_t alarm_config = {
    .reload_count = 0,
    .alarm_count = 100000, // note that this is one digit less than the resolution so that it will trigger every 10th of a second
    .flags.auto_reload_on_alarm = true,
  };
  //set the timer's alarm action
  ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));

  gptimer_event_callbacks_t cbs = {
    .on_alarm = read_adc_on_alarm,
  };
  //register timer event callback functions
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));

  //enable and start timer
  ESP_ERROR_CHECK(gptimer_enable(timer));
  ESP_ERROR_CHECK(gptimer_start(timer));


  vTaskDelete(NULL);
}