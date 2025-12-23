**Overview**
The goal of this project was to create a FreeRTOS program that contained common functionality where one task preforms calculations on incomming data while another task handles user I/O. 

I utilized many skills that I learned through DigiKey's FreeRTOS youtube course while exploring a new development environment. I also chose to explore the espidf framework instead of relying on the more abstracted Arduino framework. This project gave me exposure to useful tools like gpio, usb_serial_jtag, and gptimer.

**Explanation**
This RTOS program allows for users to type 'avg' into the command line and get back a voltage value from a pin. An interrupt handles reading in a raw adc value every 10th of a second. This value is placed in a double buffer where each buffer has a length of 10. Once a buffer is full, the ISR will use a semaphore to check if the calculation task is still using the other buffer. If the calculation task is not active, the ISR will notify the calculation task which will compute the average of all values within a buffer. If the calculation task still active, an overrun flag will be set and any read values will be dropped. On notification, the calculation task will then calculate the average voltage given the average raw adc value. Once the  This average is stored in a global variable protected by a mutex. The user interface task acts as a typical CLI, echoing back characters that are typed into the serial monitor. Upon the user typing 'avg', the user interface task will print out the globally stored average that is protected by a mutex.
 
**Tools Used**
The processor I used on was an ESP32-C3 on a board found here: https://eih-portal.nd.edu/assets/578701/build_a_board_information_sheet_v5_14aug24_.pdf

All development was done on platformio with an esp32-c3-devkitm-1 environment and espidf framework.