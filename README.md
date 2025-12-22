**Overview**
The goal of this project was to create a FreeRTOS program that contained common functionality where one task preforms calculations on incomming data while another task handles user I/O. 

I utilized many skills that I learned through DigiKey's FreeRTOS youtube course while exploring a new development environment. I also chose to explore the espidf framework instead of relying on the more abstracted Arduino framework. This project gave me exposure to useful tools like gpio, usb_serial_jtag, and gptimer.

The processor I used on was an ESP32-C3 on a board found here: https://eih-portal.nd.edu/assets/578701/build_a_board_information_sheet_v5_14aug24_.pdf

All development was done on platformio with an esp32-c3-devkitm-1 environment and espidf framework.