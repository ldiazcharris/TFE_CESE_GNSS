
/************************INCLUDES*********************************/

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "utilities.h"

/************************MACROS***********************************/

#define BUF_SIZE    1024
#define RMC         "$GNRMC"
#define JUMP         " \n"
#define UART0       UART_NUM_0
#define UART1       UART_NUM_1



/****************DECLARACIÓN DE FUNCIONES*************************/







/**************DECLARACIÓN DE VARIABLES GLOBALES*******************/




/**********************FUNCIÓN PRINCIPAL**************************/

void app_main() 
{
    uart_init(UART0, 115200, BUF_SIZE*2, 0, 0, NULL, ESP_INTR_FLAG_IRAM);
            //  (UART_NUM, TX, RX, RTS, CTS) 
    uart_set_pin(UART0,     1, 3,  22,  19);


    uart_init(UART1, 9600, BUF_SIZE*2, 0, 0, NULL, ESP_INTR_FLAG_IRAM); // ESP_INTR_FLAG_IRAM
            //  (UART_NUM, TX, RX, RTS, CTS) 
    uart_set_pin(UART1,    33, 26, 14,  12);

    char receive_data[100];
    memset(receive_data, 0, sizeof(receive_data));
    char print_data[150];

     while(1)
     {
        
        uart_transmit(UART1,  RMC);

        uart_receive(UART1, (void *) receive_data);

        sprintf(print_data, "%s", receive_data);

        uart_transmit(UART0,  receive_data);
        

     }

}



/****************DEFINICIÓN DE FUNCIONES**************************/
