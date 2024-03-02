
/************************INCLUDES*********************************/

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "utilities.h"
#include "lcd_i2c_grove.h"


/************************MACROS***********************************/

#define BUF_SIZE 1024
#define RMC "$GNRMC"
#define JUMP " \n"
#define UART0 UART_NUM_0
#define UART1 UART_NUM_1

/**************DECLARACIÓN DE VARIABLES GLOBALES*******************/

static QueueHandle_t uart0_celular_queue;
static QueueHandle_t uart1_gnss_queue;
static SemaphoreHandle_t uart_sem;
static GNSSData_t quectel_l76;

/****************DECLARACIÓN DE FUNCIONES*************************/

static void uart_interrupt_task(void *params);
static void transmit_to_server_task(void *params);

/**********************FUNCIÓN PRINCIPAL**************************/

void app_main()
{
    // UART Para recibir comandos por el teclado
    uart_init(UART0, 115200, BUF_SIZE * 2, BUF_SIZE * 2, 50, &uart0_celular_queue, ESP_INTR_FLAG_LEVEL1);
    //          (UART_NUM, TX, RX, RTS, CTS)
    uart_set_pin(UART0,     1,  3,  23,  19);

    // UART_1 conectar con modulo 4g A7670SA
    uart_init(UART1, 115200, BUF_SIZE * 2, BUF_SIZE * 2, 50, &uart1_gnss_queue, ESP_INTR_FLAG_LEVEL1); //   ESP_INTR_FLAG_IRAM
    //          (UART_NUM, TX, RX, RTS, CTS)
    uart_set_pin(UART1,    33, 26,  14,  12);

   // lcd_init(); 
   // lcd_clear(); 
   // lcd_set_RGB(255, 255, 255);

   uart_sem = xSemaphoreCreateBinary();

    xTaskCreate(uart_interrupt_task,
                "uart_interrupt_task",
                BUF_SIZE * 4,
                NULL,
                12,
                NULL);

    xTaskCreate(transmit_to_server_task,
                "transmit_to_server_task",
                BUF_SIZE * 4,
                NULL,
                12,
                NULL);



}



/****************DEFINICIÓN DE FUNCIONES**************************/

static void uart_interrupt_task(void *params)
{
    uart_event_t uart_event;
    uint8_t *uart_recv_data = (uint8_t *)malloc(BUF_SIZE*5);
    uint8_t *at_command = (uint8_t *)malloc(BUF_SIZE);
    uint8_t *at_response = (uint8_t *)malloc(BUF_SIZE*5);
    

    while (1)
    {
        if (xQueueReceive(uart0_celular_queue, (void *)&uart_event, (TickType_t)portMAX_DELAY))
        {
            bzero(uart_recv_data, BUF_SIZE*5);
            bzero(at_command, BUF_SIZE);
            

            switch (uart_event.type)
            {
            case UART_DATA:
              
                    uart_receive(UART0, (void *)uart_recv_data, (uint32_t)uart_event.size);
                
                    sprintf((char *)at_command, "%s\n\r", uart_recv_data);

                    uart_transmit(UART1, at_command, strlen((const char*)at_command));

                 
                

                break; 

            default:
                break;
            }

        }
        
        
        if (xQueueReceive(uart1_gnss_queue, (void *)&uart_event, (TickType_t)portMAX_DELAY))
        {
            bzero(uart_recv_data, BUF_SIZE*5);
            bzero(at_response, BUF_SIZE*5);

            switch (uart_event.type)
            {
            case UART_DATA:
                

                    uart_receive(UART1, (void *)uart_recv_data, (uint32_t)uart_event.size);
                    sprintf((char *)at_response, "%s\n\r", uart_recv_data);

                    uart_transmit(UART0, at_response, strlen((const char*)at_response));

                
                
                break; 

            default:
                break;
            }
        }
    }

    free(uart_recv_data);
    free(at_command);
}

static void transmit_to_server_task(void *params)
{

}