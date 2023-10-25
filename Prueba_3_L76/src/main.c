
/************************INCLUDES*********************************/

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "utilities.h"
#include "esp_log.h"

/************************MACROS***********************************/

#define BUF_SIZE 1024
#define RMC "$GNRMC"
#define JUMP " \n"
#define UART0 UART_NUM_0
#define UART1 UART_NUM_1

/**************DECLARACIÓN DE VARIABLES GLOBALES*******************/

static QueueHandle_t uart1_queue;
static GNSSData_t quectel_l76;

/****************DECLARACIÓN DE FUNCIONES*************************/

static void uart_interrupt_task(void *params);

/**********************FUNCIÓN PRINCIPAL**************************/

void app_main()
{
    uart_init(UART0, 115200, BUF_SIZE * 2, 0, 0, NULL, ESP_INTR_FLAG_LEVEL1);
    //  (UART_NUM, TX, RX, RTS, CTS)
    uart_set_pin(UART0, 1, 3, 22, 19);

    uart_init(UART1, 9600, BUF_SIZE * 2, BUF_SIZE * 2, 50, &uart1_queue, ESP_INTR_FLAG_LEVEL1); //   ESP_INTR_FLAG_IRAM
                                                                                                //  (UART_NUM, TX, RX, RTS, CTS)
    uart_set_pin(UART1, 33, 26, 14, 12);

    xTaskCreate(uart_interrupt_task,
                "uart_interrupt_task",
                BUF_SIZE * 4,
                NULL,
                12,
                NULL);
}

/****************DEFINICIÓN DE FUNCIONES**************************/

static void uart_interrupt_task(void *params)
{
    uart_event_t uart_event;
    uint8_t *uart_recv_data = (uint8_t *)malloc(BUF_SIZE);
    uint8_t *nmea_string = (uint8_t *)malloc(BUF_SIZE + 100);
    uint8_t *proof_print = (uint8_t *)malloc(BUF_SIZE + 100);
    while (1)
    {
        uart_transmit(UART1, (const void *)RMC, strlen(RMC));
        if (xQueueReceive(uart1_queue, (void *)&uart_event, (TickType_t)portMAX_DELAY))
        {
            bzero(uart_recv_data, BUF_SIZE);
            bzero(nmea_string, BUF_SIZE + 100);
            bzero(proof_print, BUF_SIZE + 100);

            switch (uart_event.type)
            {
            case UART_DATA:
                uart_receive(UART1, (void *)uart_recv_data, (uint32_t)uart_event.size);
                sprintf((char *)nmea_string, "%s", uart_recv_data);
                // uart_transmit(UART0,  "---", 3);
                //uart_transmit(UART0, nmea_string, uart_event.size);
                nmea_parser((const char *)nmea_string, &quectel_l76);
                sprintf((char *)proof_print, "Lat: %.6f , Long: %.6f", 
                        quectel_l76.latitude, quectel_l76.longitude);
                uart_transmit(UART0, proof_print, strlen(proof_print));

                break;

            default:
                break;
            }
        }
    }

    free(uart_recv_data);
    free(nmea_string);
}

