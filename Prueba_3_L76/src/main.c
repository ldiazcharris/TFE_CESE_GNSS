
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

#define BUF_SIZE    1024
#define RMC         "$GNRMC"
#define JUMP         " \n"
#define UART0       UART_NUM_0
#define UART1       UART_NUM_1




/**************DECLARACIÓN DE VARIABLES GLOBALES*******************/

static QueueHandle_t uart1_queue;

static const char *TAG = "NMEA_PARSER";

typedef struct {
    float latitude;
    float longitude;
    char time[10];
} GNSSData;


/****************DECLARACIÓN DE FUNCIONES*************************/

static void uart_interrupt_task(void *params);
static void parser_nemea_task(const char *nmeaString, GNSSData *gnssData);



/**********************FUNCIÓN PRINCIPAL**************************/

void app_main() 
{
    uart_init(UART0, 115200, BUF_SIZE*2, 0, 0, NULL, ESP_INTR_FLAG_LEVEL1);
            //  (UART_NUM, TX, RX, RTS, CTS) 
    uart_set_pin(UART0,     1, 3,  22,  19);


    uart_init(UART1, 9600, BUF_SIZE*2, BUF_SIZE*2, 50, &uart1_queue, ESP_INTR_FLAG_LEVEL1); //   ESP_INTR_FLAG_IRAM
            //  (UART_NUM, TX, RX, RTS, CTS) 
    uart_set_pin(UART1,    33, 26,  14, 12);

    xTaskCreate(    uart_interrupt_task, 
                    "uart_interrupt_task", 
                    BUF_SIZE*4, 
                    NULL, 
                    12, 
                    NULL);

    xTaskCreate(    parser_nemea_task, 
                    "parser_nemea_task", 
                    BUF_SIZE*4, 
                    NULL, 
                    12, 
                    NULL);

    uart_transmit(UART1, (const void *)RMC, strlen(RMC));



//uart_interrupt_task();

}



/****************DEFINICIÓN DE FUNCIONES**************************/

static void uart_interrupt_task(void *params)
{
    uart_event_t uart_event;
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    uint8_t *print_data = (uint8_t *)malloc(BUF_SIZE+100);
    while(1)
    {
        uart_transmit(UART1, (const void *)RMC, strlen(RMC));
        if(xQueueReceive(uart1_queue, (void *)&uart_event, (TickType_t)portMAX_DELAY))
        {
            bzero(data, BUF_SIZE);
            bzero(print_data, BUF_SIZE+100);

            switch (uart_event.type)
            {
            case UART_DATA:
                uart_receive(UART1, (void *) data, (uint32_t)uart_event.size);
                sprintf((char *)print_data, "%s", data);
                //uart_transmit(UART0,  "---", 3);
                uart_transmit(UART0,  print_data, uart_event.size);
                
                break;
            
            default:
                break;
            }
            

        }
    }

    free(data);
    free(print_data);

}

static void parser_nemea_task(const char *nmeaString, GNSSData *gnssData){
    // Verificar que la cadena comience con '$'
    if (nmeaString[0] != '$') {
        ESP_LOGW(TAG, "Cadena NMEA no válida, no comienza con '$'");
        return;
    }

    // Utilizamos strtok para dividir la cadena en tokens usando ","
    char *token;
    token = strtok((char *)nmeaString, ",");

    // Comprobamos si el primer token es "$GPRMC"
    if (strcmp(token, "$GPRMC") != 0) {
        ESP_LOGW(TAG, "Cadena NMEA no válida, no es un mensaje GPRMC");
        return;
    }

    // Iteramos a través de los tokens
    for (int i = 1; i < 12; i++) {
        token = strtok(NULL, ",");
        if (token == NULL) {
            ESP_LOGW(TAG, "Cadena NMEA no válida, falta un campo");
            return;
        }
        if (i == 1) {
            // Obtener la hora en formato HHMMSS
            strncpy(gnssData->time, token, 10);
        } else if (i == 3) {
            // Obtener la latitud en formato DDMM.MMMM
            float lat_degrees = atof(token) / 100;
            int lat_minutes = (int)lat_degrees;
            float lat_seconds = (lat_degrees - lat_minutes) * 60;
            gnssData->latitude = lat_minutes + lat_seconds;
        } else if (i == 5) {
            // Obtener la longitud en formato DDDMM.MMMM
            float lon_degrees = atof(token) / 100;
            int lon_minutes = (int)lon_degrees;
            float lon_seconds = (lon_degrees - lon_minutes) * 60;
            gnssData->longitude = lon_minutes + lon_seconds;
        }
    }
}