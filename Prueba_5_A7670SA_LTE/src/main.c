
/************************INCLUDES*********************************/

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "utilities.h"
#include "lcd_i2c_grove.h"
#include "driver/adc.h"

/************************MACROS***********************************/

#define BUF_SIZE 1024
#define RMC "$GNRMC"
#define JUMP " \n"

#define OCCUPANCY_PIN 4 


/**************DECLARACIÓN DE VARIABLES GLOBALES*******************/

static QueueHandle_t uart0_queue_gnss;
static QueueHandle_t uart1_queue_4g;
static QueueHandle_t position_queue;
static QueueHandle_t occupancy_queue;
static SemaphoreHandle_t uart_sem;
static GNSSData_t quectel_l76;
static GNSSData_t receive_pos;
static bool occupancy_state;



/****************DECLARACIÓN DE FUNCIONES*************************/

static void uart_inter_gnss_task(void *params);
static void transmit_to_server_task(void *params);
static void occupancy_detect_task(void *params);
void IRAM_ATTR occupancy_isr_handler(void* arg);

/**********************FUNCIÓN PRINCIPAL**************************/

void app_main()
{
    // UART Para recibir comandos por el teclado
    uart_init(UART0, 115200, BUF_SIZE * 2, BUF_SIZE * 2, 50, &uart0_queue_gnss, ESP_INTR_FLAG_LEVEL1);
    //          (UART_NUM, TX, RX, RTS, CTS)
    uart_set_pin(UART0,     1,  3,  23,  19);

    // UART_1 conectar con modulo 4g A7670SA
    uart_init(UART1, 115200, BUF_SIZE * 2, BUF_SIZE * 2, 50, &uart1_queue_4g, ESP_INTR_FLAG_LEVEL1); //   ESP_INTR_FLAG_IRAM
    //          (UART_NUM, TX, RX, RTS, CTS)
    uart_set_pin(UART1,    33, 26,  14,  12);

    // Se configruran los pines donde se conectarán los pilotos de ocupado o desocupado. 
    init_pilots();

    // Secuencia de inicialización del LCD
    lcd_init(); 
    lcd_clear(); 
    lcd_set_RGB(255, 255, 255);

    // Semáforo para arbitrar el uso del puerto UART
    uart_sem = xSemaphoreCreateBinary();

    // Creación de colas que servirán para la comunicación entre tareas. 
    position_queue = xQueueCreate(10, sizeof(GNSSData_t));
    occupancy_queue = xQueueCreate(10, sizeof(bool));

    xTaskCreate(uart_inter_gnss_task,
                "uart_inter_gnss_task",
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

    xTaskCreate(occupancy_detect_task,
                "occupancy_detect_task",
                BUF_SIZE * 4,
                NULL,
                12,
                NULL);


    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(OCCUPANCY_PIN, occupancy_isr_handler, (void*) OCCUPANCY_PIN);

}

    



/****************DEFINICIÓN DE FUNCIONES**************************/

static void uart_inter_gnss_task(void *params)
{
    uart_event_t uart_event;
    uint8_t *gnss_recv_data = (uint8_t *)malloc(BUF_SIZE*2);
    uint8_t *nmea_string = (uint8_t *)malloc(BUF_SIZE*2);
    char *lat_string = (char *)malloc(50);
    char *lon_string = (char *)malloc(50);
    

    while (1)
    {
        
        if (xQueueReceive(uart0_queue_gnss, (void *)&uart_event, (TickType_t)portMAX_DELAY))
        {
            bzero(gnss_recv_data, BUF_SIZE*2);
            bzero(nmea_string, BUF_SIZE*2);
            bzero(lat_string, 50);
            bzero(lon_string, 50);

            switch (uart_event.type)
            {
            case UART_DATA:
                    uart_receive(UART0, (void *)gnss_recv_data, (uint32_t)uart_event.size);
                    
                    sprintf((char *)nmea_string, "%s", gnss_recv_data);
                    
                    if (nmea_parser_r((const char *)nmea_string, &quectel_l76))
                    {
                        // https://www.freertos.org/a00117.html
                        if(xQueueSend(position_queue, &quectel_l76, (TickType_t)10) != pdPASS)
                        {
                            lcd_cursor(0, 0);
                            lcd_write_string("Err Transm Pos");
                        }
                        else{
                            // Imprimir por el LCD la posición: 
                            sprintf((char *)lat_string, "Lat:%.2f",  quectel_l76.latitude);
                            sprintf((char *)lon_string, "Lon:%.3f", quectel_l76.longitude);
                            write_position(lat_string, lon_string);
                        }
                    }

                break; 

            default:
                break;
            }
        }
    }

    free(gnss_recv_data);
    free(nmea_string);
    free(lat_string);
    free(lon_string);

}

static void transmit_to_server_task(void *params)
{
    uart_event_t uart_event;
    uint8_t *uart_recv_data = (uint8_t *)malloc(BUF_SIZE);
    uint8_t *at_command = (uint8_t *)malloc(BUF_SIZE);
    uint8_t *at_response = (uint8_t *)malloc(BUF_SIZE);
    
    //Tener una varialbe ocupación
    bool occupancy_state;
    GNSSData_t posicion;
    


    char * json_pos_mqtt = (char *)malloc(BUF_SIZE);

    while(1){

        if (xQueueReceive(uart1_queue_4g, (void *)&uart_event, (TickType_t)100/portTICK_PERIOD_MS))
        {
            bzero(uart_recv_data, BUF_SIZE);
            bzero(at_command, BUF_SIZE);
            
            switch (uart_event.type)
            {
            case UART_DATA:
              
                    uart_receive(UART1, (void *)uart_recv_data, (uint32_t)uart_event.size);
                
                    sprintf((char *)at_command, "%s\n\r", uart_recv_data);

                    uart_transmit(UART0, at_command, strlen((const char*)at_command));

                break; 

            default:
                break;
            }

        }

        if(xQueueReceive(position_queue, &receive_pos, (TickType_t)100/portTICK_PERIOD_MS))
        {
            sprintf(json_pos_mqtt, mqtt_payload_format, receive_pos.latitude, receive_pos.longitude);
            fmqtt_send_payload(json_pos_mqtt);
            uart_transmit(uart1_queue_4g, "AT+CMQTTSTART", strlen("AT+CMQTTSTART"));

        
        }

        if(xQueueReceive(occupancy_queue, &occupancy_state, (TickType_t)100/portTICK_PERIOD_MS))
        {
            /*Código para transmitir por MQTT la ocupación
            */
        }

    }
    free(uart_recv_data);
    free(at_command);

}

static void occupancy_detect_task(void *params)
{
    while (1)
    {
        if (xQueueReceive(occupancy_queue, &occupancy_state, portMAX_DELAY)) {

                write_occupancy(occupancy_state);
                
            }

    }
}

// https://github.com/espressif/esp-idf/blob/v5.2.1/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c

void IRAM_ATTR occupancy_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(occupancy_queue, &gpio_num, NULL);
}
