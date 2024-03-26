
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
#define UART0 UART_NUM_0
#define UART1 UART_NUM_1
#define OCCUPANCY_PIN 4 


/**************DECLARACIÓN DE VARIABLES GLOBALES*******************/

static QueueHandle_t uart0_queue_gnss;
static QueueHandle_t uart1_queue_4g;
static QueueHandle_t position_queue;
static QueueHandle_t occupancy_queue;
static SemaphoreHandle_t uart_sem;
static GNSSData_t quectel_l76;
static GNSSData_t receive_pos_4g;
static bool occupancy_state;



/****************DECLARACIÓN DE FUNCIONES*************************/

static void uart_inter_gnss_task(void *params);
static void transmit_to_server_task(void *params);
static void transmit_to_server_task_1(void *params);
static void occupancy_detect_task(void *params);
void IRAM_ATTR occupancy_isr_handler(void* arg);
static void mqtt_activate_server();

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
    /*
    lcd_init(); 
    lcd_clear(); 
    lcd_set_RGB(255, 255, 255);
    */

    // Semáforo para arbitrar el uso del puerto UART
    uart_sem = xSemaphoreCreateBinary();

    // Creación de colas que servirán para la comunicación entre tareas. 
    position_queue = xQueueCreate(10, sizeof(GNSSData_t));
    occupancy_queue = xQueueCreate(10, sizeof(bool));


    xTaskCreate(transmit_to_server_task,
                "transmit_to_server_task",
                BUF_SIZE * 6,
                NULL,
                8,
                NULL);

    xTaskCreate(transmit_to_server_task_1,
                "transmit_to_server_task",
                BUF_SIZE * 6,
                NULL,
                9,
                NULL);
/*
    xTaskCreate(mqtt_activate_server,
                "transmit_to_server_task",
                BUF_SIZE * 4,
                NULL,
                10,
                NULL);


    xTaskCreate(uart_inter_gnss_task,
                "uart_inter_gnss_task",
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
*/


}

    



/****************DEFINICIÓN DE FUNCIONES**************************/

static void uart_inter_gnss_task(void *params)
{
    uart_event_t uart_event;
    uint8_t *uart1_gnss_recv_data = (uint8_t *)malloc(BUF_SIZE*5);
    uint8_t *nmea_string = (uint8_t *)malloc(BUF_SIZE + 100);
    char *lat_string = (char *)malloc(100);
    char *lon_string = (char *)malloc(100);
    

    while (1)
    {
        
        if (xQueueReceive(uart1_queue_4g, (void *)&uart_event, (TickType_t)portMAX_DELAY))
        {
            bzero(uart1_gnss_recv_data, BUF_SIZE*5);
            bzero(nmea_string, BUF_SIZE + 100);
            bzero(lat_string, 100);
            bzero(lon_string, 100);

            switch (uart_event.type)
            {
            case UART_DATA:
                    uart_receive(UART1, (void *)uart1_gnss_recv_data, (uint32_t)uart_event.size);
                    sprintf((char *)nmea_string, "%s\n\r", uart1_gnss_recv_data);
                    nmea_parser((const char *)nmea_string, &quectel_l76);

                    // https://www.freertos.org/a00117.html
                    if(xQueueSend(position_queue, &quectel_l76, (TickType_t)10) != pdPASS)
                    {
                        lcd_cursor(0, 0);
                        lcd_write_string("ErrTransmPos");
                    }

                    // Imprimir por el LCD la posición: 
                    sprintf((char *)lat_string, "Lat:%.2f",  quectel_l76.latitude);
                    sprintf((char *)lon_string, "Lon:%.3f", quectel_l76.longitude);
                    write_position(lat_string, lon_string);

                break; 

            default:
                break;
            }
        }
    }

    free(uart1_gnss_recv_data);
    free(nmea_string);
    free(lat_string);
    free(lon_string);

}

static void transmit_to_server_task(void *params)
{
    uart_event_t uart0_event;
    char *uart_recv_data = (char *)malloc(BUF_SIZE);
    char *at_command = (char *)malloc(BUF_SIZE);
    uint8_t cont = 0;

    gpio_reset_pin(2);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    //mqtt_activate_server();

    while(1){

        if (xQueueReceive(uart0_queue_gnss, (void *)&uart0_event, (TickType_t)portMAX_DELAY))
        {
            
            bzero(uart_recv_data, BUF_SIZE);
            bzero(at_command, BUF_SIZE);
            
            switch (uart0_event.type)
            {
            case UART_DATA:

                    if (cont == 0){
                        gpio_set_level(2, 1);
                        cont++;
                    }else{
                        gpio_set_level(2, 0);
                        cont = 0;
                    }
                   
                    uart_receive(UART0, (void *)uart_recv_data, (uint32_t)uart0_event.size);
                                    
                    sprintf((char *)at_command, "%s", uart_recv_data);

                    int command = atoi((const char *)at_command);

                    if(command == 1){
                        bzero(uart_recv_data, BUF_SIZE);
                        bzero(at_command, BUF_SIZE); 
                        uart_transmit(UART1, "AT+CMQTTSTART\n\r", 15);
                        //uart_receive(UART1, (void *)uart_recv_data, (uint32_t)uart0_event.size);
                        //sprintf((char *)at_command, "%s", uart_recv_data);
                        //uart_transmit(UART0, (void*)at_command, (uint32_t)uart0_event.size);  
                    }
                    else
                    {
                        uart_transmit(UART0, at_command, strlen((const char*)at_command));
                    }

                    //uart_transmit(UART1, at_command, strlen((const char*)at_command));

                    

                break; 

            default:
                break;
            }

            

        }

       
        /*

        if(xQueueReceive(position_queue, &receive_pos_4g, portMAX_DELAY))
        {
            
        }

        if(xQueueReceive(occupancy_queue, &occupancy_state, portMAX_DELAY))
        {
            
        }
        */

    }
    
    free(uart_recv_data);
    free(at_command);
}

static void transmit_to_server_task_1(void *params)
{
    uart_event_t uart1_event;
    char *uart_recv_data = (char *)malloc(BUF_SIZE);
    char *at_response = (char *)malloc(BUF_SIZE);
    char * comparacion = (char *)malloc(BUF_SIZE);

    uint8_t cont = 0;

    gpio_reset_pin(2);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);

    while(1){

        if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, (TickType_t)portMAX_DELAY))
        {
            
            bzero(uart_recv_data, BUF_SIZE);
            bzero(at_response, BUF_SIZE);
            bzero(comparacion, BUF_SIZE);

               if (cont == 0){
                        gpio_set_level(2, 1);
                        cont++;
                    }else{
                        gpio_set_level(2, 0);
                        cont = 0;
                    }
            
            switch (uart1_event.type)
            {
            case UART_DATA:
              
                    uart_receive(UART1, (void *)uart_recv_data, (uint32_t)uart1_event.size);
                
                    sprintf(at_response, "%s", uart_recv_data);
                    uart_transmit(UART0, at_response, strlen((const char*)at_response));
                    
                    sprintf(comparacion, "len data: %d, len expected response %d\n", (int)strlen(at_response), (int)strlen("AT+CMQTTSTART\r\nOK\r\n\r\n\r\n+CMQTTSTART: 0\n"));
                    uart_transmit(UART0, comparacion, strlen((const char*)comparacion));
                    
                    //uart_transmit(UART1, "AT+CMQTTSTART\n\r", 15);
                    //mqtt_activate_server();



                    

                    /*
                    if(strcmp((const char *)at_response, "AT+CMQTTSTART\r\nOK\r\n\r\n\r\n+CMQTTSTART: 0\n"))
                    {
                        //sprinf("len data: %d, len expected response %d\n", (int)strlen(uart_recv_data), (int)strlen("AT+CMQTTSTART\r\nOK\r\n\r\n\r\n+CMQTTSTART: 0\r\n"));
                        
                        uart_transmit(UART0, "AT FAIL\n", 8);
                    }
                    else
                    {
                        uart_transmit(UART0, "AT OK\n", 6);
                    }
                    */
                    

                break; 

            default:
                break;
            }

            

        }
        /*

        if(xQueueReceive(position_queue, &receive_pos_4g, portMAX_DELAY))
        {
            
        }

        if(xQueueReceive(occupancy_queue, &occupancy_state, portMAX_DELAY))
        {
            
        }
        */

    }
    free(uart_recv_data);
    free(at_response);
    free(comparacion);
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



static void mqtt_activate_server()
{

    uart_event_t uart_event;
    char *uart_recv_data = (char *)malloc(BUF_SIZE);
    char *at_response = (char *)malloc(BUF_SIZE);
    char *token;
    char *last = (char *)malloc(BUF_SIZE);
    
    // Iniciar servicios MQTT en el módulo SIM A7670SA
    uart_transmit(UART0, "AT+CMQTTSTART\n\r", 15);
    uart_transmit(UART1, "AT+CMQTTSTART\n\r", 15);

    xSemaphoreTake(uart_sem, portMAX_DELAY);
    if (xQueueReceive(uart1_queue_4g, (void *)&uart_event, (TickType_t)portMAX_DELAY))
    {
        uart_receive(UART1, uart_recv_data, uart_event.size);

        sprintf(at_response, "%s", uart_recv_data);

        token = strtok_r(at_response, "\n", &last);

        while (token != NULL)
        {
            /// printf("%s\n", token);

            if (strcmp(token, "OK\r") == 0)
            {
                printf(" CMQTTSTART recibi Ok\n");
                break;
            }

            token = strtok_r(NULL, "\n", &last);
        }
    }
    xSemaphoreGive(uart_sem);

    uart_transmit(UART0, "AT+CMQTTACCQ=0,\"gnss_cavas\",0\n\r", 31);
    uart_transmit(UART1, "AT+CMQTTACCQ=0,\"gnss_cavas\",0\n\r", 31);

    xSemaphoreTake(uart_sem, portMAX_DELAY);
    if (xQueueReceive(uart1_queue_4g, (void *)&uart_event, (TickType_t)portMAX_DELAY))
    {
        uart_receive(UART1, uart_recv_data, uart_event.size);
        
        sprintf(at_response, "%s", uart_recv_data);

        token = strtok_r(at_response, "\n", &last);

        while (token != NULL)
        {
            /// printf("%s\n", token);

            if (strcmp(token, "OK\r") == 0)
            {
                printf("CMQTTACCQ recibi Ok\n");
                break;
            }

            token = strtok_r(NULL, "\n", &last);
        }
    }
    xSemaphoreGive(uart_sem);

}
