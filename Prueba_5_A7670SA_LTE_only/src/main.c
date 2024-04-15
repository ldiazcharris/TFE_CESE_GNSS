
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

/************************MACROS***********************************/

#define BUF_SIZE 1024
#define RMC "$GNRMC"
#define JUMP " \n"
#define UART0 UART_NUM_0
#define UART1 UART_NUM_1
#define OCCUPANCY_PIN 4 

typedef enum {
    MQTT_MSG_OK = 0,
    MQTT_MSG_FAIL,
    MQTT_TOPIC_OK,
    MQTT_TOPIC_FAIL,
    MQTT_ERROR
} mqtt_msg_state_t;

typedef enum {
    MQTT_SERVER_OK = 0,
    MQTT_FAIL_INIT_SERVICE,
    MQTT_FAIL_ADQ_CLIENT,
    MQTT_FAIL_INIT_SERVER
} mqtt_server_state_t;


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
static char * init_sequence_mqtt_server(uart_event_t uart1_event, char * at_response, char * mqtt_server_state);
static mqtt_msg_state_t transmit_msg_mqtt(float lat, float lon, bool occu, uart_event_t uart1_event, char * at_response);

/**********************FUNCIÓN PRINCIPAL**************************/

void app_main()
{
    // UART Para recibir comandos por el teclado
    uart_init(UART0, 115200, BUF_SIZE * 2, BUF_SIZE * 2, 50, &uart0_queue_gnss, ESP_INTR_FLAG_LEVEL1);
    //          (UART_NUM, TX, RX, RTS, CTS)
    uart_set_pin(UART0,     1,  3,  23,  19);

    // UART_1 conectar con modulo 4g A7670SA
    uart_init(UART1, 115200, BUF_SIZE * 2, 0, 50, &uart1_queue_4g, ESP_INTR_FLAG_LEVEL1); //   ESP_INTR_FLAG_IRAM
    //          (UART_NUM, TX, RX, RTS, CTS)
    uart_set_pin(UART1,    33, 26,  14,  12);

    uart_set_rx_full_threshold(UART1, 100);

    // Se configruran los pines donde se conectarán los pilotos de ocupado o desocupado. 
    pilots_init();

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
                12,
                NULL);

    xTaskCreate(transmit_to_server_task_1,
                "transmit_to_server_task",
                BUF_SIZE * 6,
                NULL,
                12,
                NULL);
/*

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
                    sprintf((char *)lat_string, "Lat:%.2f",  quectel_l76.lat);
                    sprintf((char *)lon_string, "Lon:%.3f", quectel_l76.lon);
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
    char *comparacion = (char *)malloc(BUF_SIZE);
    bzero(comparacion, BUF_SIZE);
    char *mqtt_server_state = (char *)malloc(25);
    bzero(mqtt_server_state, 25);

    mqtt_msg_state_t msg_state;

    gpio_reset_pin(2);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
    // mqtt_activate_server();

    while (1)
    {

       //xSemaphoreTake(uart_sem, 2000/portTICK_PERIOD_MS);
    
    

        if (xQueueReceive(uart0_queue_gnss, (void *)&uart0_event, 2000/portTICK_PERIOD_MS))
        {
            bzero(uart_recv_data, BUF_SIZE);
            bzero(at_command, BUF_SIZE);

            switch (uart0_event.type)
            {
            case UART_DATA:

                if (cont == 0)
                {
                    gpio_set_level(2, 1);
                    cont++;
                }
                else
                {
                    gpio_set_level(2, 0);
                    cont = 0;
                }
                uart_receive(UART0, (void *)uart_recv_data, (uint32_t)uart0_event.size);

                sprintf((char *)at_command, "%s", uart_recv_data);
                uart_transmit(UART0, at_command, strlen(at_command));

                uart_transmit(UART1, at_command, strlen((const char *)at_command));

                /*

                if(strstr(at_command, "1") != NULL){
                    uart_transmit(UART0, "llego 1\n", strlen("llego 1\n"));

                    //bzero(at_command, BUF_SIZE);
                    //msg_state = transmit_msg_mqtt(10.960548, -74.854128, true, uart0_event, at_command);
                    
                    if(MQTT_MSG_OK == msg_state)
                    {
                        mqtt_server_state = "MSG MQTT SEND OK";
                    }
                    

                   uart_transmit(UART1, at_command, strlen((const char *)at_command));

                    
                }
                else{
                    uart_transmit(UART1, at_command, strlen((const char *)at_command));
                }   
                */

                break;

            default:
                break;
            }
        }

        //xSemaphoreGive(uart_sem);
        
    }

    free(uart_recv_data);
    free(at_command);
}

static void transmit_to_server_task_1(void *params)
{
    //xSemaphoreGive(uart_sem);
    uart_event_t uart1_event;
    char *at_response = (char *)malloc(BUF_SIZE);
    bzero(at_response, BUF_SIZE);
    
    char *comparacion = (char *)malloc(BUF_SIZE);
    bzero(comparacion, BUF_SIZE);
    char *mqtt_server_state = (char *)malloc(25);
    bzero(mqtt_server_state, 25);

    mqtt_msg_state_t msg_state;

    ///delay(10000);
    

    while(1)
    {
        
        if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, portMAX_DELAY / portTICK_PERIOD_MS))
        {
            uart_receive(UART1, at_response, uart1_event.size);
            if (NULL != strstr(at_response,"PB DONE"))
            {
                bzero(at_response, BUF_SIZE);
                break;
            }
        }
    }

    //xSemaphoreTake(uart_sem, portMAX_DELAY);
    mqtt_server_state = init_sequence_mqtt_server(uart1_event, at_response, mqtt_server_state);
    //xSemaphoreGive(uart_sem);

    sprintf(comparacion, "State conection to MQTT Server: %s\n", mqtt_server_state);
    uart_transmit(UART0, comparacion, strlen(comparacion));

    delay(2000);

    bzero(at_response, BUF_SIZE);
    msg_state = transmit_msg_mqtt(10.960548, -74.854128, true, uart1_event, at_response);

    if(MQTT_MSG_OK == msg_state)
    {
        mqtt_server_state = "MSG MQTT SEND OK";
    }

    while (1)
    {
        sprintf(comparacion, "State conection to MQTT Server: %s\n", mqtt_server_state);
        uart_transmit(UART0, comparacion, strlen(comparacion));
        delay(2000);

      
        if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, portMAX_DELAY / portTICK_PERIOD_MS))
        {
            bzero(at_response, BUF_SIZE);

            switch (uart1_event.type)
            {
            case UART_DATA:

                uart_receive(UART1, (void *)at_response, (uint32_t)uart1_event.size);

                uart_transmit(UART0, at_response, strlen((const char *)at_response));

                break;

            default:
                break;
            }
        }

    
    }
    free(at_response);
    free(comparacion);
    free(mqtt_server_state);
}

static char * init_sequence_mqtt_server(uart_event_t uart1_event, char * at_response, char * mqtt_server_state)
{
    // Iniciar servicio MQTT en el módulo SIM A7670SA
    uart_transmit(UART1, CMQTT_START, strlen(CMQTT_START));
    uart_wait_tx_done(UART1, 200);

    if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, portMAX_DELAY / portTICK_PERIOD_MS))
    {
        uart_receive(UART1, at_response, uart1_event.size);
    }

    if (NULL == strstr(at_response, "OK"))
    {
        uart_transmit(UART0, "Fail to Start MQTT Service\n", strlen("Fail to Start MQTT Service\n"));
        mqtt_server_state = "FAIL MQTT Service";
        bzero(at_response, BUF_SIZE);
        
    }
    else
    {
        uart_transmit(UART0, at_response, strlen(at_response));

        uart_transmit(UART0, "Adquiring MQTT Client...\n", strlen("Adquiring MQTT Client...\n"));

        bzero(at_response, BUF_SIZE);

        uart_transmit(UART1, CMQTT_CLIENT, strlen(CMQTT_CLIENT));

        uart_wait_tx_done(UART1, 200);

        if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, portMAX_DELAY / portTICK_PERIOD_MS))
        {
            uart_receive(UART1, at_response, uart1_event.size);
        }

        uart_transmit(UART0, at_response, strlen(at_response));
        uart_wait_tx_done(UART0, 200);

        if (NULL == strstr(at_response, "OK"))
        {
            uart_transmit(UART0, "Fail to get MQTT client\n", strlen("Fail to get MQTT client\n"));
            bzero(at_response, BUF_SIZE);
            mqtt_server_state = "FAIL MQTT Client";
            
        }
        else
        {

            bzero(at_response, BUF_SIZE);
            uart_transmit(UART1, CMQTT_CONNECT, strlen(CMQTT_CONNECT));
            uart_wait_tx_done(UART1, 200);

            if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, portMAX_DELAY / portTICK_PERIOD_MS))
            {
                uart_receive(UART1, at_response, uart1_event.size);

                if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, portMAX_DELAY / portTICK_PERIOD_MS))
                {
                    uart_receive(UART1, at_response, uart1_event.size);
                }
            }

            uart_transmit(UART0, at_response, strlen(at_response));
            uart_wait_tx_done(UART0, 200);

            if (NULL == strstr(at_response, "OK"))
            {
                uart_transmit(UART0, "Fail to connect MQTT Server\n", strlen("Fail to connect MQTT Server\n"));
                bzero(at_response, BUF_SIZE);
                mqtt_server_state = "FAIL MQTT Server";
                
            }
            else
            {
                uart_transmit(UART0, "Success Connection to MQTT Server!\n", strlen("Success Connection to MQTT Server!\n"));
                bzero(at_response, BUF_SIZE);
                mqtt_server_state = "OK";
                
            }
        }
    }
    return mqtt_server_state;
}


static mqtt_msg_state_t transmit_msg_mqtt(float lat, float lon, bool occu, uart_event_t uart1_event, char * at_response)
{
    uart_transmit(UART0, "Iniciando send message\n", strlen("Iniciando send message\n"));
    uart_wait_tx_done(UART0, 200);
    mqtt_msg_state_t msg_state = MQTT_ERROR;
    char * mqtt_payload = (char *)malloc(BUF_SIZE);
    bzero(mqtt_payload, BUF_SIZE);
    char * mqtt_payload_command = (char *)malloc(32);
    bzero(mqtt_payload_command, 32);
    size_t payload_len;

    uart_transmit(UART0, "variables inicializadas\n", strlen("variables inicializadas\n"));
    uart_wait_tx_done(UART0, 200);


    if(NULL == mqtt_payload){
        uart_transmit(UART0, "Malloc falló en mqtt_payload\n", strlen("Malloc falló en mqtt_payload\n"));
        uart_wait_tx_done(UART0, 200);
    }
    if(NULL == mqtt_payload_command){
        uart_transmit(UART0, "Malloc falló en mqtt_payload_command\n", strlen("Malloc falló en mqtt_payload_command\n"));
        uart_wait_tx_done(UART0, 200);
    }

    // Configurar el tópico SIM A7670SA 
    uart_transmit(UART1, CMQTT_TOPIC, strlen(CMQTT_TOPIC));
    uart_wait_tx_done(UART1, 200);

    uart_transmit(UART0, "Se envio CMQTT_TOPIC\n", strlen("Se envio CMQTT_TOPIC\n"));
    uart_wait_tx_done(UART0, 200);

    bzero(at_response, BUF_SIZE);

    if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12000)))
    {
        sprintf(at_response, "len recived after CMQTT_TOPIC %d\n", uart1_event.size);

        uart_transmit(UART0, at_response, strlen(at_response));
        uart_wait_tx_done(UART0, 200);
        uart_transmit(UART0, "1 Se recibio luego de CMQTT_TOPIC\n", strlen("1 Se recibio luego de CMQTT_TOPIC\n"));

        bzero(at_response, BUF_SIZE);
        uart_wait_tx_done(UART0, 200);
        uart_receive(UART1, at_response, uart1_event.size);
        //bzero(at_response, BUF_SIZE);
        //sprintf(at_response, "len recived > hola\n");
        //uart_flush(UART1);
        //uart_transmit(UART0, "2 Se recibio luego de CMQTT_TOPIC\n", strlen("2 Se recibio luego de CMQTT_TOPIC\n"));
    }

    if (NULL == strstr(at_response, ">"))
    {
        uart_transmit(UART0, "Fail CMQTT Topic Command\n", strlen("Fail CMQTT Topic Command\n"));
        uart_wait_tx_done(UART0, 200);
        msg_state = MQTT_TOPIC_FAIL;
        bzero(at_response, BUF_SIZE);
        
    }
    else
    {
        uart_transmit(UART0, "Enviando TOPIC\n", strlen("Enviando TOPIC\n"));
        uart_wait_tx_done(UART0, 200);
        // Si todo sale bien, se envía el tópico
        bzero(at_response, BUF_SIZE);

        uart_transmit(UART1, MQTT_TOPIC, strlen(MQTT_TOPIC));
        uart_wait_tx_done(UART1, 200);

        uart_transmit(UART0, MQTT_TOPIC, strlen(MQTT_TOPIC));
        uart_wait_tx_done(UART0, 200);

        if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12000)))
        {
            uart_transmit(UART0, "Se recibio luego de TOPIC\n", strlen("Se recibio luego de TOPIC\n"));
            uart_wait_tx_done(UART0, 200);
            uart_receive(UART1, at_response, uart1_event.size);
            uart_transmit(UART0, at_response, uart1_event.size);
            uart_wait_tx_done(UART0, 200);
        }

        if (NULL == strstr(at_response, "OK"))
        {
            uart_transmit(UART0, "Fail sending MQTT Topic\n", strlen("Fail sending MQTT Topic\n"));
            uart_wait_tx_done(UART0, 200);
            msg_state = MQTT_TOPIC_FAIL;
            bzero(at_response, BUF_SIZE);
        }
        // Si el topico se envío correctamente, se carga el payload
        else
        {
            uart_transmit(UART0, "Enviando CMQTT_PAYLOAD\n", strlen("Enviando CMQTT_PAYLOAD\n"));
            uart_wait_tx_done(UART0, 200);

            bzero(at_response, BUF_SIZE);
        //         (                                 , lat, lon, occu),
            sprintf(mqtt_payload, MQTT_PAYLOAD_FORMAT, lat, lon, occu);

            payload_len = strlen(mqtt_payload);

            sprintf(mqtt_payload_command, CMQTT_PAYLOAD, payload_len);
            //sprintf(mqtt_payload_command, CMQTT_PAYLOAD, 4);

            uart_transmit(UART1, mqtt_payload_command, strlen(mqtt_payload_command));
            uart_wait_tx_done(UART1, 200);

            uart_transmit(UART0, mqtt_payload_command, strlen(mqtt_payload_command));
            uart_wait_tx_done(UART0, 200);

            if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12000)))
            {
                uart_transmit(UART0, "Se recibio luego de CMQTT_PAYLOAD\n", strlen("Se recibio luego de CMQTT_PAYLOAD\n"));
                uart_wait_tx_done(UART0, 200);
                uart_receive(UART1, at_response, uart1_event.size);
            }

            if (NULL == strstr(at_response, ">"))
            {
                uart_transmit(UART0, "Fail CMQTT Payload Command\n", strlen("Fail CMQTT Payload Command\n"));
                uart_wait_tx_done(UART0, 200);
                msg_state = MQTT_MSG_FAIL;
                bzero(at_response, BUF_SIZE);
                
            }
            else
            {
                uart_transmit(UART0, "Enviando PAYLOAD\n", strlen("Enviando PAYLOAD\n"));
                uart_wait_tx_done(UART0, 200);
                bzero(at_response, BUF_SIZE);
                // Se envía el payload
                
                uart_transmit(UART1, mqtt_payload, payload_len);
                uart_wait_tx_done(UART1, 200);

                //uart_transmit(UART0, "Hola", 4);
                uart_transmit(UART0, mqtt_payload, payload_len);
                uart_wait_tx_done(UART0, 200);

                if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12000)))
                {
                    uart_transmit(UART0, "Se recibio luego de PAYLOAD\n", strlen("Se recibio luego de PAYLOAD\n"));
                    uart_wait_tx_done(UART0, 200);
                    uart_receive(UART1, at_response, uart1_event.size);
                }

                if (NULL == strstr(at_response, "OK"))
                {
                    uart_transmit(UART0, "Fail CMQTT Payload sending\n", strlen("Fail CMQTT Payload sending\n"));
                    uart_wait_tx_done(UART0, 200);
                    msg_state = MQTT_MSG_FAIL;
                    bzero(at_response, BUF_SIZE);
                }
                else
                {
                    uart_transmit(UART1, MQTT_PUBLISH, strlen(MQTT_PUBLISH));
                    uart_wait_tx_done(UART1, 200);


                    if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12000)))
                    {
                        uart_receive(UART1, at_response, uart1_event.size);
                    }

                    if (NULL == strstr(at_response, "OK"))
                    {
                        uart_transmit(UART0, "Fail CMQTT Publish\n", strlen("Fail CMQTT Publish\n"));
                        uart_wait_tx_done(UART0, 200);
                        msg_state = MQTT_MSG_FAIL;
                        bzero(at_response, BUF_SIZE);
                    }
                    else
                    {
                        uart_transmit(UART0, "MQTT Publish OK\n", strlen("MQTT Publish OK\n"));
                        uart_wait_tx_done(UART0, 200);
                        msg_state = MQTT_MSG_OK;
                        bzero(at_response, BUF_SIZE);
                    }

                    
                    
                }
            }
        }
    }
    free(mqtt_payload);
    free(mqtt_payload_command);
    return msg_state;
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




