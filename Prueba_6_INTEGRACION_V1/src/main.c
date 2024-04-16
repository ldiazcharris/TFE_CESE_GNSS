
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
//#include "driver/adc.h"

/************************MACROS***********************************/

#define BUF_SIZE 1024
#define RMC "$GNRMC"
#define JUMP " \n"

#define OCCUPANCY_PIN 4 
#define ENABLE_4G_PIN 2

/**************DECLARACIÓN DE VARIABLES GLOBALES*******************/

static QueueHandle_t uart0_queue_gnss;
static QueueHandle_t uart1_queue_4g; // Descomentar si se quiere usar las interrupciones de UART1
static QueueHandle_t position_queue;
static QueueHandle_t occupancy_queue;
static SemaphoreHandle_t uart_sem;
static GNSSData_t quectel_l76;
static GNSSData_t receive_pos;
static bool occupancy_state;



/****************DECLARACIÓN DE FUNCIONES*************************/

static void gnss_task(void *params);
static void transmit_to_server_task(void *params);
static void collect_data_task(void *params);
static void occupancy_task(void *params);
void IRAM_ATTR occupancy_isr_handler(void* arg);
static void lcd_task(void *params);
static void init_mqtt_server_task(void *params);
static char * init_sequence_mqtt_server(uart_event_t uart1_event, char * at_response, char * mqtt_server_state);
static void create_tasks();


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
    //ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, ESP_INTR_FLAG_IRAM));

    // Se configruran los pines donde se conectarán los pilotos de ocupado o desocupado. 
    pilots_init();

    // Se configura el pin de habilitación ENABLE_4G_PIN, del módulo 4g para controlar reinicios. 
    gpio_reset_pin(ENABLE_4G_PIN);
    gpio_set_direction(ENABLE_4G_PIN, GPIO_MODE_OUTPUT_OD);
    gpio_set_pull_mode(ENABLE_4G_PIN, GPIO_PULLUP_ONLY);
    


    // Secuencia de inicialización del LCD
    lcd_init(); 
    lcd_clear(); 
    lcd_set_RGB(255, 255, 255);

    // Semáforo para arbitrar el uso del puerto UART
    uart_sem = xSemaphoreCreateBinary();

    // Creación de colas que servirán para la comunicación entre tareas. 
    position_queue = xQueueCreate(10, sizeof(GNSSData_t));
    occupancy_queue = xQueueCreate(10, sizeof(bool));

/// Crear una tarea que se dedique unicamente a escribir por el LCD. Leer Queues de estado y actualizar la data.
// Estado comunicación GNSS y 4G y reportar el estado de Ocupación y posición.
// crear una variable global que se actualice en el LCD. 

    xTaskCreate(init_mqtt_server_task,
                "init_mqtt_server_task",
                BUF_SIZE * 4,
                NULL,
                12,
                NULL);
}



/****************DEFINICIÓN DE FUNCIONES Y TAREAS**************************/


static void init_mqtt_server_task(void *params)
{
    xSemaphoreGive(uart_sem);
    uart_event_t uart1_event;

    char *at_response = (char *)malloc(BUF_SIZE);
    bzero(at_response, BUF_SIZE);
    
    char *mqtt_server_state = (char *)malloc(25);
    bzero(mqtt_server_state, 25);

    uint8_t try_conection_c = 0;
    
    // Espera a recibir "PB DONE" del modulo 4g que indica que está conectado a la red celular
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

    mqtt_server_state = init_sequence_mqtt_server(uart1_event, at_response, mqtt_server_state);
   

    // Si se inicia correctamente la comunicación con el servidor MQTT
    // Entonces se crean las demás tareas. 
    if(!strcmp(mqtt_server_state, "OK"))
    {
        create_tasks();
        vTaskDelete(NULL);
    }
    else // Si no, intentará 3 veces la conexión.
    {
        while(try_conection_c < 3)
        {
            
            mqtt_server_state = init_sequence_mqtt_server(uart1_event, at_response, mqtt_server_state);

            if(!strcmp(mqtt_server_state, "OK"))
            {
                create_tasks();
                vTaskDelete(NULL);
                break;
            }

            try_conection_c++;
        }

        
        lcd_clear();
        lcd_cursor(0, 0);
        lcd_write_string("MQTT Serv State:");
        lcd_cursor(0, 0);
        lcd_write_string(mqtt_server_state);
    }
}




static void create_tasks()
{
    xTaskCreate(gnss_task,
                "gnss_task",
                BUF_SIZE * 4,
                NULL,
                12,
                NULL);

    xTaskCreate(collect_data_task,
                "transmit_to_server_task",
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

    xTaskCreate(occupancy_task,
                "occupancy_task",
                BUF_SIZE * 4,
                NULL,
                12,
                NULL);

    xTaskCreate(lcd_task,
                "lcd_task",
                BUF_SIZE * 4,
                NULL,
                12,
                NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(OCCUPANCY_PIN, occupancy_isr_handler, (void *)OCCUPANCY_PIN);
}

static void gnss_task(void *params)
{
    uart_event_t uart_event;
    uint8_t *gnss_recv_data = (uint8_t *)malloc(BUF_SIZE*2);
    uint8_t *nmea_string = (uint8_t *)malloc(BUF_SIZE*2);
    char *lat_string = (char *)malloc(50);
    char *lon_string = (char *)malloc(50);
    

    while (1)
    {
        
        if (xQueueReceive(uart0_queue_gnss, (void *)&uart_event, pdMS_TO_TICKS(portMAX_DELAY)))
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
                    
                    if (nmea_rmc_parser_r((const char *)nmea_string, &quectel_l76))
                    {
                        // https://www.freertos.org/a00117.html
                        if(xQueueSend(position_queue, &quectel_l76, (TickType_t)10) != pdPASS)
                        {
                            lcd_cursor(0, 0);
                            lcd_write_string("Err Transm Pos");
                        }
                        else{
                            // Imprimir por el LCD la posición y el tópico: 
                            sprintf((char *)lat_string, "Lat:%.2f",  quectel_l76.lat);
                            sprintf((char *)lon_string, "Lon:%.3f", quectel_l76.lon);
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


static void occupancy_task(void *params)
{
    while (1)
    {
        if (xQueueReceive(occupancy_queue, &occupancy_state, portMAX_DELAY)) {

                write_occupancy(occupancy_state);
                
            }

    }
}


static void collect_data_task(void *params)
{

}

static void lcd_task(void *params)
{

}


// Esta tarea debería tener la más baja prioridad
static void transmit_to_server_task(void *params)
{
    uart_event_t uart_event;
    uint8_t *uart_recv_data = (uint8_t *)malloc(BUF_SIZE);
    uint8_t *at_command = (uint8_t *)malloc(BUF_SIZE);
    uint8_t *at_response = (uint8_t *)malloc(BUF_SIZE);
    xSemaphoreGive(uart_sem);
    CAVA_DATA_t cava_data;
    cava_data.occupacion = false;
    // Colocar la posición matriz del centro de distribución. 
    cava_data.posicion.lat = 0;

    char * json_pos_mqtt = (char *)malloc(BUF_SIZE);

    while(1){


        bzero(uart_recv_data, BUF_SIZE);
        bzero(at_command, BUF_SIZE);

    

        //uart_receive(UART0, (void *)uart_recv_data, (uint32_t)uart_event.size);

        sprintf((char *)at_command, "%s\n\r", uart_recv_data);

        uart_transmit(UART1, at_command, strlen((const char *)at_command));

        if(xQueueReceive(position_queue, &receive_pos, (TickType_t)100/portTICK_PERIOD_MS))
        {
            cava_data.posicion.lat = receive_pos.lat;
            cava_data.posicion.lon = receive_pos.lon;
            
            
            //sprintf(json_pos_mqtt, mqtt_payload_format, receive_pos.lat, receive_pos.lon);

            // Probar así primeramente
            //fmqtt_send_payload(json_pos_mqtt, topic);

            // Luego crear una tarea aparte que se encargue de controlar la comunicación UART con el mod 4g
            // Enviar en una Queue el json_pos_mqtt, reemplazar 
            // Hay que tener un mecanismo para comunicar si hay un fallo en la comunicación MQTT

        
        }

        if(xQueueReceive(occupancy_queue, &occupancy_state, (TickType_t)100/portTICK_PERIOD_MS))
        {
            /*Código para transmitir por MQTT la ocupación
            */
           //Actualizar cava_data.ocupacion y transmitir a la tarea de transmisión MQTT.

        }

    }
    free(uart_recv_data);
    free(at_command);

}


// https://github.com/espressif/esp-idf/blob/v5.2.1/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c

void IRAM_ATTR occupancy_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(occupancy_queue, &gpio_num, NULL);
}

static char * init_sequence_mqtt_server(uart_event_t uart1_event, char * at_response, char * mqtt_server_state)
{
     // Iniciar servicio MQTT en el módulo SIM A7670SA
    uart_transmit(UART1, CMQTT_START, strlen(CMQTT_START));
    uart_wait_tx_done(UART1, 200);

    if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12050)))
    {
        uart_receive(UART1, at_response, uart1_event.size);
    }

    if (NULL == strstr(at_response, "OK"))
    {
        //uart_transmit(UART0, "Fail to Start MQTT Service\n", strlen("Fail to Start MQTT Service\n"));
        mqtt_server_state = "FAIL MQTT Service";
        bzero(at_response, BUF_SIZE);
        
    }
    else
    {
        bzero(at_response, BUF_SIZE);
        uart_transmit(UART1, CMQTT_CLIENT, strlen(CMQTT_CLIENT));
        uart_wait_tx_done(UART1, 200);
        if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12050)))
        {
            uart_receive(UART1, at_response, uart1_event.size);
        }

        uart_transmit(UART0, at_response, strlen(at_response));
        uart_wait_tx_done(UART0, 200);

        if (NULL == strstr(at_response, "OK"))
        {
            //uart_transmit(UART0, "Fail to get MQTT client\n", strlen("Fail to get MQTT client\n"));
            mqtt_server_state = "FAIL MQTT Client";
            bzero(at_response, BUF_SIZE);
            
        }
        else
        {

            bzero(at_response, BUF_SIZE);
            uart_transmit(UART1, CMQTT_CONNECT, strlen(CMQTT_CONNECT));
            uart_wait_tx_done(UART1, 200);

            if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12050)))
            {
                uart_receive(UART1, at_response, uart1_event.size);

                if (NULL == strstr(at_response, "OK"))
                {
                    if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12050)))
                    {
                        uart_receive(UART1, at_response, uart1_event.size);
                    }
                }
            }

            if (NULL == strstr(at_response, "OK"))
            {
                //uart_transmit(UART0, "Fail to connect MQTT Server\n", strlen("Fail to connect MQTT Server\n"));
                mqtt_server_state = "FAIL MQTT Server";
                bzero(at_response, BUF_SIZE);
                
            }
            else
            {
                //uart_transmit(UART0, "Success Connection to MQTT Server!\n", strlen("Success Connection to MQTT Server!\n"));
                bzero(at_response, BUF_SIZE);
                mqtt_server_state = "OK";
                
            }
        }
    }
    return mqtt_server_state;
}


// float lat, float lon, bool occu
static mqtt_msg_state_t transmit_msg_mqtt(char * payload, char * topic, uart_event_t uart1_event, char * at_response)
{
    mqtt_msg_state_t msg_state = MQTT_ERROR;
    char * mqtt_payload = (char *)malloc(BUF_SIZE);
    bzero(mqtt_payload, BUF_SIZE);
    size_t payload_len;
    char * mqtt_payload_command = (char *)malloc(32);
    bzero(mqtt_payload_command, 32);
    


   // Configurar el tópico SIM A7670SA 
    uart_transmit(UART1, CMQTT_TOPIC, strlen(CMQTT_TOPIC));
    uart_wait_tx_done(UART1, 200);
    bzero(at_response, BUF_SIZE);

    if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12000)))
        uart_receive(UART1, at_response, uart1_event.size);

    if (NULL == strstr(at_response, ">"))
    {
        msg_state = MQTT_TOPIC_FAIL;
        bzero(at_response, BUF_SIZE);
    }
    else
    {
        // Si todo sale bien, se envía el tópico
        bzero(at_response, BUF_SIZE);

        uart_transmit(UART1, topic, strlen(topic));
        uart_wait_tx_done(UART1, 200);

        if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12000)))
            uart_receive(UART1, at_response, uart1_event.size);

        if (NULL == strstr(at_response, "OK"))
        {
            msg_state = MQTT_TOPIC_FAIL;
            bzero(at_response, BUF_SIZE);
        }
        // Si el topico se envío correctamente, se carga el payload
        else
        {
            bzero(at_response, BUF_SIZE);
            sprintf(mqtt_payload, MQTT_PAYLOAD_FORMAT, payload);
            payload_len = strlen(mqtt_payload);

            sprintf(mqtt_payload_command, CMQTT_PAYLOAD, payload_len);

            uart_transmit(UART1, mqtt_payload_command, strlen(mqtt_payload_command));
            uart_wait_tx_done(UART1, 200);

            if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12000)))
                uart_receive(UART1, at_response, uart1_event.size);
            
            if (NULL == strstr(at_response, ">"))
            {
                msg_state = MQTT_MSG_FAIL;
                bzero(at_response, BUF_SIZE);
            }
            else
            {
                bzero(at_response, BUF_SIZE);
                // Se envía el payload
                uart_transmit(UART1, mqtt_payload, payload_len);
                uart_wait_tx_done(UART1, 200);

                if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, pdMS_TO_TICKS(12000)))
                    uart_receive(UART1, at_response, uart1_event.size);

                if (NULL == strstr(at_response, "OK"))
                {
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
                        msg_state = MQTT_MSG_FAIL;
                        bzero(at_response, BUF_SIZE);
                    }
                    else
                    {
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