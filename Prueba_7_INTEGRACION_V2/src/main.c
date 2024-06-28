
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



/**************DECLARACIÓN DE VARIABLES GLOBALES*******************/

static QueueHandle_t uart0_queue_gnss;
static QueueHandle_t uart1_queue_4g; 
static QueueHandle_t position_queue;
static QueueHandle_t occupancy_queue;
static QueueHandle_t cava_data_queue;
static QueueHandle_t lcd_queue;
static SemaphoreHandle_t uart1_sem;
static SemaphoreHandle_t lcd_sem;
static GNSSData_t quectel_l76;


/****************DECLARACIÓN DE FUNCIONES*************************/

static void init_mqtt_server_task(void *params);
static void gnss_task(void *params);
void IRAM_ATTR occupancy_isr_handler(void* arg);
static void collect_data_task(void *params);
static void transmit_to_server_task(void *params);
static void lcd_task(void *params);

static void create_tasks();
static mqtt_server_state_t init_sequence_mqtt_server(uart_event_t uart1_event, char * at_response);
static mqtt_msg_state_t transmit_msg_mqtt(char * mqtt_payload, char * topic, uart_event_t uart1_event, char * at_response);
static bool wait_PB_DONE(QueueHandle_t uart_queue, uart_event_t uart_event, char * at_response);


/**************************FUNCIÓN PRINCIPAL*******************************/

void app_main()
{
    // UART Para recibir trama NMEA del Modulo L76
    uart_init(UART0, 9600, BUF_SIZE * 2, BUF_SIZE * 2, 50, &uart0_queue_gnss, ESP_INTR_FLAG_LEVEL1);
    //          (UART_NUM, TX, RX, RTS, CTS)
    uart_set_pin(UART0,     1,  3,  23,  19);

    // UART_1 conectar con modulo 4g A7670SA
    uart_init(UART1, 115200, BUF_SIZE * 2, 0, 50, &uart1_queue_4g, ESP_INTR_FLAG_LEVEL1); //   ESP_INTR_FLAG_IRAM
    //          (UART_NUM, TX, RX, RTS, CTS)
    uart_set_pin(UART1,    26, 25,  14,  12);
    //ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, ESP_INTR_FLAG_IRAM));

    // Se configruran los pines donde se conectarán los pilotos de ocupado o desocupado. 
    occupancy_pilots_init();

    // Se configura el pin de habilitación EN_4G_BUTTON, del módulo 4g para controlar reinicios. 
    enable_pin_4g_init();

    // Se configruran los pines donde se conectarán los botones de ocupado o desocupado.

    ocupancy_buttons_init(); 
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(BUSSY_BUTTON, occupancy_isr_handler, (void *)BUSSY_BUTTON);
    gpio_isr_handler_add(FREE_BUTTON, occupancy_isr_handler, (void *)FREE_BUTTON);

    debounce_init();

    // Secuencia de inicialización del LCD
    lcd_init(); 
    lcd_clear(); 
    lcd_set_RGB(0, 0, 255); //LCD color azul

    // Semáforo para arbitrar el uso del puerto UART
    uart1_sem = xSemaphoreCreateBinary();
    lcd_sem = xSemaphoreCreateBinary();

    // Creación de colas que servirán para la comunicación entre tareas. 
    position_queue = xQueueCreate(10, sizeof(GNSSData_t));
    occupancy_queue = xQueueCreate(10, sizeof(occupancy_t));
    cava_data_queue = xQueueCreate(10, sizeof(CAVA_DATA_t));
    lcd_queue = xQueueCreate(10, sizeof(LCD_data_t));

    lcd_write(0, 0, "Init Ok...");

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


// Luego crear una tarea aparte que se encargue de controlar la comunicación UART con el mod 4g

// Hacer secuencia de desconexión del Cliente MQTT
                        // Hacer secuencia de liberación del Cliente MQTT
                        // Hacer secuencia de conexión al servidor nuevamente

static void init_mqtt_server_task(void *params)
{
    xSemaphoreGive(uart1_sem);
    xSemaphoreGive(lcd_sem);
    lcd_write(0, 0, "Init serv task");
    uart_event_t uart1_event;

    char *at_response = (char *)malloc(BUF_SIZE);
    bzero(at_response, BUF_SIZE);
    
    mqtt_server_state_t mqtt_server_state;

    uint8_t try_conection_c = 0;
    
    // Espera a recibir "PB DONE" del modulo 4g que indica que está conectado a la red celular
    
    lcd_write(1, 0, "Waiting PB_DONE");

    wait_PB_DONE(uart1_queue_4g, uart1_event, at_response);

    /*
    while(1)
    {
        if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, portMAX_DELAY))
        {
            uart_receive(UART1, at_response, uart1_event.size);
            lcd_write(1, 0, at_response);
            if (NULL != strstr(at_response,"PB DONE"))
            {
                bzero(at_response, BUF_SIZE);
                break;
            }
            bzero(at_response, BUF_SIZE);
        }
    }
    */
    

    mqtt_server_state = init_sequence_mqtt_server(uart1_event, at_response);
   
    // Si se inicia correctamente la comunicación con el servidor MQTT
    // Entonces se crean las demás tareas. 
    if(MQTT_SERVER_OK == mqtt_server_state)
    {
        create_tasks();
        lcd_clear();
        lcd_write(0, 0, "MQTT Serv OK");
        vTaskDelete(NULL);
    }
    else // Si no, intentará 3 veces la conexión.
    {
        while(try_conection_c < 3)
        {
            mqtt_server_state = init_sequence_mqtt_server(uart1_event, at_response);

            if(MQTT_SERVER_OK == mqtt_server_state)
            {
                create_tasks();
                lcd_clear();
                lcd_write(0, 0, "MQTT Serv OK");
                break;
            }
            try_conection_c++;
        }
        lcd_clear();
        lcd_write(0, 0, "MQTT Serv ERR");
        vTaskDelete(NULL);
    }
}



static void gnss_task(void *params)
{

    xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(100));
    lcd_write(0,0, "GNSS TASK OK");
    xSemaphoreGive(lcd_sem);

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
                    
                    nmea_rmc_parser_r((char *)nmea_string, &quectel_l76);

                    xQueueSend(position_queue, &quectel_l76, pdMS_TO_TICKS(100));
                    
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



static void collect_data_task(void *params)
{
    xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(100));
    lcd_write(0,0, "COLLECT TASK OK");
    xSemaphoreGive(lcd_sem);

    GNSSData_t receive_pos;
    occupancy_t occupancy_state;
    xSemaphoreGive(uart1_sem);
    CAVA_DATA_t cava_data;

    while(1){

        if(xQueueReceive(occupancy_queue, &occupancy_state, pdMS_TO_TICKS(500)))
        {
            cava_data.occupancy = occupancy_state;
        }

        if(xQueueReceive(position_queue, &receive_pos, pdMS_TO_TICKS(500)))
        {
            cava_data.position.NMEA_state = receive_pos.NMEA_state;

            if (cava_data.position.NMEA_state != NMEA_PARSER_OK)
            {
                // Por defecto la posición matriz del centro de distribución.
                cava_data.position.lat = 10.918982886682658;
                cava_data.position.lon = -74.87194240611939;
                strcpy(cava_data.position.time, "33:33");
            }
            else 
            {
                cava_data.position.lat = receive_pos.lat;
                cava_data.position.lon = receive_pos.lon;
                strcpy(cava_data.position.time, receive_pos.time);
            }

        }

        xQueueSend(cava_data_queue, &cava_data, pdMS_TO_TICKS(500));
       
    }
}


// Esta tarea debería tener la más baja prioridad
static void transmit_to_server_task(void *params)
{
    xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(100));
    lcd_write(0,0, "SERV TASK OK");
    xSemaphoreGive(lcd_sem);

    CAVA_DATA_t cava_data;
    uart_event_t uart1_event;
    mqtt_msg_state_t mqtt_msg_state = MQTT_MSG_ERROR;
    LCD_data_t lcd_data;

    // Puntero requerido por la función transmit_msg_mqtt() para procesar mensajes UART
    char * at_response = (char *)malloc(BUF_SIZE);
    char * mqtt_payload = (char *)malloc(100);
    char * topic = CAVA_TOPIC;
    bzero(at_response, BUF_SIZE);
    bzero(mqtt_payload, 100);
    
    while(1){

        if(xQueueReceive(cava_data_queue, &cava_data, portMAX_DELAY))
        {
            sprintf(mqtt_payload, MQTT_PAYLOAD_FORMAT, 
                    cava_data.position.lat, 
                    cava_data.position.lon, 
                    cava_data.occupancy, 
                    cava_data.position.NMEA_state
                    );
            
            // mqtt_msg_state es el mecanismo para saber si hay un fallo en la comunicación MQTT
            xSemaphoreTake(uart1_sem, portMAX_DELAY);
            mqtt_msg_state = transmit_msg_mqtt(mqtt_payload, topic, uart1_event, at_response);
            xSemaphoreGive(uart1_sem);

            memcpy(&lcd_data.cava_data, &cava_data, sizeof(cava_data));

            // lcd_data.cava_data.occupancy = cava_data.occupancy;
            // lcd_data.cava_data.position.lat = cava_data.position.lat;
            // lcd_data.cava_data.position.lon = cava_data.position.lon;
            lcd_data.msg_state = mqtt_msg_state;

            xQueueSend(lcd_queue, &lcd_data, pdMS_TO_TICKS(100));
        }
        delay(2000);
    }
    free(mqtt_payload);
    free(at_response);
}


static void lcd_task(void *params)
{
    xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(100));
    lcd_write(0,0, "LCD TASK OK");
    xSemaphoreGive(lcd_sem);
    
    LCD_data_t lcd_data;
    char print_to_lcd[16];
    char occupancy_str[8];

    bzero(print_to_lcd, 16);
    bzero(occupancy_str, 8);

    while (1)
    {

        if (xQueueReceive(lcd_queue, &lcd_data, portMAX_DELAY))
        {
            
            xSemaphoreTake(lcd_sem, portMAX_DELAY);
            
            lcd_clear();
            sprintf(print_to_lcd, "%.3f, %.3f", lcd_data.cava_data.position.lat, lcd_data.cava_data.position.lon);
            lcd_write(0, 0, print_to_lcd);

            delay(1000);
            
            bzero(print_to_lcd, 16);
            nmea_state_to_str(lcd_data.cava_data.position.NMEA_state, print_to_lcd);
            lcd_write(1, 0, print_to_lcd);

            delay(1000);

            bzero(print_to_lcd, 16);
            bzero(occupancy_str, 8);
            occupancy_to_string(lcd_data.cava_data.occupancy, occupancy_str);
            sprintf(print_to_lcd, "Cava %s", occupancy_str);
            lcd_clear();
            lcd_write(0, 0, print_to_lcd);
            delay(1000);
            
            bzero(print_to_lcd, 16);
            mqtt_msg_state_to_string(lcd_data.msg_state, print_to_lcd);
            lcd_write(1, 0, print_to_lcd);
            delay(1000);

            xSemaphoreGive(lcd_sem);
        }
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
                10,
                NULL);

    xTaskCreate(transmit_to_server_task,
                "transmit_to_server_task",
                BUF_SIZE * 4,
                NULL,
                8,
                NULL);

    xTaskCreate(lcd_task,
                "lcd_task",
                BUF_SIZE * 4,
                NULL,
                8,
                NULL); 
}



void occupancy_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    occupancy_t occupancy;
    switch (gpio_num)
    {
    case BUSSY_BUTTON:
        occupancy = BUSSY_CAVA;
        gpio_set_level(BUSSY_PILOT, 0);
        gpio_set_level(FREE_PILOT, 1);

        break;
    case FREE_BUTTON:
        occupancy = FREE_CAVA;
        gpio_set_level(BUSSY_PILOT, 1);
        gpio_set_level(FREE_PILOT, 0);
        break;
    
    default:
        break;
    }
    xQueueSendFromISR(occupancy_queue, &occupancy, NULL);

}



static mqtt_server_state_t init_sequence_mqtt_server(uart_event_t uart1_event, char * at_response)
{

    mqtt_server_state_t mqtt_server_state = MQTT_SERVER_ERR;

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
        mqtt_server_state = MQTT_FAIL_INIT_SERVICE;
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

        if (NULL == strstr(at_response, "OK"))
        {
            //uart_transmit(UART0, "Fail to get MQTT client\n", strlen("Fail to get MQTT client\n"));
            mqtt_server_state = MQTT_FAIL_ADQ_CLIENT;
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
                mqtt_server_state = MQTT_FAIL_INIT_SERVER;
                bzero(at_response, BUF_SIZE);
                
            }
            else
            {
                //uart_transmit(UART0, "Success Connection to MQTT Server!\n", strlen("Success Connection to MQTT Server!\n"));
                bzero(at_response, BUF_SIZE);
                mqtt_server_state = MQTT_SERVER_OK;
                
            }
        }
    }
    return mqtt_server_state;
}

// Función para transmitir un mensaje por MQTT
static mqtt_msg_state_t transmit_msg_mqtt(char * mqtt_payload, char * topic, uart_event_t uart1_event, char * at_response)
{
    mqtt_msg_state_t msg_state = MQTT_MSG_ERROR;

    size_t payload_len = strlen(mqtt_payload);

    char * mqtt_payload_command = (char *)malloc(32); 
    bzero(mqtt_payload_command, 32);
    sprintf(mqtt_payload_command, CMQTT_PAYLOAD, payload_len);
    
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
                        uart_receive(UART1, at_response, uart1_event.size);

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
    free(mqtt_payload_command);
    return msg_state;
}

static bool wait_PB_DONE(QueueHandle_t uart_queue, uart_event_t uart_event, char * at_response)
{
    while(1)
    {
        if (xQueueReceive(uart_queue, (void *)&uart_event, portMAX_DELAY))
        {
            uart_receive(UART1, at_response, uart_event.size);
            lcd_write(1, 0, at_response);
            if (NULL != strstr(at_response,"PB DONE"))
            {
                bzero(at_response, BUF_SIZE);
                break;
            }
        }
    }
    return true;
}

