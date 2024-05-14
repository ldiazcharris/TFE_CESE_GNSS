
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
static QueueHandle_t uart1_queue_4g; 
static QueueHandle_t position_queue;
static QueueHandle_t occupancy_queue;
static QueueHandle_t cava_data_queue;
static QueueHandle_t lcd_queue;
static SemaphoreHandle_t uart1_sem;
static SemaphoreHandle_t lcd_sem;
static GNSSData_t quectel_l76;




/****************DECLARACIÓN DE FUNCIONES*************************/

static void gnss_task(void *params);
static void transmit_to_server_task(void *params);
static void collect_data_task(void *params);
static void occupancy_task(void *params);
void occupancy_isr_handler(void* arg);
static void lcd_task(void *params);
static void init_mqtt_server_task(void *params);
static mqtt_server_state_t init_sequence_mqtt_server(uart_event_t uart1_event, char * at_response);
static mqtt_msg_state_t transmit_msg_mqtt(char * mqtt_payload, char * topic, uart_event_t uart1_event, char * at_response);
static void create_tasks();
static bool wait_PB_DONE(QueueHandle_t uart_queue, uart_event_t uart_event, char * at_response);


/**************************FUNCIÓN PRINCIPAL*******************************/

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
    lcd_set_RGB(0, 0, 255); //LCD color azul

    // Semáforo para arbitrar el uso del puerto UART
    uart1_sem = xSemaphoreCreateBinary();
    lcd_sem = xSemaphoreCreateBinary();

    // Creación de colas que servirán para la comunicación entre tareas. 
    position_queue = xQueueCreate(10, sizeof(GNSSData_t));
    occupancy_queue = xQueueCreate(10, sizeof(bool));
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

    if(wait_PB_DONE(uart1_queue_4g, uart1_event, at_response))
        mqtt_server_state = init_sequence_mqtt_server(uart1_event, at_response);
   
    // Si se inicia correctamente la comunicación con el servidor MQTT
    // Entonces se crean las demás tareas. 
    if(MQTT_SERVER_OK == mqtt_server_state)
    {
        create_tasks();
        lcd_clear();
        lcd_write(0, 0, "MQTT Serv OK");
        bzero(at_response, BUF_SIZE);
        while(1)
        {
            if(pdTRUE == xSemaphoreTake(uart1_sem, portMAX_DELAY))
            {
                if (xQueueReceive(uart1_queue_4g, (void *)&uart1_event, portMAX_DELAY))
                {
                    uart_receive(UART1, at_response, uart1_event.size);
                    lcd_write(1, 0, at_response);
                    if (NULL != strstr(at_response,"+CMQTTCONNLOST"))
                    {
                        bzero(at_response, BUF_SIZE);
                        break;
                    }
                }

                xSemaphoreGive(uart1_sem);
            }

            delay()
        }
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

                    xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(1000));
                    lcd_write(0, 0, "Receving L76...");
                    xSemaphoreGive(lcd_sem);
                    
                    switch(nmea_rmc_parser_r((const char *)nmea_string, &quectel_l76))
                    {
                    case NMEA_PARSER_OK:
                        xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(1000));
                        lcd_write(0, 0, "Sending Pos...");
                        xSemaphoreGive(lcd_sem);
                        
                        if(xQueueSend(position_queue, &quectel_l76, (TickType_t)10) != pdPASS)
                        {
                            xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(1000));
                            lcd_clear();
                            lcd_write(0, 0, "Err Transm Pos");
                            delay(2000);
                            xSemaphoreGive(lcd_sem);
                            
                        }
                        else{
                            // Imprimir por el LCD la posición y el tópico: 
                            sprintf(lat_string, "%.2f, %.2f",  quectel_l76.lat, quectel_l76.lon);
                            xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(1000));
                            lcd_write(1, 0, lat_string);
                            xSemaphoreGive(lcd_sem);
                        }
                        break;
                        case NMEA_FRAME_NO_VALID:
                             xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(1000));
                            lcd_write(1, 0, "NMEA_NO_VALID");
                            xSemaphoreGive(lcd_sem);
                            break;
                        case NMEA_FRAME_NO_RMC:
                            xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(1000));
                            lcd_write(1, 0, "NMEA_NO_RMC");
                            xSemaphoreGive(lcd_sem);
                            break;
                        case NMEA_FRAME_VOID_FIELD:
                            xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(1000));
                            lcd_write(1, 0, "NMEA_VOID_FIELD");
                            xSemaphoreGive(lcd_sem);
                            break;
                        case NMEA_PARSER_ERROR:
                            xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(1000));
                            lcd_write(1, 0, "NMEA_PARSER_ERR");
                            xSemaphoreGive(lcd_sem);
                            break;
                        default:
                            xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(1000));
                            lcd_write(1, 0, "GNSS TASK ERR");
                            xSemaphoreGive(lcd_sem);
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

/*
static void occupancy_task(void *params)
{
    while (1)
    {
        if (xQueueReceive(occupancy_queue, &occupancy_state, portMAX_DELAY)) {

                write_occupancy(occupancy_state);
                
            }

    }
}
*/

static void collect_data_task(void *params)
{
    xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(100));
    lcd_write(0,0, "COLLECT TASK OK");
    xSemaphoreGive(lcd_sem);

    static GNSSData_t receive_pos;
    static bool occupancy_state;
    xSemaphoreGive(uart1_sem);
    CAVA_DATA_t cava_data;
    // Por defecto la caba desocupada
    cava_data.occupancy = false;
    // Por defecto la posición matriz del centro de distribución. 10.918982886682658, -74.87194240611939
    cava_data.position.lat = 10.918982886682658;
    cava_data.position.lon = -74.87194240611939;

    while(1){

        if(xQueueReceive(position_queue, &receive_pos, pdMS_TO_TICKS(100)))
        {
            cava_data.position.lat = receive_pos.lat;
            cava_data.position.lon = receive_pos.lon;
            // Luego crear una tarea aparte que se encargue de controlar la comunicación UART con el mod 4g
            // Enviar en una Queue el json_pos_mqtt, reemplazar 
            // Hay que tener un mecanismo para comunicar si hay un fallo en la comunicación MQTT
        }

        if(xQueueReceive(occupancy_queue, &occupancy_state, pdMS_TO_TICKS(100)))
        {
            cava_data.occupancy = occupancy_state;
        }

        xQueueSend(cava_data_queue, &cava_data, pdMS_TO_TICKS(100));
        delay(12000);
       
    }
}

static void lcd_task(void *params)
{
    xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(100));
    lcd_write(0,0, "LCD TASK OK");
    xSemaphoreGive(lcd_sem);
    
    LCD_data_t lcd_data;
    char print_to_lcd[16];
    char msg_state_chr[16];
    bzero(print_to_lcd, 16);
    bzero(msg_state_chr, 16);

    while (1)
    {

        if (xQueueReceive(lcd_queue, &lcd_data, portMAX_DELAY))
        {
            switch (lcd_data.msg_state)
            {
                case MQTT_MSG_OK:
                    bzero(msg_state_chr, 16);
                    sprintf(msg_state_chr, "MQTT Msg OK");
                    lcd_set_RGB(0, 255, 0); //LCD color verde
                    break;
                case MQTT_MSG_FAIL:
                    bzero(msg_state_chr, 16);
                    sprintf(msg_state_chr, "MQTT Msg FAIL");
                    lcd_set_RGB(125, 2, 0); //LCD color rojo
                    break;
                case MQTT_TOPIC_OK:
                    bzero(msg_state_chr, 16);
                    sprintf(msg_state_chr, "MQTT Topic OK");
                    lcd_set_RGB(255, 255, 0); //LCD color amarillo
                    break;
                case MQTT_TOPIC_FAIL:
                    bzero(msg_state_chr, 16);
                    sprintf(msg_state_chr, "MQTT Topic FAIL");
                    lcd_set_RGB(125, 2, 0); //LCD color rojo
                    break;
                case MQTT_MSG_ERROR:
                    bzero(msg_state_chr, 16);
                    sprintf(msg_state_chr, "MQTT Msg ERROR");
                    lcd_set_RGB(125, 2, 0); //LCD color rojo
                    break;

                default:
                    bzero(msg_state_chr, 16);
                    sprintf(msg_state_chr, "Msg st not recv");
                    lcd_set_RGB(0, 0, 255); //LCD color azul
                    break;
            }
        }
    xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(1000));
        lcd_clear();

        sprintf(print_to_lcd, "Pos: %.2f, %.2f", lcd_data.cava_data.position.lat, lcd_data.cava_data.position.lon);
        lcd_write(0, 0, print_to_lcd);

        delay(2000);
        lcd_clear();

        lcd_write(1, 0, msg_state_chr);
        delay(2000);
    xSemaphoreGive(lcd_sem);
        
    }
}

// Esta tarea debería tener la más baja prioridad
static void transmit_to_server_task(void *params)
{
    xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(100));
    lcd_write(0,0, "SERV TASK OK");
    delay(2000);
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

        if(xQueueReceive(cava_data_queue, &cava_data, pdMS_TO_TICKS(100)))
        {
            sprintf(mqtt_payload, MQTT_PAYLOAD_FORMAT, cava_data.position.lat, cava_data.position.lon, cava_data.occupancy);
            
            // mqtt_msg_state es el mecanismo para saber si hay un fallo en la comunicación MQTT
            xSemaphoreTake(uart1_sem, pdMS_TO_TICKS(200));
            mqtt_msg_state = transmit_msg_mqtt(mqtt_payload, topic, uart1_event, at_response);
            xSemaphoreGive(uart1_sem);

            xSemaphoreTake(lcd_sem, pdMS_TO_TICKS(100));
            lcd_clear();
            if(MQTT_MSG_OK == mqtt_msg_state)
                lcd_write(0,0, "SERV TASK-MSG OK");
            else
                lcd_write(0,0, "SERV TASK-MSG ER");
            delay(2000);
            xSemaphoreGive(lcd_sem);

            lcd_data.cava_data.occupancy = cava_data.occupancy;

            lcd_data.cava_data.position.lat = cava_data.position.lat;

            lcd_data.cava_data.position.lon = cava_data.position.lon;

            lcd_data.msg_state = mqtt_msg_state;

            

            xQueueSend(lcd_queue, &lcd_data, pdMS_TO_TICKS(100));
        }
    }
    free(mqtt_payload);
    free(at_response);
}


// https://github.com/espressif/esp-idf/blob/v5.2.1/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c

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
/*
    xTaskCreate(occupancy_task,
                "occupancy_task",
                BUF_SIZE * 4,
                NULL,
                12,
                NULL);
*/
    xTaskCreate(lcd_task,
                "lcd_task",
                BUF_SIZE * 4,
                NULL,
                12,
                NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(OCCUPANCY_PIN, occupancy_isr_handler, (void *)OCCUPANCY_PIN);
}

void occupancy_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(occupancy_queue, &gpio_num, NULL);
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

        uart_transmit(UART0, at_response, strlen(at_response));
        uart_wait_tx_done(UART0, 200);

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

/*
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
        }
    }
    

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
*/