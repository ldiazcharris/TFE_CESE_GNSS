
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
#include "esp_sleep.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"


/************************MACROS***********************************/

#define BUF_SIZE    1024
#define TX_0        1
#define RX_0        3
#define RTS_0       23
#define CTS_0       19
#define TX_1        26
#define RX_1        25
#define RTS_1       14
#define CTS_1       12
#define NVS_NAME    "DSleep"
#define KEY_CAVA    "cava_save"



/**************DECLARACIÓN DE VARIABLES GLOBALES*******************/

static QueueHandle_t uart0_queue_gnss;
static QueueHandle_t uart1_queue_4g; 
static QueueHandle_t position_queue;
static QueueHandle_t occupancy_queue;
static QueueHandle_t occupancy_queue_isr;
static QueueHandle_t cava_data_queue;
static QueueHandle_t lcd_queue;
static QueueHandle_t gpio_evt_queue;
static SemaphoreHandle_t uart1_sem;
static SemaphoreHandle_t lcd_sem;
static GNSSData_t quectel_l76;

static CAVA_DATA_t cava_data_saved;
static wakeup_type_t wakeup_type = WAKEUP_FROM_SET;



/****************DECLARACIÓN DE FUNCIONES*************************/

static void init_mqtt_server_task(void *params);
static void gnss_task(void *params);
void IRAM_ATTR occupancy_isr_handler(void* arg);
static void collect_data_task(void *params);
static void transmit_to_server_task(void *params);
static void lcd_task(void *params);
static void button_bridge(void *params);

static void create_tasks(wakeup_type_t wakeup_type, CAVA_DATA_t * cava_data);
static mqtt_server_state_t init_sequence_mqtt_server(uart_event_t uart1_event, char * at_response);
static mqtt_msg_state_t transmit_msg_mqtt(char * mqtt_payload, char * topic, uart_event_t uart1_event, char * at_response);
static bool wait_PB_DONE(QueueHandle_t uart_queue, uart_event_t uart_event, char * at_response);

static void save_cava_state(CAVA_DATA_t * cava_data);
static void get_cava_state(CAVA_DATA_t * cava_data);
static void enter_deep_sleep(CAVA_DATA_t * cava_data);



/**************************FUNCIÓN PRINCIPAL*******************************/

void app_main()
{
    // Se inicializa el non-volatile storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Se verifica la causa del wakeup
    switch (esp_sleep_get_wakeup_cause())
    {

    case ESP_SLEEP_WAKEUP_TIMER:
        lcd_on();
        get_cava_state(&cava_data_saved);
        create_tasks(wakeup_type, &cava_data_saved);
        break;

    default:

        // UART Para recibir trama NMEA del Modulo L76
        uart_init(UART0, 9600, BUF_SIZE * 2, BUF_SIZE * 2, 50, &uart0_queue_gnss, ESP_INTR_FLAG_LEVEL1);
        //          (UART_NUM, TX, RX, RTS, CTS)
        uart_set_pin(UART0, TX_0, RX_0, RTS_0, CTS_0);

        // UART_1 conectar con modulo 4g A7670SA
        uart_init(UART1, 115200, BUF_SIZE * 2, 0, 50, &uart1_queue_4g, ESP_INTR_FLAG_LEVEL1); //   ESP_INTR_FLAG_IRAM
        //          (UART_NUM, TX, RX, RTS, CTS)
        uart_set_pin(UART1, TX_1, RX_1, RTS_1, CTS_1);
        // ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, ESP_INTR_FLAG_IRAM));

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
        lcd_set_RGB(0, 0, 255); // LCD color azul

        // Semáforo para arbitrar el uso del puerto UART
        uart1_sem = xSemaphoreCreateBinary();
        lcd_sem = xSemaphoreCreateBinary();

        // Creación de colas que servirán para la comunicación entre tareas.

        gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
        position_queue = xQueueCreate(10, sizeof(GNSSData_t));
        occupancy_queue = xQueueCreate(10, sizeof(occupancy_t));
        occupancy_queue_isr = xQueueCreate(10, sizeof(occupancy_t));
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
        break;
    }
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
    
    mqtt_server_state = init_sequence_mqtt_server(uart1_event, at_response);
   
    // Si se inicia correctamente la comunicación con el servidor MQTT
    // Entonces se crean las demás tareas. 
    if(MQTT_SERVER_OK == mqtt_server_state)
    {
        create_tasks(wakeup_type, &cava_data_saved);
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
                create_tasks(wakeup_type, &cava_data_saved);
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

                    xQueueSend(position_queue, &quectel_l76, pdMS_TO_TICKS(500));
                    
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
    GNSSData_t receive_pos;
    occupancy_t occupancy_state;
    xSemaphoreGive(uart1_sem);
    CAVA_DATA_t cava_data;

    while(1){

        if(xQueueReceive(occupancy_queue, &occupancy_state, pdMS_TO_TICKS(200)))
        {
            cava_data.occupancy = occupancy_state;
        }

        if(xQueueReceive(position_queue, &receive_pos, pdMS_TO_TICKS(200)))
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
    CAVA_DATA_t cava_data;
    uart_event_t uart1_event;
    mqtt_msg_state_t mqtt_msg_state = MQTT_MSG_ERROR;
    LCD_data_t lcd_data;
    const uint8_t payload_size = 200;

    // Puntero requerido por la función transmit_msg_mqtt() para procesar mensajes UART
    char * at_response = (char *)malloc(BUF_SIZE);
    char * mqtt_payload = (char *)malloc(payload_size);
    char * topic = CAVA_TOPIC;
    bzero(at_response, BUF_SIZE);
    bzero(mqtt_payload, payload_size);
    
    while(1){

        if(xQueueReceive(cava_data_queue, &cava_data, portMAX_DELAY))
        {
            sprintf(mqtt_payload, MQTT_PAYLOAD_FORMAT, 
                    cava_data.position.lat, 
                    cava_data.position.lon, 
                    cava_data.occupancy, 
                    cava_data.position.NMEA_state,
                    CAVA_REF,
                    cava_data.position.time,
                    cava_data.position.date
                    );
            if(NMEA_PARSER_OK == cava_data.position.NMEA_state)
            {
                // mqtt_msg_state es el mecanismo para saber si hay un fallo en la comunicación MQTT
                xSemaphoreTake(uart1_sem, portMAX_DELAY);
                mqtt_msg_state = transmit_msg_mqtt(mqtt_payload, topic, uart1_event, at_response);
                xSemaphoreGive(uart1_sem);
            }

            lcd_data.cava_data = cava_data;
            lcd_data.msg_state = mqtt_msg_state;

            xQueueSend(lcd_queue, &lcd_data, pdMS_TO_TICKS(200));
            bzero(mqtt_payload, payload_size);
        }
        //delay(2000); // Aquí es donde debe ir el comando de ahorro de energía. 
        enter_deep_sleep(&cava_data);
    }
    free(mqtt_payload);
    free(at_response);
}


static void lcd_task(void *params)
{
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
            sprintf(print_to_lcd, "%.4lf,%.4lf", lcd_data.cava_data.position.lat, lcd_data.cava_data.position.lon);
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


static void create_tasks(wakeup_type_t wakeup_type, CAVA_DATA_t * cava_data)
{
    switch (wakeup_type)
    {
    case WAKEUP_FROM_SET:
        xTaskCreate(gnss_task,
                    "gnss_task",
                    BUF_SIZE * 4,
                    NULL,
                    12,
                    NULL);
        
        xTaskCreate(button_bridge,
                    "button_bridge",
                    BUF_SIZE * 4,
                    NULL,
                    15,
                    NULL);

        xTaskCreate(collect_data_task,
                    "transmit_to_server_task",
                    BUF_SIZE * 4,
                    NULL,
                    13,
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
        break;
    
    case WAKEUP_FROM_SLEEP:
        xTaskCreate(gnss_task,
                    "gnss_task",
                    BUF_SIZE * 4,
                    NULL,
                    12,
                    NULL);
        
        xTaskCreate(button_bridge,
                    "button_bridge",
                    BUF_SIZE * 4,
                    NULL,
                    15,
                    NULL);

        xTaskCreate(collect_data_task,
                    "transmit_to_server_task",
                    BUF_SIZE * 4,
                    cava_data,
                    13,
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
        break;
    }

}


void occupancy_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (BUTTON_DOWN == debounce_up(gpio_get_level(gpio_num)))
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR();   
    }
}


static void button_bridge(void *params)
{
    occupancy_t occupancy;
    uint32_t gpio_num;
    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            switch (gpio_num)
            {
            case BUSSY_BUTTON:
                occupancy = BUSSY_CAVA;
                gpio_set_level(BUSSY_PILOT, 1);
                gpio_set_level(FREE_PILOT, 0);

                break;
            case FREE_BUTTON:
                occupancy = FREE_CAVA;
                gpio_set_level(BUSSY_PILOT, 0);
                gpio_set_level(FREE_PILOT, 1);
                break;

            default:
                break;
            }
            xQueueSend(occupancy_queue, &occupancy, portMAX_DELAY);
        }
    }
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

// Función para guardar el estado de la tarea en la NVS
static void save_cava_state(CAVA_DATA_t * cava_data) 
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAME, NVS_READWRITE, &nvs_handle);

    if (ESP_OK == err) 
    {
        err = nvs_set_blob(nvs_handle, KEY_CAVA, (const void *) cava_data, sizeof(CAVA_DATA_t));
        if (ESP_OK == err)  
        {   
            nvs_commit(nvs_handle);
        }
    }
    else 
    {
        return;
    }
    nvs_close(nvs_handle);
}

// Función para recuperar el estado de la tarea de la NVS
static void get_cava_state(CAVA_DATA_t * cava_data) 
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAME, NVS_READWRITE, &nvs_handle);
    
    if (ESP_OK == err) 
    {
        nvs_set_blob(nvs_handle, KEY_CAVA, (const void *) cava_data, sizeof(CAVA_DATA_t));
    }
    else 
    {
        return;
    }
    nvs_close(nvs_handle);
}

// Función para entrar en deep sleep
static void enter_deep_sleep(CAVA_DATA_t * cava_data) {
    save_cava_state(cava_data);
    esp_sleep_enable_timer_wakeup(1000000 * 10);  // Tiempo de sleep: 10 segundos
    wakeup_type = WAKEUP_FROM_SLEEP;
    lcd_off();
    esp_deep_sleep_start();
}
