#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"


#define BUSY_PILOT            27
#define FREE_PILOT            13
#define FREE_BUTTON           36 
#define BUSSY_BUTTON          39
//#define EN_GNSS_PIN           17
#define EN_4G_PIN             33
#define BOARD_LED              2

#define UART0 UART_NUM_0
#define UART1 UART_NUM_1

// A funturo cambiar a "proyectoLuis/cava001/datos" Para tener un control numérico de las cavas
#define CAVA_TOPIC          "proyectoLuis/cava/datos" 

#define CMQTT_START         "AT+CMQTTSTART\r\n"
#define CMQTT_CLIENT        "AT+CMQTTACCQ=0,\"gnss_cavas\",0\r\n"

// Incluir en el manual como una configuración
#define CMQTT_CONNECT        "AT+CMQTTCONNECT=0,\"tcp://18.212.130.131:1883\",300,0,\"test\",\"CloudTech*\"\r\n"
#define CMQTT_TOPIC          "AT+CMQTTTOPIC=0,23\r\n"
#define CMQTT_PAYLOAD        "AT+CMQTTPAYLOAD=0,%d\r\n"
#define MQTT_PAYLOAD_FORMAT  "{\"lat\":\"%.6f\", \"long\":\"%.6f\", \"occup\":\"%d\"}\r\n"
#define MQTT_PUBLISH         "AT+CMQTTPUB=0,0,60,0,0\r\n"


typedef enum {
    NMEA_PARSER_OK = 0,
    NMEA_FRAME_NO_VALID,
    NMEA_FRAME_NO_RMC,
    NMEA_FRAME_VOID_FIELD,
    NMEA_PARSER_ERROR
} NMEA_state_t;

typedef enum {
    MQTT_MSG_OK = 0,
    MQTT_MSG_FAIL,
    MQTT_TOPIC_OK,
    MQTT_TOPIC_FAIL,
    MQTT_MSG_ERROR
} mqtt_msg_state_t;

typedef enum {
    MQTT_SERVER_OK = 0,
    MQTT_FAIL_INIT_SERVICE,
    MQTT_FAIL_ADQ_CLIENT,
    MQTT_FAIL_INIT_SERVER,
    MQTT_SERVER_ERR
} mqtt_server_state_t;

typedef struct
{
    float lat;
    float lon;
    char time[10];
    NMEA_state_t NMEA_state;
    
} GNSSData_t;


// Analizar Si la posición GNSSData_t puede ser un arreglo de 10 valores. 
typedef struct
{
    bool occupancy;
    GNSSData_t position;

} CAVA_DATA_t;

typedef struct 
{
    CAVA_DATA_t cava_data;
    mqtt_msg_state_t msg_state;
    mqtt_server_state_t server_state;

} LCD_data_t;




/**
 * @brief explican que hace la función
 * 
 * @param baud_rate: 
 * @param 
 * 
 * @return 
*/

void uart_init(  uart_port_t     uart_num, 
                        int             baud_rate, 
                        int             rx_buffer_size, 
                        int             tx_buffer_size, 
                        int             queue_size, 
                        QueueHandle_t   *uart_queue, 
                        int             intr_alloc_flags
                     );
void uart_transmit(uart_port_t uart_num, const void *src, size_t size);

void uart_receive(uart_port_t uart_num, void *buf, uint32_t length);

void delay(const TickType_t delay_ms);



/**
 * @brief Función que permite parsear las cadenas tipo RMC recibidas por el módulo Quectel L76
 *        Es la versión de tipo "reentrant".
 * @param nmeaString: string que priviene del módulo GNSS en formato de trama NMEA, que será parseado para extraer la 
 *                    información necesaria
 * @param gnssData: estructura de datos del tipo GNSSData_t, que permite almacenar hora, latitud, longitud y resultao
 *                  del proceso de parseo
 * 
*/
NMEA_state_t nmea_rmc_parser_r(char *nmeaString, GNSSData_t *gnssData);

/**
 * @brief Esta función permite inicializar un pin GPIO de la ESP32 para usarlo como interrupición
 * @param occupancy_pin_config: structura del tipo  gpio_config_t para inicializar la interrupción por GPIO
 * @param occupancy_pin: Pin que será utilizado para detectar la interrupción.
*/
void ocupancy_pin_init(gpio_config_t* occupancy_pin_config, uint64_t occupancy_pin);

void write_position(char * lat, char * lon);

void write_occupancy(bool occupancy_state);

void pilots_init();

bool mqtt_service_init();

bool fmqtt_send_payload(const char * mqtt_payload_to_send);




/*
UART EVENTS TYPES

    UART_DATA,              /!< UART data event
    UART_BREAK,             /!< UART break event
    UART_BUFFER_FULL,       /!< UART RX buffer full event
    UART_FIFO_OVF,          /!< UART FIFO overflow event
    UART_FRAME_ERR,         /!< UART RX frame error event
    UART_PARITY_ERR,        /!< UART RX parity event
    UART_DATA_BREAK,        /!< UART TX data and break evenT
    UART_PATTERN_DET,       /!< UART pattern detected 

*/