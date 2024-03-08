#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"


#define BUSY_PILOT 15
#define FREE_PILOT 3

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

typedef struct
{
    float latitude;
    float longitude;
    char time[10];
} GNSSData_t;

/**
 * @brief Función que permite parsear las cadenas tipo RMC recibidad por el módulo Quectel L76.
 * @param nmeaString: string que priviene del módulo GNSS, que será parseado para estraer la información necesaria
 * @param gnssData: estructura de datos del tipo GNSSData_t, que permite almacenar la latitud y la longitud
 * 
*/
void nmea_parser(const char *nmeaString, GNSSData_t *gnssData);

/**
 * @brief Función que permite parsear las cadenas tipo RMC recibidad por el módulo Quectel L76
 *        Es la versión de tipo "reentrant".
 * @param nmeaString: string que priviene del módulo GNSS, que será parseado para estraer la información necesaria
 * @param gnssData: estructura de datos del tipo GNSSData_t, que permite almacenar la latitud y la longitud
 * 
*/
void nmea_rmc_parser_r(const char *nmeaString, GNSSData_t *gnssData);

/**
 * @brief Esta función permite inicializar un pin GPIO de la ESP32 para usarlo como interrupición
 * @param occupancy_pin_config: structura del tipo  gpio_config_t para inicializar la interrupción por GPIO
 * @param occupancy_pin: Pin que será utilizado para detectar la interrupción.
*/
void ocupancy_pin_init(gpio_config_t* occupancy_pin_config, uint64_t occupancy_pin);

void write_position(const char * lat, const char * lon);

void write_occupancy(bool occupancy_state);

void init_pilots();



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