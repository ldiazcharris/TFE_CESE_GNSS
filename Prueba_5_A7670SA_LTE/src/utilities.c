#include "utilities.h"
#include "lcd_i2c_grove.h"

// Uncomment for debugging with UART_0
//static const char *TAG = "NMEA_PARSER";

static void error_message_gnss_data(const char *message){
    lcd_clear();
    lcd_cursor(0,0);
    lcd_write_string((char *)"ERR posicion:");
    lcd_cursor(1,0);
    lcd_write_string((char *)*message);
}

void uart_init(  uart_port_t     uart_num, 
                        int             baud_rate, 
                        int             rx_buffer_size, 
                        int             tx_buffer_size, 
                        int             queue_size, 
                        QueueHandle_t   *uart_queue, 
                        int             intr_alloc_flags
                     ){
    
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(    uart_num, 
                                            rx_buffer_size*2, 
                                            tx_buffer_size,
                                            queue_size, 
                                            uart_queue, 
                                            intr_alloc_flags
                                         )
                     );

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
}



void uart_transmit(uart_port_t uart_num, const void *src, size_t size)
{
    uart_write_bytes(uart_num, src, size);
}


void uart_receive(uart_port_t uart_num, void *buf, uint32_t length)
{
    uart_read_bytes(uart_num, buf, length, portMAX_DELAY);
}


void delay(const TickType_t delay_ms)
{
    vTaskDelay( delay_ms / portTICK_PERIOD_MS);
}

void nmea_parser(const char *nmeaString, GNSSData_t *gnssData)
{
    // Verificar que la cadena comience con '$'
    if (nmeaString[0] != '$')
    {
        // Uncomment for debugging with UART_0
        //ESP_LOGW(TAG, "Cadena NMEA no válida, no comienza con '$'");
        char message[] = "falta $";
        error_message_gnss_data(message);
        return;
    }

    // Se utiliza strtok para dividir la cadena en tokens usando "," como divisor
    char *token;
    token = strtok((char *)nmeaString, ",");

    // Comprobar si el primer token es "$GPRMC"
    if (strcmp(token, "$GPRMC") != 0)
    {
        // Uncomment for debugging with UART_0
        //ESP_LOGW(TAG, "Cadena NMEA no válida, no es un mensaje GPRMC");
        char message[] = "falta $GPRMC";
        error_message_gnss_data(message);
        return;
    }

    // Si todo sale bien se itera a través de los tokens
    for (int i = 1; i < 12; i++)
    {
        token = strtok(NULL, ",");
        if (token == NULL)
        {
            // Uncomment for debugging with UART_0
            //ESP_LOGW(TAG, "Cadena NMEA no válida, falta un campo");
            char message[] = "falta un campo";
            error_message_gnss_data(message);
            return;
        }
        if (i == 1)
        {
            // Obtener la hora en formato HHMMSS
            strncpy(gnssData->time, token, 10);
        }
        else if (i == 3)
        {
            // Obtener la latitud en formato DDMM.MMMM
            float lat_degrees = atof(token) / 100;
            int lat_minutes = (int)lat_degrees;
            float lat_seconds = (lat_degrees - lat_minutes) * 60;
            gnssData->latitude = lat_minutes + lat_seconds;
        }
        else if (i == 5)
        {
            // Obtener la longitud en formato DDDMM.MMMM
            float lon_degrees = atof(token) / 100;
            int lon_minutes = (int)lon_degrees;
            float lon_seconds = (lon_degrees - lon_minutes) * 60;
            gnssData->longitude = lon_minutes + lon_seconds;
        }
    }
}



bool nmea_rmc_parser_r(const char *nmeaString, GNSSData_t *gnssData)
{
    bool result_parser = false;
    // Verificar que la cadena comience con '$'
    if (nmeaString[0] != '$')
    {
        // Uncomment for debugging with UART_0
        //ESP_LOGW(TAG, "Cadena NMEA no válida, no comienza con '$'");
        //char message[] = "falta $";
        //error_message_gnss_data(message);
        return result_parser;
    }

    // Se utiliza strtok para dividir la cadena en tokens usando "," como divisor
    char *token;
    char* rest = (char *)nmeaString;
    token = strtok_r((char *)nmeaString, ",", &rest);
    uint8_t data_count = 0;

    // Comprobar si el primer token es "$GPRMC"
    if (strcmp(token, "$GPRMC") != 0)
    {
        // Uncomment for debugging with UART_0
        //ESP_LOGW(TAG, "Cadena NMEA no válida, no es un mensaje GPRMC");
        //char message[] = "falta $GPRMC";
        //error_message_gnss_data(message);
        return result_parser;
    }

    // Si todo sale bien se itera a través de los tokens
    for (int i = 1; i < 12; i++)
    {
        token = strtok_r(NULL, ",", &rest);
        if ((token == NULL)&&(data_count == 3))
        {
            // Uncomment for debugging with UART_0
            //ESP_LOGW(TAG, "Cadena NMEA no válida, falta un campo");
            //char message[] = "falta un campo";
            //error_message_gnss_data(message);
            result_parser = true;
            return result_parser;
        }
        else if((token == NULL))
        {
            return result_parser;
        }

        if (i == 1)
        {
            // Obtener la hora en formato HHMMSS
            strncpy(gnssData->time, token, 10);
            data_count++;
        }
        else if (i == 3)
        {
            // Obtener la latitud en formato DDMM.MMMM
            float lat_degrees = atof(token) / 100;
            int lat_minutes = (int)lat_degrees;
            float lat_seconds = (lat_degrees - lat_minutes) * 60;
            gnssData->latitude = lat_minutes + lat_seconds;
            data_count++;
        }
        else if (i == 5)
        {
            // Obtener la longitud en formato DDDMM.MMMM
            float lon_degrees = atof(token) / 100;
            int lon_minutes = (int)lon_degrees;
            float lon_seconds = (lon_degrees - lon_minutes) * 60;
            gnssData->longitude = lon_minutes + lon_seconds;
            data_count++;
        }
    }
}


void ocupancy_pin_init(gpio_config_t* occupancy_pin_config, uint64_t occupancy_pin)
{
    occupancy_pin_config->intr_type = GPIO_INTR_ANYEDGE;
    occupancy_pin_config->pin_bit_mask = (1ULL << occupancy_pin);
    occupancy_pin_config->mode = GPIO_MODE_INPUT;
    occupancy_pin_config->pull_up_en = GPIO_PULLUP_DISABLE;
    occupancy_pin_config->pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(occupancy_pin_config);
}

void init_pilots()
{    
    gpio_set_direction(BUSY_PILOT, GPIO_MODE_OUTPUT);
    gpio_reset_pin(BUSY_PILOT);
    gpio_set_direction(FREE_PILOT, GPIO_MODE_OUTPUT);
    gpio_reset_pin(FREE_PILOT);
}

void write_position(const char * lat, const char * lon)
{
    //lcd_clear();
    lcd_cursor(0, 0);
    lcd_write_string(lat);
    lcd_cursor(1, 0);
    lcd_write_string(lon);
}

void write_occupancy(bool occupancy_state)
{
                //lcd_clear();
            if(occupancy_state)
            {
                gpio_set_level(FREE_PILOT, 0);
                gpio_set_level(BUSY_PILOT, 1);
                lcd_cursor(0, 11);
                lcd_write_string("occ:SI");
                
            }
            else if(!occupancy_state)
            {
                gpio_set_level(FREE_PILOT, 1);
                gpio_set_level(BUSY_PILOT, 0);
                lcd_cursor(0, 11);
                lcd_write_string("occ:NO");
                
            }
            else
            {
                gpio_set_level(FREE_PILOT, 1);
                gpio_set_level(BUSY_PILOT, 1);
                lcd_cursor(0, 11);
                lcd_write_string("occ:--");
            }
}

bool fmqtt_send_payload(const char * mqtt_payload_to_send)
{
    uart_transmit(UART1, cmqtt_topic, strlen(cmqtt_topic));
    delay(50);
    uart_transmit(UART1, mqtt_topic, strlen(mqtt_topic));
    dalay(50);
    uart_transmit(UART1, cmqtt_payload, strlen(cmqtt_payload));
    delay(50);
    uart_transmit(UART1, mqtt_payload_to_send, strlen(mqtt_payload_to_send));

}