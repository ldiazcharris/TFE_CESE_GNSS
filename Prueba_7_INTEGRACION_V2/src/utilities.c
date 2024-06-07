#include "utilities.h"
#include "lcd_i2c_grove.h"

// Uncomment for debugging with UART_0
//static const char *TAG = "NMEA_PARSER";

static void error_message_gnss_data(const char *message){
    lcd_clear();
    lcd_cursor(0,0);
    lcd_write_string((char *)"ERR posicion:");
    lcd_cursor(1,0);
    lcd_write_string((char *) message);
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
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

NMEA_state_t nmea_rmc_parser_r(char *nmeaString, GNSSData_t *gnssData)
{
    NMEA_state_t result_parser = NMEA_PARSER_ERROR;
    // Verificar que la cadena comience con '$'
    if (NULL == strstr(nmeaString, "$"))
    {
        // Cadena NMEA no válida, no comienza con $
        result_parser = NMEA_FRAME_NO_VALID;
    }
    else
    {
        // Se utiliza strtok para dividir la cadena en tokens usando "," como divisor
        char *token;
        char* rest = (char *)nmeaString;
        token = strtok_r(nmeaString, ",", &rest);
       

        // Comprobar si el primer token contiene "RMC"
        if (NULL == strstr(token, "RMC"))
        {
            // Cadena NMEA no válida, no es un mensaje RMC
            result_parser = NMEA_FRAME_NO_RMC;
            
        }
        else
        {    // Si todo sale bien se itera a través de los tokens
            for (int i = 1; i < 12; i++)
            {
                token = strtok_r(NULL, ",", &rest);
                if (token == NULL)
                {
                    // Cadena NMEA no válida, falta un campo
                    result_parser = NMEA_FRAME_VOID_FIELD;
                    break;
                }

            
                if (i == 1)
                {
                    // Obtener la hora en formato HHMMSS
                    strncpy(gnssData->time, token, 10);
                    
                }
                else if (i == 2)
                {
                    // Obtener la hora en formato HHMMSS
                    if(NULL == strstr(token, "A"))
                    {
                        result_parser = NMEA_FRAME_NO_VALID;
                        break;
                    }
                        
                }
                else if (i == 3)
                {
                    // Obtener la latitud en formato DMS (DDMM.MMMM) y convertir a formato DD
                    float lat_float_1 = atof(token) / 100;
                    int lat_deg = (int)lat_float_1;
                    float lat_float_2 = (lat_float_1 - lat_deg)*100;
                    int lat_min = (int)(lat_float_2);
                    float lat_sec = (lat_float_2 - lat_min)*10;
                    gnssData->lat = (float)lat_deg + ((float)lat_min/60) + (lat_sec/3600);
                    
                }
                else if (i == 4)
                {
                    // Obtener la dirección de latitud Norte o Sur
                    if (0 == strcmp("S", token))
                        gnssData->lat *=-1;
                }
                else if (i == 5)
                {
                    // Obtener la longitud en formato DMS (DDDMM.MMMM) y convertir a formato DD
                    float lon_float_1 = atof(token) / 100;
                    int lon_deg = (int)lon_float_1;
                    float lon_float_2 = (lon_float_1 - lon_deg)*100;
                    int lon_min = (int)(lon_float_2);
                    float lon_sec = (lon_float_2 - lon_min)*10;
                    gnssData->lon = (float)lon_deg + ((float)lon_min/60) + (lon_sec/3600);
                    

                    result_parser = NMEA_PARSER_OK;
                }
                else if (i == 6)
                {
                    // Obtener la dirección de longitud Este u Oeste
                    if (0 == strcmp("W", token))
                        gnssData->lon *=-1;
                    break;
                }
            }
        }
    }
    gnssData->NMEA_state = result_parser;
    return result_parser;
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

void pilots_init()
{    
    gpio_reset_pin(BUSY_PILOT);
    gpio_reset_pin(FREE_PILOT);
    gpio_set_direction(BUSY_PILOT, GPIO_MODE_OUTPUT);
    gpio_set_direction(FREE_PILOT, GPIO_MODE_OUTPUT);
    
}

void write_position(char * lat, char * lon)
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