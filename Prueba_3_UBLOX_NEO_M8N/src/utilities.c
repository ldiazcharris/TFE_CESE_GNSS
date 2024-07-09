#include "utilities.h"

static const char *TAG = "NMEA_PARSER";

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

/**
 * @brief Función que permite parsear las instrucicones recibidad por el módulo Quectel L76.
 * Guarda en el parámetro gnssData el valor de la latitud y longitud obtenido de la cadena
 * RMC que se debe proporcionar a través del parámetro nmeaString.
 * 
 * @param nmeaString: Puntero a char de la cadena recibida por UART dek módulo L76
 * @param gnssData: puntero estructura del tipo GNSSData_t que almacena la latitud
 *                  y longitud de la posición que reporta el módulo L76. 
 * 
*/

void nmea_parser(const char *nmeaString, GNSSData_t *gnssData)
{
    // Verificar que la cadena comience con '$'
    if (nmeaString[0] != '$')
    {
        ESP_LOGW(TAG, "Cadena NMEA no válida, no comienza con '$'");
        return;
    }

    // Se utiliza strtok para dividir la cadena en tokens usando "," como divisor
    char *token;
    token = strtok((char *)nmeaString, ",");

    // Comprobar si el primer token es "$GPRMC"
    if (strcmp(token, "$GPRMC") != 0)
    {
        ESP_LOGW(TAG, "Cadena NMEA no válida, no es un mensaje GPRMC");
        return;
    }

    // Si todo sale bien se itera a través de los tokens
    for (int i = 1; i < 12; i++)
    {
        token = strtok(NULL, ",");
        if (token == NULL)
        {
            ESP_LOGW(TAG, "Cadena NMEA no válida, falta un campo");
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
            gnssData->lat = lat_minutes + lat_seconds;
        }
        else if (i == 5)
        {
            // Obtener la longitud en formato DDDMM.MMMM
            float lon_degrees = atof(token) / 100;
            int lon_minutes = (int)lon_degrees;
            float lon_seconds = (lon_degrees - lon_minutes) * 60;
            gnssData->lon = lon_minutes + lon_seconds;
        }
    }
}

/**
 * @brief Función que permite parsear las cadenas tipo RMC recibidad por el módulo Quectel L76
 *        Es la versión de tipo "reentrant". Guarda en el parámetro gnssData el valor de la latitud
 *        y longitud obtenido de la cadena RMC que se debe proporcionar a través del parámetro nmeaString.
 * 
 * @param nmeaString: Puntero a char de la cadena recibida por UART dek módulo L76
 * @param gnssData: puntero estructura del tipo GNSSData_t que almacena la latitud
 *                  y longitud de la posición que reporta el módulo L76. 
*/

void nmea_rmc_parser_r(const char *nmeaString, GNSSData_t *gnssData)
{
    // Verificar que la cadena comience con '$'
    if (nmeaString[0] != '$')
    {
        ESP_LOGW(TAG, "Cadena NMEA no válida, no comienza con '$'");
        return;
    }

    // Se utiliza strtok para dividir la cadena en tokens usando "," como divisor
    char *token;
    char* rest = (char *)nmeaString;
    token = strtok_r((char *)nmeaString, ",", &rest);

    // Comprobar si el primer token es "$GPRMC"
    if (strcmp(token, "$GPRMC") != 0)
    {
        ESP_LOGW(TAG, "Cadena NMEA no válida, no es un mensaje GPRMC");
        return;
    }

    // Si todo sale bien se itera a través de los tokens
    for (int i = 1; i < 12; i++)
    {
        token = strtok_r(NULL, ",", &rest);
        if (token == NULL)
        {
            ESP_LOGW(TAG, "Cadena NMEA no válida, falta un campo");
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
            gnssData->lat = lat_minutes + lat_seconds;
        }
        else if (i == 5)
        {
            // Obtener la longitud en formato DDDMM.MMMM
            float lon_degrees = atof(token) / 100;
            int lon_minutes = (int)lon_degrees;
            float lon_seconds = (lon_degrees - lon_minutes) * 60;
            gnssData->lon = lon_minutes + lon_seconds;
        }
    }
}

NMEA_state_t nmea_rmc_parser_r_2(char *nmeaString, GNSSData_t *gnssData)
{
    NMEA_state_t result_parser = NMEA_PARSER_ERR;
    // Verificar que la cadena comience con '$'
    if (nmeaString[0] != '$')
    {
        //"Cadena NMEA no válida, no comienza con $
        result_parser = NMEA_NO_VALID;
    }
    else
    {
        // Se utiliza strtok para dividir la cadena en tokens usando "," como divisor
        char *token;
        char* rest = (char *)nmeaString;
        token = strtok_r(nmeaString, ",", &rest);
       

        // Comprobar si el primer token contiebe "RMC"
        if (NULL == strstr(token, "RMC"))
        {
            // Cadena NMEA no válida, no es un mensaje RMC
            result_parser = NMEA_NO_RMC;
        }
        else
        {    // Si todo sale bien se itera a través de los tokens
            for (int i = 1; i < 9; i++)
            {
                char proof_print[50];
                sprintf(proof_print, "Iteracion %d\n", i);
                uart_transmit(UART_NUM_0, proof_print, strlen((const char*)proof_print));
                token = strtok_r(NULL, ",", &rest);
                
                /*
                if (token == NULL)
                {
                    // Cadena NMEA no válida, falta un campo
                    result_parser = NMEA_VOID_FIELD;
                    break;
                }
                */

            
                if (i == 1)
                {
                    // Obtener la hora en formato HHMMSS
                    strncpy(gnssData->time, token, 10);
                    
                }
                else if (i == 2)
                {
                    // Obtener la hora en formato HHMMSS
                    if(NULL == strstr(token, "A"))
                        return NMEA_NO_VALID;
                    
                    break;
                }
                else if (i == 3)
                {
                    // Obtener la latitud en formato DDMM.MMMM
                    double lat_float_1 = atof(token) / 100;
                    int lat_deg = (int)lat_float_1;
                    double lat_float_2 = (lat_float_1 - lat_deg)*100;
                    int lat_min = (int)(lat_float_2);
                    double lat_sec = (lat_float_2 - lat_min)*10;
                    gnssData->lat = (double)lat_deg + ((double)lat_min/60) + (lat_sec/3600);
                    
                }
                else if (i == 4)
                {
                    // Obtener la dirección de latitud Norte o Sur
                    if (0 == strcmp("S", token))
                        gnssData->lat *=-1;
                }
                else if (i == 5)
                {
                    // Obtener la longitud en formato DDDMM.MMMM
                    double lon_float_1 = atof(token) / 100;
                    int lon_deg = (int)lon_float_1;
                    double lon_float_2 = (lon_float_1 - lon_deg)*100;
                    int lon_min = (int)(lon_float_2);
                    double lon_sec = (lon_float_2 - lon_min)*10;
                    gnssData->lon = (double)lon_deg + ((double)lon_min/60) + (lon_sec/3600);
                    

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
    return result_parser;
}


NMEA_state_t nmea_rmc_parser_r_3(char *nmeaString, GNSSData_t *gnssData)
{
    NMEA_state_t result_parser = NMEA_PARSER_ERR;
    // Verificar que la cadena comience con '$'
    if (NULL == strstr(nmeaString, "$"))
    {
        // Cadena NMEA no válida, no comienza con $
        result_parser = NMEA_NO_VALID;
    }
    else
    {
        // Se utiliza strtok para dividir la cadena en tokens usando "," como divisor
        char *token;
        char* rest = (char *)nmeaString;
        token = strtok_r(nmeaString, ",", &rest);
       

        // Comprobar si el primer token contiene "RMC"
        if (NULL == strstr(token, "GNRMC"))
        {
            // Cadena NMEA no válida, no es un mensaje GNRMC
            result_parser = NMEA_NO_RMC;
        }
        else
        {    // Si todo sale bien se itera a través de los tokens
            for (int i = 1; i < 12; i++)
            {
                token = strtok_r(NULL, ",", &rest);

                switch (i)
                {
                case 1:
                    // Obtener la hora en formato HHMMSS
                    strncpy(gnssData->time, token, 10);
                    break;
                case 2:
                    // 
                    if(NULL == strstr(token, "A"))
                        return NMEA_NO_VALID;

					break;

                case 3:
                    // Obtener la latitud en formato DMS (DDMM.MMMM) y convertir a formato DD
                    
                    double lat_float_1 = atof(token) / 100;
                    int lat_deg = (int)lat_float_1;
                    double lat_min = (lat_float_1 - lat_deg)*100;
                    
                    gnssData->lat = (double)lat_deg + ((double)lat_min/60);
                    
                    break;
                
                case 4:
                    // Obtener la dirección de latitud Norte o Sur
                    if (0 == strcmp("S", token))
                        gnssData->lat *=-1;
					break;
					
                case 5:
                    // Obtener la longitud en formato DMS (DDDMM.MMMM) y convertir a formato DD
                    
                    double lon_float_1 = atof(token) / 100;
                    int lon_deg = (int)lon_float_1;
                    double lon_min = (lon_float_1 - lon_deg)*100;
                    
                    gnssData->lon = (double)lon_deg + ((double)lon_min/60);
                    
                    
                    result_parser = NMEA_PARSER_OK;
                    break;
					
                case 6:
                    // Obtener la dirección de longitud Este u Oeste
                    if (0 == strcmp("W", token))
                        gnssData->lon *=-1;
                    break;
				case 9:
                    // Obtener la fecha
                    strncpy(gnssData->date, token, 7);
                    break;

                default:
                    break;
                }
            }
        }
    }
    gnssData->NMEA_state = result_parser;
    return result_parser;
}