#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUF_SIZE    1024
#define RMC         "$GNRMC,,,,,,,,,,,,,"
#define UART0       UART_NUM_0
#define UART1       UART_NUM_1

static void uart_init(uart_port_t uart_num);
static void uart_transmit(uart_port_t uart_num, const void *src);
static void uart_receive(uart_port_t uart_num, void *buf);
void delay(const TickType_t delay_ms);



void app_main() 
{
    uart_init(UART0);
    ESP_ERROR_CHECK(uart_set_pin(UART0, 1, 3, 22, 19));
    uart_init(UART1);
    ESP_ERROR_CHECK(uart_set_pin(UART1, 33, 26, 14, 12));
    char receive_data[500];

     while(1)
     {
        uart_transmit(UART1, RMC);
        uart_transmit(UART0, RMC);

        delay(1000);

        uart_receive(UART1, receive_data);
        delay(1000);
        uart_transmit(UART0, (const void *) receive_data); 
        

        delay(1000);
        

     }

}


static void uart_init(uart_port_t uart_num)
{
    
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE*2, 0, 0, NULL, ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
}

static void uart_transmit(uart_port_t uart_num, const void *src)
{
    uart_write_bytes(uart_num, src, strlen(src));
}


static void uart_receive(uart_port_t uart_num, void *buf)
{
    uart_read_bytes(uart_num, buf, 1, portMAX_DELAY);
}

void delay(const TickType_t delay_ms)
{
    vTaskDelay( delay_ms / portTICK_PERIOD_MS);
}
