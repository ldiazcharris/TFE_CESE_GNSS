
/************************INCLUDES*********************************/

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "utilities.h"
#include "lcd_i2c_grove.h"


/************************MACROS***********************************/

#define BUF_SIZE 1024
#define RMC "$GNRMC"
#define JUMP " \n"
#define UART0 UART_NUM_0
#define UART1 UART_NUM_1
#define DELAY_LCD 3000

/**************DECLARACIÓN DE VARIABLES GLOBALES*******************/

static QueueHandle_t uart1_queue;
static GNSSData_t quectel_l76;

/****************DECLARACIÓN DE FUNCIONES*************************/



/**********************FUNCIÓN PRINCIPAL**************************/

void app_main()
{

    lcd_init(); 
    lcd_clear(); 
    lcd_set_RGB(255, 255, 255); //pantalla blanca
    lcd_write(0, 0, "Linea 0");
    lcd_write(1, 0, "Linea 1");
    delay(DELAY_LCD);

    while(1)
    {
        lcd_clear(); 
        lcd_write(0, 0, "Color: ");
        lcd_write(1, 0, "Rojo");
        lcd_set_RGB(255, 0, 0); //pantalla roja

        delay(DELAY_LCD);
        lcd_off();
        delay(DELAY_LCD);
        lcd_on();

        lcd_clear(); 
        lcd_write(0, 0, "Color: ");
        lcd_write(1, 0, "Verde");
        lcd_set_RGB(0, 255, 0); //pantalla verde

        delay(DELAY_LCD);
        lcd_off();
        delay(DELAY_LCD);
        lcd_on();

        lcd_clear(); 
        lcd_write(0, 0, "Color: ");
        lcd_write(1, 0, "Azul");
        lcd_set_RGB(0, 0, 255); //pantalla azul

        delay(DELAY_LCD);
        lcd_off();
        delay(DELAY_LCD);
        lcd_on();

        lcd_clear(); 
        lcd_write(0, 0, "Color: ");
        lcd_write(1, 0, "amarillo");
        lcd_set_RGB(255, 255, 0); //pantalla amarilla

        delay(DELAY_LCD);
        lcd_off();
        delay(DELAY_LCD);
        lcd_on();

        lcd_clear(); 
        lcd_write(0, 0, "Color: ");
        lcd_write(1, 0, "magenta");
        lcd_set_RGB(255, 0, 255); //pantalla magenta

        delay(DELAY_LCD);
        lcd_off();
        delay(DELAY_LCD);
        lcd_on();

        lcd_clear(); 
        lcd_write(0, 0, "Color: ");
        lcd_write(1, 0, "Cyan");
        lcd_set_RGB(0, 255, 255); //pantalla cyan

        delay(DELAY_LCD);
        lcd_off();
        delay(DELAY_LCD);
        lcd_on();
    }
}

/****************DEFINICIÓN DE FUNCIONES**************************/



