
/**
 * ESP-IDF Framework Library for "Grove-LCD RGB Backlight V4.0"
 * 
 * This library was adapted from diferent source codes: 
 * https://github.com/Seeed-Studio/Grove_LCD_RGB_Backlight
 * https://controllerstech.com/i2c-in-esp32-esp-idf-lcd-1602/ y 
 * https://controllerstech.com/i2c-lcd-in-stm32/
 * 
 * This software is written by ldiazcharris(lddiazcharris@gmail.com) and is licensed 
 * under The MIT License. Check License.txt for more information.
 *  
 * 
*/


#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_LCD_PORT    I2C_NUM_0

// Dirección para la LCD Grove 0x3E
// Dirección Para la LCD Generica con el driver I2C PCF8574 es 0x27
#define I2C_LCD_SLAVE_ADDRESS         0x3E 
#define I2C_LCD_SLAVE_RGB_ADDRESS     0x62
#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_OUTPUT      0x08

#define REG_RED         0x04
#define REG_GREEN       0x03
#define REG_BLUE        0x02


/**
 * @brief lcd_init() incializa las funcionalidades I2C y la configuración de escritura de la pantalla.
 * Utiliza por defecto:
 * El puerto I2C_NUM_0
 * Mode = Master
 * SDA = GPIO_NUM_21
 * SCL = GPIO_NUM_22
 * CLK Speed = 100000
 */
void lcd_init();

/**
 * @brief lcd_write_char() escribe en la pantalla un caracter.
 * 
 * @param character: caracter que se desea escribir
 */
void lcd_write_char(char character);


/**
 * @brief lcd_cursor() Posiciona el cursosr en la fila y columna especificada.
 * 
 * @param row: fila sobre la cual se desea escribir
 * @param col: columna sobre la cual se desea escribir
 */
void lcd_cursor(uint8_t row, uint8_t col);

/**
 * @brief lcd_write_string() escribe en la pantalla un string.
 * 
 * @param str: string que se desea escribir. 
 */
void lcd_write_string(char *str);

/**
 * @brief cd_write() escribe en la pantalla LCD el texto deseado.
 * 
 * @param row: Columna sobre la cual se desea escribir
 * @param column: Fila sobre la cual se desea escribir
 * @param str: Texto que se desea escribir
 */
void lcd_write(uint8_t row, uint8_t column, char *str);

/**
 * @brief lcd_clear() limpia o borra todos los caracteres de la pantalla LCD.
 */
void lcd_clear();

/**
 * @brief lcd_set_RGB() cambia el color de fondo de la pantalla LCD según el modelo de color RGB.
 * 
 * @param r: valor para cantidad de color rojo.
 * @param g: valor para cantidad de color verde.
 * @param b: valor para cantidad de color azul.
 */
void lcd_set_RGB(unsigned char r, unsigned char g, unsigned char b);

/**
 * @brief lcd_on() enciende el LCD.
 */
void lcd_on();

/**
 * @brief lcd_off() apaga el LCD.
 */
void lcd_off();

