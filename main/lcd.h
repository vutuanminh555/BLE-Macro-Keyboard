#ifndef LCD_H
#define LCD_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h" // for delay us


#define EN 27
#define RS 14
#define D4 26 
#define D5 25
#define D6 33
#define D7 32
#define BL 13

void LCD_String (char *str); // consider use uint8 t instead of char 
void LCD_String_xy (uint8_t row, uint8_t pos, char *str);
void LCD_Command (uint8_t cmnd);
void LCD_Data (uint8_t data);
void LCD_Init();
void LCD_Deinit();

#endif