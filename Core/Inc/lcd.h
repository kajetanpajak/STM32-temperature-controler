/*
 * lcd.h
 *
 *  Created on: Jan 11, 2025
 *      Author: kajpa
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f7xx_hal.h"
#include "stdbool.h"
#include "stdint.h"

#define HI2C_DEF hi2c1

#define RS_PIN 0x01
#define RW_PIN 0x02
#define EN_PIN 0x04
#define BL_PIN 0x08

#define INIT_8_BIT_MODE 0x30
#define INIT_4_BIT_MODE 0x02

#define CLEAR_LCD 0x01

#define UNDERLINE_OFF_BLINK_OFF		0x0C
#define UNDERLINE_OFF_BLINK_ON		0x0D
#define UNDERLINE_ON_BLINK_OFF		0x0E
#define UNDERLINE_ON_BLINK_ON		0x0F

#define FIRST_CHAR_LINE_1	0x80
#define FIRST_CHAR_LINE_2	0xC0

struct lcd_disp {
	uint8_t addr;
	char f_line[17];
	char s_line[17];
	bool bl;
	TIM_HandleTypeDef* Timer;
};

void lcd_init(struct lcd_disp * lcd);
void lcd_write(struct lcd_disp * lcd, uint8_t data, uint8_t xpin);
void lcd_display(struct lcd_disp * lcd);
void lcd_clear(struct lcd_disp * lcd);
void lcd_delay_us(struct lcd_disp * lcd, uint32_t delay_us);

#endif /* INC_LCD_H_ */
