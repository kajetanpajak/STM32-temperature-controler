/*
 * lcd.c
 *
 *  Created on: Jan 11, 2025
 *      Author: kajpa
 */

#include "lcd.h"
#include "i2c.h"
#include "stm32f7xx_hal.h"

void lcd_init(struct lcd_disp *lcd)
{
	uint8_t xpin = 0;
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	lcd_delay_us(lcd, 40000);
	lcd_write(lcd, INIT_8_BIT_MODE, xpin);
	lcd_delay_us(lcd, 5000);
	lcd_write(lcd, INIT_8_BIT_MODE, xpin);
	lcd_delay_us(lcd, 1000);
	lcd_write(lcd, INIT_8_BIT_MODE, xpin);

	lcd_write(lcd, INIT_4_BIT_MODE, xpin);

	lcd_write(lcd, UNDERLINE_OFF_BLINK_OFF, xpin);

	lcd_clear(lcd);
}

void lcd_write(struct lcd_disp * lcd, uint8_t data, uint8_t xpin)
{
	uint8_t addr = lcd->addr;
	uint8_t tx_data[4];

	/* split data */
	tx_data[0] = (data & 0xF0) | EN_PIN | xpin;
	tx_data[1] = (data & 0xF0) | xpin;
	tx_data[2] = (data << 4) | EN_PIN | xpin;
	tx_data[3] = (data << 4) | xpin;

	/* send data via i2c */
	HAL_I2C_Master_Transmit(&HI2C_DEF, addr, tx_data, 4, 100);
	lcd_delay_us(lcd, 5000);
}

void lcd_display(struct lcd_disp * lcd)
{
	uint8_t xpin = 0, i = 0;

	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	lcd_clear(lcd);

	/* send first line data */
	lcd_write(lcd, FIRST_CHAR_LINE_1, xpin);
	while(lcd->f_line[i])
	{
		lcd_write(lcd, lcd->f_line[i], (xpin | RS_PIN));
		i++;
	}

	/* send second line data */
	i = 0;
	lcd_write(lcd, FIRST_CHAR_LINE_2, xpin);
	while(lcd->s_line[i])
	{
		lcd_write(lcd, lcd->s_line[i], (xpin | RS_PIN));
		i++;
	}
}

void lcd_clear(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;

	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	/* clear display */
	lcd_write(lcd, CLEAR_LCD, xpin);
}

void lcd_delay_us(struct lcd_disp * lcd, uint32_t delay_us)
{
  __HAL_TIM_SET_COUNTER(lcd->Timer, 0);
  HAL_TIM_Base_Start(lcd->Timer);
  while(__HAL_TIM_GET_COUNTER(lcd->Timer) < delay_us);
  HAL_TIM_Base_Stop(lcd->Timer);
}
