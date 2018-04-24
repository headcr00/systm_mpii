/*
 * lcd_oled.h
 *
 *  Created on: 15 мар. 2018 г.
 *      Author: Кочкин
 */

#ifndef LCD_OLED_H_
#define LCD_OLED_H_
#include "stm32f1xx_hal.h"

extern RTC_HandleTypeDef hrtc;


void oled_lcd_init();
void oled_print_upper(const char * buff, uint8_t len);
void oled_print_lower(char * buff, uint8_t len);
void oled_print_lower_and_time(char * buff, uint8_t len);
void oled_clr_all();
void oled_clr_lower();
void oled_clr_upper();

#endif /* LCD_OLED_H_ */
