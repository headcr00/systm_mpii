/*
 * oled_strings.h
 *
 *  Created on: 27 мар. 2018 г.
 *      Author: Кочкин
 */

#ifndef OLED_STRINGS_H_
#define OLED_STRINGS_H_

const char oled_msg_welcome[] = {0xA0, 0xA1, 0xE0};


const char oled_msg_succ[] = {168, 112, 184, 189, 199, 191, 111, 32, 190, 97, 186, 101, 191, 111, 179};	//Принято
const char oled_msg_total[] = {66, 99, 101, 180, 111, 32, 190, 112, 184, 189, 199, 191, 111}; //Всего пакетов
const char oled_msg_temp_plast[] = {84, 101, 188, 190, 46, 32, 190, 187, 97, 99, 191, 97, 44, 32, 0xef, 67}; //T гр пласта
const char oled_msg_pressure[] = {224, 97, 179, 187, 101, 189, 184, 101, 44, 32, 97, 191}; //Давление
const char oled_msg_temp_oil[] = {84, 101, 188, 190, 46, 32, 188, 97, 99, 187, 97, 44, 32, 0xEF, 67}; //Темп масла
const char oled_msg_accx[] = {79, 88, 32, 66, 184, 178, 112, 46, 44, 32, 103}; //Уск
const char oled_msg_accy[] = {79, 89, 32, 66, 184, 178, 112, 46, 44, 32, 103};
const char oled_msg_accz[] = {79, 90, 32, 66, 184, 178, 112, 46, 44, 32, 103};
const char oled_msg_temp_wiring[] = {84, 101, 188, 190, 46, 32, 189, 97, 99, 111, 99, 97, 44, 32, 0xef, 67}; //Т обм
const char oled_msg_resistance[] = {67, 111, 190, 112, 46, 32, 184, 183, 111, 187, 46, 44, 32, 79, 188}; //сопр


const char oled_usb_msg_connected[] = {168, 111, 227, 186, 187, 198, 192, 101, 189, 32, 85, 83, 66};
const char oled_usb_msg_writing[] = {164, 97, 190, 184, 99, 196, 32, 97, 112, 120, 184, 179, 97, 46, 46, 46};
const char oled_usb_msg_finished[] = {164, 97, 190, 184, 99, 196, 32, 111, 186, 111, 189, 192, 101, 189, 97};
const char oled_usb_msg_error[] = {79, 193, 184, 178, 186, 97, 32, 183, 97, 190, 184, 99, 184};
#endif /* OLED_STRINGS_H_ */
