/*
 * display_task.c
 *
 *  Created on: 29 мар. 2018 г.
 *      Author: Кочкин
 */

#include <task_oled.h>
#include "lcd_oled.h"
#include "FreeRTOS.h"
#include "task.h"
#include "oled_strings.h"
#include "stdio.h"
#include "adc_bp_struct.h"
#include "modbus_regmap.h"
#include "usb_host.h"
volatile uint8_t display_state = D_TEST;
volatile uint8_t explicit_oled_state = D_NONE;

/**
 * Displays set of data on display, e.g. temperature of oil, acceleration...
 * @param state state of display #state. Data is acquired from volatile #bps
 */
static void s_display_data(uint8_t state)
{
	RTC_DateTypeDef date;
	char data_buff[16] = {};
	uint8_t len;
	//oled_clr_upper();
	//oled_clr_lower();

	switch (state)
	{

	case D_TEST:
		oled_print_upper("Welcome, user", 13);
		break;
	case D_TGD:
		oled_print_upper(oled_msg_temp_plast, sizeof(oled_msg_temp_plast));
		len = sprintf(data_buff, "%d", hr_temp_plast);
		oled_print_lower_and_time(data_buff, len > 15 ? 15 : len);
		break;
	case D_RPG:
		oled_print_upper(oled_msg_pressure, sizeof(oled_msg_pressure));
		len = sprintf(data_buff, "%d", hr_pressure);
		oled_print_lower_and_time(data_buff, len > 15 ? 15 : len);
		break;
	case D_TMC:
		oled_print_upper(oled_msg_temp_oil, sizeof(oled_msg_temp_oil));
		len = sprintf(data_buff, "%d", hr_temp_oil);
		oled_print_lower_and_time(data_buff, len > 15 ? 15 : len);
		break;
	case D_GX:
		oled_print_upper(oled_msg_accx, sizeof(oled_msg_accx));
		len = sprintf(data_buff, "%d", hr_accx);
		oled_print_lower_and_time(data_buff, len > 15 ? 15 : len);
		break;
	case D_GY:
		oled_print_upper(oled_msg_accy, sizeof(oled_msg_accy));
		len = sprintf(data_buff, "%d", hr_accy);
		oled_print_lower_and_time(data_buff, len > 15 ? 15 : len);
		break;
	case D_GZ:
		oled_print_upper(oled_msg_accz, sizeof(oled_msg_accz));
		len = sprintf(data_buff, "%d", hr_accz);
		oled_print_lower_and_time(data_buff, len > 15 ? 15 : len);
		break;
	case D_TOB:
		oled_print_upper(oled_msg_temp_wiring, sizeof(oled_msg_temp_wiring));
		len = sprintf(data_buff, "%d", hr_temp_wiring);
		oled_print_lower_and_time(data_buff, len > 15 ? 15 : len);
		break;
	case D_RIZ:
		oled_print_upper(oled_msg_resistance, sizeof(oled_msg_resistance));
		len = sprintf(data_buff, "%d", hr_resistance);
		oled_print_lower_and_time(data_buff, len > 15 ? 15 : len);

		break;

	case D_SUCC:
		oled_print_upper(oled_msg_succ, sizeof(oled_msg_succ));

		len = sprintf(data_buff, "%d", hr_pack_number);
		oled_print_lower_and_time(data_buff, len > 15 ? 15 : len);

		break;

	case D_DATE:
		HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
		len = sprintf(data_buff, "%02d.%02d.%d", date.Date, date.Month, date.Year +1900);

		oled_print_upper(data_buff, len);
		oled_print_lower(" ", 1);
		break;
	case D_CONN_PARAMS:
		len = sprintf(data_buff, "%d.%d.%d.%d", hr_bn_ip_h >> 8, hr_bn_ip_h & 0xFF, hr_bn_ip_l >> 8, hr_bn_ip_l & 0xFF);
		oled_print_upper(data_buff, len);
		len = sprintf(data_buff, "MB: %#02X",hr_bn_modbus_addr);
		oled_print_lower(data_buff, len);
		break;
	case D_ERRCNT:
		oled_print_upper(oled_msg_total, sizeof(oled_msg_total));
		len = sprintf(data_buff, "%d", (hr_bn_err_packs + hr_pack_number));
		oled_print_lower_and_time(data_buff, len > 15 ? 15 : len);
		break;
	case D_ERR:
		oled_print_upper(oled_msg_temp_plast, sizeof(oled_msg_temp_plast));

		//s_display_value(r_izol); Not implemented yet
		break;
	default:
		break;
	}

}

void task_oled(void *pvParameters)
{
	char buffer[16];
	char len;
	static uint8_t prev_exp_state;
	oled_lcd_init();

	for(;;)
	{
		if(explicit_oled_state!= D_NONE)
		{
			if (explicit_oled_state != prev_exp_state)
			{
				prev_exp_state = explicit_oled_state;
				oled_clr_all();
			}
			switch(explicit_oled_state)
			{
			case D_USB_ATTACHED:
				oled_print_upper(oled_usb_msg_connected, sizeof(oled_usb_msg_connected));
				len = sprintf(buffer, "V: %04x, P: %04x",hUsbHostFS.device.DevDesc.idVendor, hUsbHostFS.device.DevDesc.idProduct);
				oled_print_lower(buffer, len);
				break;
			case D_USB_WRITING:
				oled_print_upper(oled_usb_msg_writing, sizeof(oled_usb_msg_writing));
				break;
			case D_USB_FINISHED:
				oled_print_upper(oled_usb_msg_finished, sizeof(oled_usb_msg_finished));
				len = sprintf(buffer, "V: %04x, P: %04x",hUsbHostFS.device.DevDesc.idVendor, hUsbHostFS.device.DevDesc.idProduct);
				oled_print_lower(buffer, len);
				break;
			case D_USB_ERROR:
				oled_print_upper(oled_usb_msg_error, sizeof(oled_usb_msg_error));
				break;
			default:
				break;
			}
			vTaskDelay(500);
		}
		else
		{
			if (display_state != D_ERR)
			{
				display_state++;
				if (display_state == D_ERR)
					display_state = D_TMC;

				s_display_data(display_state);
			}
			else
			{
				s_display_data(display_state);
			}
			vTaskDelay(1500);
		}

	}

}
