/*
 * task_hr_handler.c
 *
 *  Created on: 13 апр. 2018 г.
 *      Author: Кочкин
 */
#include "modbus_regmap.h"
#include "lcd_oled.h"
#include "stm32f1xx_hal.h"
#include "shv_modbus.h"
#include "task_hr_handler.h"
#include "time.h"
#include "stdlib.h"
static void reg_check_time()
{
	time_t time;
	struct tm * timeinfo;
	RTC_DateTypeDef datehal;
	RTC_TimeTypeDef timehal;

	if (hr_bn_time_l != 0 || hr_bn_time_h != 0)
	{
		time = hr_bn_time_h << 16 | hr_bn_time_l;
		timeinfo = localtime(&time);

		datehal.Date = timeinfo->tm_mday;
		datehal.Month = timeinfo->tm_mon + 1;
		datehal.Year = timeinfo->tm_year;

		timehal.Hours = timeinfo->tm_hour;
		timehal.Minutes = timeinfo->tm_min;

		HAL_RTC_SetTime(&hrtc, &timehal, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&hrtc, &datehal, RTC_FORMAT_BIN);
		HAL_RTC_GetTime(&hrtc, &timehal, RTC_FORMAT_BIN);
				  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2,(uint32_t*) &datehal);
		hr_bn_time_l = 0;
		hr_bn_time_h = 0;
	}
}

static void reg_check_ip()
{
	static uint32_t prev_ip;
	static uint32_t prev_gw;
	static uint32_t prev_nm;
	static uint32_t prev_dns;

	uint8_t check = 0;
	if (((hr_bn_ip_h << 16) | hr_bn_ip_l) != prev_ip)
	{
		vTaskDelay(1);
		prev_ip = (hr_bn_ip_h << 16) | hr_bn_ip_l;
		mb.ip_addr.ip1 = hr_bn_ip_h >> 8;
		mb.ip_addr.ip2 = hr_bn_ip_h & 0xFF;
		mb.ip_addr.ip3 = hr_bn_ip_l >> 8;
		mb.ip_addr.ip4 = hr_bn_ip_l & 0xFF;
		check = 1;

	}

	if (((hr_bn_ipgw_h << 16) | hr_bn_ipgw_l) != prev_gw)
	{
		vTaskDelay(1);
		prev_gw = (hr_bn_ipgw_h << 16) | hr_bn_ipgw_l;
		mb.gw_ip_addr.ip1 = hr_bn_ipgw_h >> 8;
		mb.gw_ip_addr.ip2 = hr_bn_ipgw_h & 0xFF;
		mb.gw_ip_addr.ip3 = hr_bn_ipgw_l >> 8;;
		mb.gw_ip_addr.ip4 = hr_bn_ipgw_l & 0xFF;
		check = 1;
	}

	if (((hr_bn_ipnm_h << 16) | hr_bn_ipnm_l) != prev_nm)
	{
		vTaskDelay(1);
		prev_nm = (hr_bn_ipnm_h << 16) | hr_bn_ipnm_l;
		mb.netmask_addr.ip1 = hr_bn_ipnm_h >> 8;
		mb.netmask_addr.ip2 = hr_bn_ipnm_h & 0xFF;
		mb.netmask_addr.ip3 = hr_bn_ipnm_l >> 8;
		mb.netmask_addr.ip4 = hr_bn_ipnm_l & 0xFF;
		check = 1;
	}

	if (((hr_bn_ipdns_h << 16) | hr_bn_ipdns_l) != prev_dns)
	{
		vTaskDelay(1);
		prev_dns = (hr_bn_ipdns_h << 16) | hr_bn_ipdns_l;
		mb.dns_ip_addr.ip1 = hr_bn_ipdns_h >> 8;
		mb.dns_ip_addr.ip2 = hr_bn_ipdns_h & 0xFF;
		mb.dns_ip_addr.ip3 = hr_bn_ipdns_l >> 8;
		mb.dns_ip_addr.ip4 = hr_bn_ipdns_l & 0xFF;
		check = 1;
	}

	if (check == 1)
		modbus_reinit_w5500(&mb);
}

static void reg_check_rtu_addr()
{
	static uint8_t prev_addr;

	if(prev_addr != (uint8_t) hr_bn_modbus_addr)
	{
		prev_addr = hr_bn_modbus_addr;
		mb.id = hr_bn_modbus_addr;
		reinit_uart1(&mb);
	}
}

void task_hr_handler(void *pvParameter)
{
	for(;;)
	{
		reg_check_ip();
		reg_check_rtu_addr();
		reg_check_time();

		vTaskDelay(1000);
	}
}
