/*
 * write_to_usb.c
 *
 *  Created on: 5 апр. 2018 г.
 *      Author: Кочкин
 */


#include "write_to_usb.h"
#include "fatfs.h"
#include "ff.h"
#include "usb_host.h"
#include "adc_bp_struct.h"
#include "archive.h"
#include "task_oled.h"
#include "time.h"
static uint8_t write_to_usb()
{
	ArchiveStruct arch_struct;
	FRESULT res;
	FIL arch_file;
	time_t time;
	struct tm * timeinfo;

	if (Appli_state == APPLICATION_READY)
	{
		res = f_open(&arch_file, "archive.txt", FA_CREATE_ALWAYS | FA_WRITE);
		if (res != FR_OK)
		{
			__asm("nop");
		}

		res = f_printf(&arch_file, "Time;WindingT;OilT;CpuT;Pressure;FluidT;URms;AccAmpl;AccRms;Errors;LineV;Res\n");

		if (res != FR_OK)
		{
			__asm("nop");
		}

		for (uint32_t pack_cnt = 0; pack_cnt < PACK_CNT; pack_cnt++)
		{
			archive_get_data(&arch_struct, 0);
			time = arch_struct.rtc;
			timeinfo = localtime(&time);

			res = f_printf(&arch_file,"%u.%u.%u.%u %u:%u:%u;%u;%u;%u;%u;%u;%u;%u;%u;%u;%u;%u\n",\
					timeinfo->tm_mday, timeinfo->tm_mon++, 1900 + timeinfo->tm_year, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, arch_struct.winding_t, arch_struct.oil_t, \
					arch_struct.cpu_t, ((uint16_t)arch_struct.pressure_msb << 8 )| arch_struct.pressure_lsb, \
					arch_struct.fluid_t, arch_struct.u_rms, ((uint16_t)arch_struct.accel_ampl_msb << 8 )| arch_struct.accel_ampl_lsb,\
					((uint16_t)arch_struct.accel_rms_msb << 8 )| arch_struct.accel_rms_lsb, \
					((uint16_t)arch_struct.errors_msb << 8 )| arch_struct.errors_lsb, arch_struct.line_voltage, (uint32_t) arch_struct.resistance);
			if (res != FR_OK)
			{
				__asm("nop");
			}
		}


		f_close(&arch_file);
	}
	return res;
}


void usb_write_archive_task(void *pvParameters)
{
	explicit_oled_state = D_USB_WRITING;
	write_to_usb();
	explicit_oled_state = D_USB_FINISHED;
	vTaskDelete(NULL);
}
