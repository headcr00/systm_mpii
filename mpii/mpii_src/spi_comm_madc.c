/*
 * spi_comm_madc.c
 *
 *  Created on: 4 апр. 2018 г.
 *      Author: Кочкин
 */

#include "spi_comm_madc.h"
#include "stm32f1xx_hal_conf.h"
#include "adc_bp_struct.h"
void init_spi_comm_task()
{

}

void nss_interrupt_handler()
{
	HAL_SPI_DMAStop(&hspi2);
	//HAL_SPI_Receive_DMA(&hspi2, (uintptr) bp_struct, sizeof(bp_struct));
}
