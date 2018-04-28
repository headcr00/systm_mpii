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
	HAL_SPI_Receive_DMA(&hspi2, (uint8_t*) &bp_struct, sizeof(bp_struct));
}

void nss_interrupt_handler()
{
	HAL_SPI_DMAStop(&hspi2);
	HAL_SPI_Receive_DMA(&hspi2,  (uint8_t*) &bp_struct, sizeof(bp_struct));
}

void spi_comm_dma_irq_handler()
{
	static uint16_t pack_no = 0;
//if /dma transfer complete
	//HAL_SPI_Receive_DMA(&hspi2, (uintptr) bp_struct, sizeof(bp_struct));
	if (bp_struct.succ_packs_cnt != pack_no)
	{
		pack_no = bp_struct.succ_packs_cnt;
		//write archive;
	}
	shv_modbus_fill_parameters_from_spi();

}
