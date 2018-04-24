/**
 * @file shv_modbus.c
 *
 * @date 1 февр. 2018 г.
 * @author Shvabenland
 * @brief
 */

#include <shv_modbus.h>
#include "stm32f1xx_hal.h"
#include "stdlib.h"

uint8_t rx_buffer[205] = {0};
modbus_t * mbs;

char * output_buffer;


void init_uart1(modbus_t * mb_pointer)
{
	mbs = mb_pointer;
	HAL_TIM_Base_Start_IT(&htim3);
}

void reinit_uart1(modbus_t * mb_pointer)
{
	mbs = mb_pointer;
}


void reload_dma()
{
	HAL_UART_DMAStop(&huart1);
	HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));

}



void TIM3_Update_Handler()
{
	static uint16_t counter;
	uint16_t len;


	int dma_cnt = huart1.hdmarx->Instance->CNDTR;//DMA_GetCurrDataCounter(DMA1_Channel5);
	if ((dma_cnt - counter == 0) && dma_cnt != sizeof(rx_buffer)) //we received data
	{
		reload_dma();
		mb_error_code err = modbus_rt_process_request(mbs, rx_buffer, &output_buffer, &len);
		if (err == MB_ENOERR || err == MB_ENOREG)
		{

			HAL_UART_Transmit(&huart1, output_buffer, len, 0x1000);

		}
		if (output_buffer != 0)
			free(output_buffer);

	}
	counter =  huart1.hdmarx->Instance->CNDTR;
}
