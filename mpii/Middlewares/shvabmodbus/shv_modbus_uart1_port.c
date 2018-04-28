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

uint8_t rx_buffer_uart1[205] = {0};
uint8_t rx_buffer_uart2[205] = {0};
uint8_t rx_buffer_uart3[205] = {0};
modbus_t * mbs;

char * output_buffer;


void init_uart(modbus_t * mb_pointer)
{
	mbs = mb_pointer;
	HAL_TIM_Base_Start_IT(&htim3);

}

void reinit_uart(modbus_t * mb_pointer)
{
	mbs = mb_pointer;
}


void reload_dma(UART_HandleTypeDef * huart)
{


	HAL_UART_DMAStop(huart);

	switch((uint32_t)huart->Instance)
	{
	case (uint32_t)USART1:
		HAL_UART_Receive_DMA(huart, rx_buffer_uart1, sizeof(rx_buffer_uart1));
		break;
	case (uint32_t)USART2:
		HAL_UART_Receive_DMA(huart, rx_buffer_uart2, sizeof(rx_buffer_uart2));
		break;
	case (uint32_t)USART3:
		HAL_UART_Receive_DMA(huart, rx_buffer_uart3, sizeof(rx_buffer_uart3));
		break;
	}


}



void TIM3_Update_Handler()
{
	static uint16_t counter1=0;
	static uint16_t counter2=0;
	static uint16_t counter3=0;
	uint16_t len;
	int dma_cnt = 0;

	dma_cnt = huart1.hdmarx->Instance->CNDTR;//DMA_GetCurrDataCounter(DMA1_Channel5);
	if ((dma_cnt - counter1 == 0) && dma_cnt != sizeof(rx_buffer_uart1)) //we received data
	{
		reload_dma(&huart1);
		mb_error_code err = modbus_rt_process_request(mbs, (char *)rx_buffer_uart1, &output_buffer, &len);
		if (err == MB_ENOERR || err == MB_ENOREG)
		{

			HAL_UART_Transmit(&huart1, (uint8_t *)output_buffer, len, 0x1000);

		}
		if (output_buffer != 0)
			free(output_buffer);

	}
	counter1 =  huart1.hdmarx->Instance->CNDTR;

	dma_cnt = huart2.hdmarx->Instance->CNDTR;//DMA_GetCurrDataCounter(DMA1_Channel5);
	if ((dma_cnt - counter2 == 0) && dma_cnt != sizeof(rx_buffer_uart2)) //we received data
	{
		reload_dma(&huart2);
		mb_error_code err = modbus_rt_process_request(mbs, (char *)rx_buffer_uart2, &output_buffer, &len);
		if (err == MB_ENOERR || err == MB_ENOREG)
		{

			HAL_UART_Transmit(&huart2, (uint8_t *)output_buffer, len, 0x1000);

		}
		if (output_buffer != 0)
			free(output_buffer);

	}
	counter2 =  huart2.hdmarx->Instance->CNDTR;


	dma_cnt = huart3.hdmarx->Instance->CNDTR;//DMA_GetCurrDataCounter(DMA1_Channel5);
	if ((dma_cnt - counter3 == 0) && dma_cnt != sizeof(rx_buffer_uart3)) //we received data
	{
		reload_dma(&huart3);
		mb_error_code err = modbus_rt_process_request(mbs, (char *)rx_buffer_uart3, &output_buffer, &len);
		if (err == MB_ENOERR || err == MB_ENOREG)
		{

			HAL_UART_Transmit(&huart3, (uint8_t *)output_buffer, len, 0x1000);

		}
		if (output_buffer != 0)
			free(output_buffer);

	}
	counter3 =  huart3.hdmarx->Instance->CNDTR;

}
