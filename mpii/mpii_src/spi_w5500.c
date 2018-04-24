/*
 * spi_w5500.c
 *
 *  Created on: 30 мар. 2018 г.
 *      Author: Кочкин
 */

#include "spi_w5500.h"
#include "stm32f1xx_hal.h"
/**
 * PHY operation mode select pins
 */
enum W5500Mode{
	HD10AD,       //!< 10BT Half-duplex, Auto-negotiation disabled
	FD10AD,       //!< 10BT Full-duplex, Auto-negotiation disabled
	HD100AD,      //!< 100BT Half-duplex, Auto-negotiation disabled
	FD100AD,      //!< 100BT Full-duplex, Auto-negotiation disabled
	HD100AE,      //!< 100BT Half-duplex, Auto-negotiation enabled
	FD100AE = 0x07//!< All capable, Auto-negotiation enabled
};
/**
 * Setup SPI and W5500 mode
 * @param mode #W5500Mode PHY operation mode
 */
static void s_w5500_init(uint8_t mode)
{
	HAL_GPIO_WritePin(PMODE_0_GPIO_Port, PMODE_0_Pin, mode & 0x01);
	HAL_GPIO_WritePin(PMODE_1_GPIO_Port, PMODE_1_Pin, mode >> 1);
	HAL_GPIO_WritePin(PMODE_2_GPIO_Port, PMODE_2_Pin, mode >> 2);

	HAL_GPIO_WritePin(W5500_NSS_GPIO_Port, W5500_NSS_Pin, GPIO_PIN_SET);

}
/**
 * Chip deselect W5500
 */
static void s_w5500_deselect_chip()
{
	HAL_GPIO_WritePin(W5500_NSS_GPIO_Port, W5500_NSS_Pin, GPIO_PIN_SET);
}

static void s_w5500_select_chip()
{
	HAL_GPIO_WritePin(W5500_NSS_GPIO_Port, W5500_NSS_Pin, GPIO_PIN_RESET);
}

static uint8_t s_w5500_send_byte(uint8_t byte)
{
	uint8_t retbyte;
	HAL_SPI_TransmitReceive(&hspi1, &byte, &retbyte, 1, 0x1000);
	return retbyte;
}

static uint8_t s_w5500_read_byte()
{
	return s_w5500_send_byte(0xFF);
}

void w5500_init(wiz_NetInfo net_info)
{
	uint8_t rcvBuf[20], bufSize[] = {2, 2, 2, 2};

	s_w5500_init(HD10AD);
	reg_wizchip_cs_cbfunc(s_w5500_select_chip, s_w5500_deselect_chip);
	reg_wizchip_spi_cbfunc(s_w5500_read_byte, s_w5500_send_byte);
	//	reg_wizchip_spiburst_cbfunc(s_w5500_burst_read, s_w5500_burst_write);

	wizchip_init(bufSize, bufSize);
	wizchip_setnetinfo(&net_info);
	wizchip_getnetinfo(&net_info);


}
