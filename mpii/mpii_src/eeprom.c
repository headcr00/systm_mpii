
#include "eeprom.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "modbus_regmap.h"

enum{
	TCP_IP_CONNSTATE,
	LED_ERR,
	MADC_CONN_ERR,
	BP_CONN_ERR,
	ISOL_ERR,
	EEPROM1_ERR,
	EEPROM2_ERR,
	EEPROM_FULL

};
/**
 * Selects eeprom. Low is selected
 * @param fl EEPROM1, EEPROM2 or EEPROM_NONE
 */
static void s_eeprom_select(uint8_t fl)
{
	switch (fl){
	case EEPROM1:
		  HAL_GPIO_WritePin(CS_ROM0_GPIO_Port, CS_ROM0_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(CS_ROM1_GPIO_Port, CS_ROM1_Pin, GPIO_PIN_SET);
		break;
	case EEPROM2:
		  HAL_GPIO_WritePin(CS_ROM0_GPIO_Port, CS_ROM0_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(CS_ROM1_GPIO_Port, CS_ROM1_Pin, GPIO_PIN_RESET);
		break;
	case EEPROM_NONE:
		  HAL_GPIO_WritePin(CS_ROM0_GPIO_Port, CS_ROM0_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(CS_ROM1_GPIO_Port, CS_ROM1_Pin, GPIO_PIN_SET);
		break;
	}
}

static uint8_t s_get_selected_eeprom()
{
	uint8_t sel;
	sel = (HAL_GPIO_ReadPin(CS_ROM0_GPIO_Port, CS_ROM0_Pin) | HAL_GPIO_ReadPin(CS_ROM1_GPIO_Port, CS_ROM1_Pin) << 1);
	switch (sel)
	{
	case 0b11:
		return EEPROM_NONE;
	case 0b10:
		return EEPROM2;
	case 0b01:
		return EEPROM1;
	default:
		return EEPROM_NONE;
	}
}

static void s_eeprom_timeout_handler()
{
	switch(s_get_selected_eeprom())
	{
	case EEPROM1:
		hr_bn_selfcheckout |= 1 << EEPROM1_ERR;
		return;
	case EEPROM2:
		hr_bn_selfcheckout |= 1 << EEPROM2_ERR;
		return;
	default:
		hr_bn_selfcheckout |= 1 << EEPROM1_ERR;
		hr_bn_selfcheckout |= 1 << EEPROM2_ERR;
		return;
	}
}

static void s_eeprom_clear_timeout_error()
{
	switch(s_get_selected_eeprom())
	{
	case EEPROM1:
		hr_bn_selfcheckout &= ~(1 << EEPROM1_ERR);
		return;
	case EEPROM2:
		hr_bn_selfcheckout &= ~(1 << EEPROM2_ERR);
		return;
	default:
		hr_bn_selfcheckout &= ~(1 << EEPROM1_ERR);
		hr_bn_selfcheckout &= ~(1 << EEPROM2_ERR);
		return;
	}
}
/**
 * Sends byte to SPI
 * @param byte byte to send
 * @return returns data in buffer DR
 */
static uint8_t s_eeprom_send_byte(uint8_t byte)
{
	uint8_t retbyte;
	HAL_StatusTypeDef state = HAL_OK;
	s_eeprom_clear_timeout_error();
	state = HAL_SPI_TransmitReceive(&hspi3, &byte, &retbyte, 1, 0x1000);
	if (state != HAL_OK)
		s_eeprom_timeout_handler();

	/*!< Return the byte read from the SPI bus */
	return retbyte;
}
/**
 * Reads data. sends 0xFF and returns data from buffer
 * @return
 */
static uint8_t s_eeprom_read_byte()
{
	return s_eeprom_send_byte(0xFF);
}

/**
 * Writes buffer to eepro,
 * @param fl EEPROM1, EEPROM2 or EEPROM_NONE
 * @param addr address to write
 * @param buffer buffer to write
 * @param len length of buffer
 */
void eeprom_write_page(uint8_t fl, uint32_t addr, uint8_t * buffer, uint16_t len)
{

	s_eeprom_select(fl);
	s_eeprom_send_byte(EEPROM_WREN); //Write enable
	s_eeprom_select(EEPROM_NONE);

	s_eeprom_select(fl);

	addr = (EEPROM_WRITE << 24) | (addr & 0x1FFFF);
	s_eeprom_send_byte((uint8_t)((addr >> 24 ) & 0xFF));
	s_eeprom_send_byte((uint8_t)((addr >> 16 ) & 0xFF));
	s_eeprom_send_byte((uint8_t)((addr >> 8 ) & 0xFF));
	s_eeprom_send_byte((uint8_t)(addr & 0xFF));

	HAL_SPI_Transmit(&hspi3, buffer, len, 0x1000);
	s_eeprom_select(EEPROM_NONE);
	HAL_Delay(5);
}

/**
 * Reads buffer from eeprom
 * @param fl EEPROM1, EEPROM2 or EEPROM_NONE
 * @param addr address to read
 * @param buffer buffer where to read to
 * @param len length of buffer
 */
void eeprom_read_page(uint8_t fl, uint32_t addr, uint8_t * buffer, uint16_t len)
{
	s_eeprom_select(fl);
	addr = (EEPROM_READ << 24) | (addr & 0x1FFFF);

	s_eeprom_send_byte((uint8_t)((addr >> 24 ) & 0xFF));
	s_eeprom_send_byte((uint8_t)((addr >> 16 ) & 0xFF));
	s_eeprom_send_byte((uint8_t)((addr >> 8 ) & 0xFF));
	s_eeprom_send_byte((uint8_t)(addr & 0xFF));

	for (uint16_t i = 0; i < len; i++)
	{
		*(buffer + i) = s_eeprom_read_byte();
	}
	s_eeprom_select(EEPROM_NONE);
	HAL_Delay(1);
}

/**
 * Reads status register of EEPROM
 * @param fl EEPROM1, EEPROM2 or EEPROM_NONE
 * @return data from RDSR register
 */
uint8_t eeprom_read_status_register(uint8_t fl)
{
	uint8_t retval = 0;
	s_eeprom_select(fl);
	s_eeprom_send_byte(EEPROM_RDSR);
	retval = s_eeprom_read_byte();
	s_eeprom_select(EEPROM_NONE);
	return retval;
}

/**
 * Writes status register to eeprom
 * @param fl EEPROM1, EEPROM2 or EEPROM_NONE
 * @return
 */
uint8_t eeprom_write_status_register(uint8_t fl)
{
	uint8_t retval = 0;
	s_eeprom_select(fl);
	s_eeprom_send_byte(EEPROM_WRSR);
	retval = s_eeprom_read_byte();
	s_eeprom_select(EEPROM_NONE);
	return retval;
}


