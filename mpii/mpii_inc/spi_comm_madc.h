/*
 * spi_comm_madc.h
 *
 *  Created on: 4 ���. 2018 �.
 *      Author: ������
 */

#ifndef SPI_COMM_MADC_H_
#define SPI_COMM_MADC_H_

#include "stm32f1xx_hal.h"

extern SPI_HandleTypeDef hspi2;

void init_spi_comm_task();
#endif /* SPI_COMM_MADC_H_ */
