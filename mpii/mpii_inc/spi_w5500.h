/*
 * spi_w5500.h
 *
 *  Created on: 30 мар. 2018 г.
 *      Author: Кочкин
 */

#ifndef SPI_W5500_H_
#define SPI_W5500_H_

#include "wizchip_conf.h"
#include "stm32f1xx_hal.h"
extern SPI_HandleTypeDef hspi1;

void w5500_init(wiz_NetInfo net_info);


#endif /* SPI_W5500_H_ */
