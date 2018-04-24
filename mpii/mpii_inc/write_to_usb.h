/*
 * write_to_usb.h
 *
 *  Created on: 5 апр. 2018 г.
 *      Author: Кочкин
 */

#ifndef WRITE_TO_USB_H_
#define WRITE_TO_USB_H_

#include "stm32f1xx.h"

void usb_write_archive_task(void *pvParameters);
uint32_t timetest();
#endif /* WRITE_TO_USB_H_ */
