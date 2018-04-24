/*
 * display_task.h
 *
 *  Created on: 29 мар. 2018 г.
 *      Author: Кочкин
 */

#ifndef TASK_OLED_H_
#define TASK_OLED_H_
#include "stm32f1xx.h"
/**
 * @def disp_state structure
 * @{
 */
enum state {
	D_TEST,//!< Test mode
	D_TMC, //!< Temperature of oil
	D_TOB, //!< Winding temperature
	D_TGD, //!< Temperature of fluid
	D_RPG, //!< Pressure of oil
	D_GX,  //!< Acceleration X
	D_GY,  //!< Acceleration Y
	D_GZ,  //!< Acceleration Z
	D_RIZ, //!< Isolation resistance
	D_SUCC, //!< Succ packs cnt
	D_DATE, //!< Date indication
	D_CONN_PARAMS,//!<Connection parameters, modbus address and ip address
	D_ERRCNT,	//!< Err packs cnt
	D_ERR  //!< Error indicator
};

enum explicit_usb_state{
	D_NONE,
	D_USB_ATTACHED,
	D_USB_WRITING,
	D_USB_FINISHED,
	D_USB_ERROR
};

extern volatile uint8_t display_state;
extern volatile uint8_t explicit_oled_state;


void task_oled(void *pvParameters);
#endif /* TASK_OLED_H_ */
