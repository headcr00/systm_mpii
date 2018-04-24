/*
 * archive.h
 *
 *  Created on: 10 апр. 2018 г.
 *      Author: Кочкин
 */

#ifndef ARCHIVE_H_
#define ARCHIVE_H_
#include "stm32f1xx.h"
#include "adc_bp_struct.h"

#define MAX_ADDRESS_VALUE 0x20000
#define SETTINGS_ADDRESS 0x04
#define SETTINGS_SIZE_MODBUS_REGS 7
#define PACKS_START_ADDRESS SETTINGS_ADDRESS + SETTINGS_SIZE_MODBUS_REGS * sizeof(uint16_t)
#define MAX_PACK_ADDR (uint32_t) (MAX_ADDRESS_VALUE / sizeof(ArchiveStruct)) + 4
#define PACK_CNT (uint32_t) (MAX_ADDRESS_VALUE / sizeof(ArchiveStruct))
typedef enum
{
	A_OK,
	A_ERR
}ArchiveStatusTypedef;

typedef struct
{
	uint32_t rtc;
	uint8_t winding_t;
	uint8_t  oil_t;
	uint8_t  cpu_t;
	uint8_t  pressure_lsb;
	uint8_t  pressure_msb;
	uint8_t  fluid_t;
	uint8_t  u_rms;
	uint8_t  accel_ampl_lsb;
	uint8_t  accel_ampl_msb;
	uint8_t  accel_rms_lsb;
	uint8_t  accel_rms_msb;
	uint8_t  errors_lsb;
	uint8_t  errors_msb;
	uint16_t line_voltage;
	float    resistance;
}ArchiveStruct;

ArchiveStatusTypedef archive_get_pack_offset(uint32_t * offset);
ArchiveStatusTypedef archive_write_data(BpDefStruct * bp_struct_t, uint32_t rtc);
ArchiveStatusTypedef archive_get_data(ArchiveStruct * arch_struct_t, uint32_t packno);
ArchiveStatusTypedef archive_write_settings();
ArchiveStatusTypedef archive_read_settings();
ArchiveStatusTypedef archive_clear_data();


#endif /* ARCHIVE_H_ */
