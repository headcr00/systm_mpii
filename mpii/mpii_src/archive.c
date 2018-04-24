/*
 * archive.c
 *
 *  Created on: 6 апр. 2018 г.
 *      Author: Кочкин
 *
 *      TODO: make EEPROM write circular
 */
#include "eeprom.h"
#include "stm32f1xx_hal.h"
#include "archive.h"
#include "modbus_regmap.h"


ArchiveStatusTypedef archive_get_pack_offset(uint32_t * offset)
{
	eeprom_read_page(EEPROM1, 0x00, (uint8_t *)offset, sizeof(*offset));
	if (*offset == 0xFFFFFFFF)
		return A_ERR;
	return A_OK;
}

ArchiveStatusTypedef archive_write_data(BpDefStruct * bp_struct_t, uint32_t rtc)
{
	uint32_t offset = 0;
	ArchiveStruct arch_struct_t;

	arch_struct_t.rtc = rtc;
	arch_struct_t.winding_t =  bp_struct_t->winding_t;
	arch_struct_t.oil_t =  bp_struct_t->oil_t;
	arch_struct_t.cpu_t = bp_struct_t->cpu_t;
	arch_struct_t.pressure_lsb = bp_struct_t->pressure_lsb;
	arch_struct_t.pressure_msb = bp_struct_t->pressure_msb;
	arch_struct_t.fluid_t = bp_struct_t->fluid_t;
	arch_struct_t.u_rms = bp_struct_t->u_rms;
	arch_struct_t.accel_ampl_lsb = bp_struct_t->accel_ampl_lsb;
	arch_struct_t.accel_ampl_msb = bp_struct_t->accel_ampl_msb;
	arch_struct_t.accel_rms_lsb = bp_struct_t->accel_rms_lsb;
	arch_struct_t.accel_rms_msb = bp_struct_t->accel_rms_msb;
	arch_struct_t.errors_lsb = bp_struct_t->errors_lsb;
	arch_struct_t.errors_msb = bp_struct_t->errors_msb;
	arch_struct_t.line_voltage = (uint16_t) bp_struct_t->line_voltage;
	arch_struct_t.resistance = bp_struct_t->resistance;

	if (archive_get_pack_offset(&offset) != A_OK)
	{
		offset = PACKS_START_ADDRESS;
		eeprom_write_page(EEPROM1, 0x00, (uint8_t *) &offset, sizeof(offset));
	}
//TODO: move this section after arch_struct writing
	if (offset + sizeof(arch_struct_t) < MAX_ADDRESS_VALUE)
	{
		offset += sizeof(arch_struct_t);
	}
	else
	{
		offset = PACKS_START_ADDRESS;
	}

	eeprom_write_page(EEPROM1, 0x00, (uint8_t *) &offset, sizeof(offset));
	eeprom_write_page(EEPROM1, offset, (uint8_t *) &arch_struct_t, sizeof(arch_struct_t));
	return A_OK;
}

ArchiveStatusTypedef archive_get_data(ArchiveStruct * arch_struct_t, uint32_t packno)
{

	uint32_t offset = 0;
	uint32_t addr = 0;
	uint32_t last_pack_no;
	archive_get_pack_offset(&offset);
	addr = offset + (packno * sizeof(ArchiveStruct));
	if (addr > MAX_ADDRESS_VALUE)
	{
		addr = PACKS_START_ADDRESS + (addr - MAX_ADDRESS_VALUE)/sizeof(ArchiveStruct);
	}

	eeprom_read_page(EEPROM1, addr, (uint8_t *) arch_struct_t, sizeof(ArchiveStruct));
	return A_OK;
}

ArchiveStatusTypedef archive_write_settings()
{
	uint16_t settings[SETTINGS_SIZE_MODBUS_REGS];

	settings[0] = hr_bn_ip_l;
	settings[1] = hr_bn_ip_h;
	settings[2] = hr_bn_ipgw_l;
	settings[3] = hr_bn_ipgw_h;
	settings[4] = hr_bn_ipdns_l;
	settings[5] = hr_bn_ipdns_h;
	settings[6] = hr_bn_modbus_addr;

	eeprom_write_page(EEPROM1, SETTINGS_ADDRESS, (uint8_t *) settings, sizeof(settings));
	return A_OK;

}

ArchiveStatusTypedef archive_read_settings()
{
	uint16_t settings[SETTINGS_SIZE_MODBUS_REGS];

	eeprom_read_page(EEPROM1, SETTINGS_ADDRESS, (uint8_t *) settings, sizeof(settings));

	hr_bn_ip_l        = settings[0];
	hr_bn_ip_h        = settings[1];
	hr_bn_ipgw_l      = settings[2];
	hr_bn_ipgw_h      = settings[3];
	hr_bn_ipdns_l     = settings[4];
	hr_bn_ipdns_h     = settings[5];
	hr_bn_modbus_addr = settings[6];
	return A_OK;
}
