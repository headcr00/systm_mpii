/**
 * @file shv_modbus_misc.c
 *
 * @date 14 февр. 2018 г.
 * @author Shvabenland
 * @brief
 */


#include "shv_modbus.h"
#include "adc_bp_struct.h"
#include "math.h"

void shv_modbus_fill_parameters_from_spi()
{


	//packets
	hr_pack_number = (uint16_t)bp_struct.succ_packs_cnt;
	hr_resistance = (uint16_t)(bp_struct.resistance / 1000);
	hr_voltage = (uint16_t)bp_struct.line_voltage;
	hr_temp_plast = (uint16_t)bp_struct.fluid_t;
	hr_temp_oil = (uint16_t)bp_struct.oil_t;
	hr_temp_wiring = (uint16_t)bp_struct.winding_t;
	hr_pressure = (uint16_t)(bp_struct.pressure_lsb | (uint16_t)(bp_struct.pressure_msb << 8));
	hr_accx =  (uint16_t)(bp_struct.accel_ampl_lsb | (uint16_t)(bp_struct.accel_ampl_msb << 8)) >> 10;
	hr_accy = (uint16_t)((bp_struct.accel_ampl_lsb | (uint16_t)(bp_struct.accel_ampl_msb << 8)) & 0x3E0) >> 5;
	hr_accz = (uint16_t)((bp_struct.accel_ampl_lsb | (uint16_t)(bp_struct.accel_ampl_msb << 8)) & 0x1F);
	hr_accxy = (uint16_t) sqrt(((bp_struct.accel_ampl_lsb | (uint16_t)(bp_struct.accel_ampl_msb << 8)) >> 10)^2 + (((bp_struct.accel_ampl_lsb | (uint16_t)(bp_struct.accel_ampl_msb << 8)) & 0x3E0) >> 5) ^ 2);
//	&hr_madc_selfcheckout,
//	&hr_fail,
//	&hr_emergency,
	hr_bn_err_packs = bp_struct.error_packs_cnt;
	hr_isol_meter_state = bp_struct.line_status;
	hr_bp_selfcheckout = (uint16_t) (bp_struct.errors_msb << 8) | bp_struct.errors_lsb;
}
