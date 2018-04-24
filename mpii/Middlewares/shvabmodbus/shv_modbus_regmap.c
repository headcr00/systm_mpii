/**
 * @file shv_modbus_regmap.c
 *
 * @date 6 февр. 2018 г.
 * @author Shvabenland
 * @brief
 */

#include "modbus_regmap.h"
#include "shv_modbus.h"

//Init all holding registers.
uint16_t hr_pack_number = 1;
uint16_t hr_resistance = 2;
uint16_t hr_voltage = 3;
uint16_t hr_temp_plast = 4;
uint16_t hr_temp_oil = 5;
uint16_t hr_temp_wiring = 6;
uint16_t hr_pressure = 7;
uint16_t hr_accx = 8;
uint16_t hr_accy = 9;
uint16_t hr_accxy = 10;
uint16_t hr_accz = 11;
uint16_t hr_bn_err_packs;

uint16_t hr_madc_firmware;
uint16_t hr_madc_serial;
uint16_t hr_madc_timeon;
uint16_t hr_bp_firmware;
uint16_t hr_bp_serial;
uint16_t hr_bp_timeon;
uint16_t hr_bn_firmware;
uint16_t hr_bn_serial;
uint16_t hr_bn_timeon;

uint16_t hr_madc_selfcheckout;
uint16_t hr_fail;
uint16_t hr_emergency;
uint16_t hr_isol_meter_state;
uint16_t hr_bp_connection_status;
uint16_t hr_bp_selfcheckout;
uint16_t hr_bn_selfcheckout;

uint16_t hr_bn_technomode = 0;

uint16_t hr_bn_time_l;
uint16_t hr_bn_time_h;
uint16_t hr_bn_ip_l;
uint16_t hr_bn_ip_h;
uint16_t hr_bn_ipgw_l;
uint16_t hr_bn_ipgw_h;
uint16_t hr_bn_ipnm_l;
uint16_t hr_bn_ipnm_h;
uint16_t hr_bn_ipdns_l;
uint16_t hr_bn_ipdns_h;
uint16_t hr_bn_modbus_addr;

ModbusRegPool bp_spt_pool;
ModbusRegPool bp_service_pool;
ModbusRegPool bp_error_pool;
ModbusRegPool bp_bortsuha_pool;
ModbusRegPool bp_elekton_pool;
ModbusRegPool bp_irz_pool;
ModbusRegPool bp_technomode_pool;

/**
 * Definition of SPT protocol measure
 */
uint16_t * spt_pool[] = {	//0x03e8 is start address
		&hr_pack_number,
		&hr_resistance,
		&hr_voltage,
		&hr_temp_plast,
		&hr_temp_oil,
		&hr_temp_wiring,
		&hr_pressure,
		&hr_accx,
		&hr_accy,
		&hr_accz,
		&hr_bn_err_packs
};
/**
 * Service data pool
 */
uint16_t * service_pool[] = {//start 0x05dc address
		&hr_madc_firmware,
		&hr_madc_serial,
		&hr_madc_timeon,
		&hr_bp_firmware,
		&hr_bp_serial,
		&hr_bp_timeon,
		&hr_bn_firmware,
		&hr_bn_serial,
		&hr_bn_timeon,
		&hr_bn_time_l,
		&hr_bn_time_h,
		&hr_bn_ip_l,
		&hr_bn_ip_h,
		&hr_bn_ipgw_l,
		&hr_bn_ipgw_h,
		&hr_bn_ipnm_l,
		&hr_bn_ipnm_h,
		&hr_bn_ipdns_l,
		&hr_bn_ipdns_h,
		&hr_bn_modbus_addr
};
/**
 * Error pool
 */
uint16_t * error_pool[] = {//address of 0x0836 is start
		&hr_madc_selfcheckout,
		&hr_fail,
		&hr_emergency,
		&hr_isol_meter_state,
		&hr_bp_connection_status,
		&hr_bp_selfcheckout,
		&hr_bn_selfcheckout
};

/**
 * Bortsuha protocol
 */
uint16_t * bortsuha_pool[] = {//is start of address 0x0240
		&hr_resistance,
		&hr_temp_plast,
		&hr_temp_oil,
		&hr_accxy,
		&hr_accz,
		&hr_pressure
};
/**
 * electon protocol
 */
uint16_t * elekton_pool[] = {//address 0x002d. achtung, register nummern sind nicht posledovateln
		&hr_resistance,
		&hr_pressure,
		&hr_temp_oil,
		&hr_temp_plast,//0x0031
		&hr_pack_number,//0x32
		&hr_pack_number,//0x33
		&hr_accxy,//0x0034
		&hr_accz,
		&hr_temp_wiring
};
/**
 * irz protocol
 */
uint16_t * irz_pool[] = {//irkutsky address 0x0600. saw pool before? yep, not serial again
		&hr_resistance,
		&hr_pack_number,
		&hr_pack_number,
		&hr_pack_number,
		&hr_pack_number,
		&hr_accx,
		&hr_accy,
		&hr_accz,
		&hr_temp_plast,
		&hr_temp_wiring,
		&hr_pressure,
		&hr_temp_oil
};
/**
 * Technological/bootloader pool
 */
uint16_t * technomode_pool[] = {//0xc8 addr
		&hr_bn_technomode
};
ModbusRegPool * pool_of_pools[POOL_CNT];

void modbus_init_registers()
{

	bp_spt_pool.addr = POOL_ADDR_SPT;
	bp_spt_pool.pool = spt_pool;
	bp_spt_pool.pool_size = sizeof(spt_pool) / 4;
	bp_spt_pool.readonly = 1;

	bp_service_pool.addr = POOL_ADDR_SERVICE;
	bp_service_pool.pool = service_pool;
	bp_service_pool.pool_size = sizeof(service_pool) / 4;
	bp_service_pool.readonly = 0;

	bp_error_pool.addr = POOL_ADDR_ERROR;
	bp_error_pool.pool = error_pool;
	bp_error_pool.pool_size = sizeof(error_pool) / 4;
	bp_error_pool.readonly = 1;

	bp_bortsuha_pool.addr = POOL_ADDR_BORTSUHA;
	bp_bortsuha_pool.pool = bortsuha_pool;
	bp_bortsuha_pool.pool_size = sizeof(bortsuha_pool) / 4;
	bp_bortsuha_pool.readonly = 1;

	bp_elekton_pool.addr = POOL_ADDR_ELEKTON;
	bp_elekton_pool.pool = elekton_pool;
	bp_elekton_pool.pool_size= sizeof(elekton_pool) / 4;
	bp_elekton_pool.readonly = 1;

	bp_irz_pool.addr = POOL_ADDR_IRZ;
	bp_irz_pool.pool = irz_pool;
	bp_irz_pool.pool_size = sizeof(irz_pool) / 4;
	bp_irz_pool.readonly = 1;

	bp_technomode_pool.addr = POOL_ADDR_TECHNOMODE;
	bp_technomode_pool.pool = technomode_pool;
	bp_technomode_pool.pool_size = sizeof(technomode_pool) / 4;
	bp_technomode_pool.readonly = 0;

	pool_of_pools[0] = &bp_spt_pool;
	pool_of_pools[1] = &bp_service_pool;
	pool_of_pools[2] = &bp_error_pool;
	pool_of_pools[3] = &bp_bortsuha_pool;
	pool_of_pools[4] = &bp_elekton_pool;
	pool_of_pools[5] = &bp_irz_pool;
	pool_of_pools[6] = &bp_technomode_pool;

	hr_bn_time_l = 0;
	hr_bn_time_h = 0;
}

