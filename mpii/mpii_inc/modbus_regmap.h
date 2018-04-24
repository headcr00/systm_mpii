
#ifndef MODBUS_REGMAP_H_
#define MODBUS_REGMAP_H_
#include "stm32f1xx.h"



#define POOL_ADDR_SPT		(uint16_t) 0x03e8
#define POOL_ADDR_SERVICE	(uint16_t) 0x05dc
#define POOL_ADDR_ERROR		(uint16_t) 0x0836
#define POOL_ADDR_BORTSUHA	(uint16_t) 0x0240
#define POOL_ADDR_ELEKTON	(uint16_t) 0x002d
#define POOL_ADDR_IRZ		(uint16_t) 0x0600

#define POOL_ADDR_TECHNOMODE	(uint16_t) 0x00c8

#define POOL_CNT	7


//bp data
extern uint16_t hr_pack_number;
extern uint16_t hr_resistance;
extern uint16_t hr_voltage;
extern uint16_t hr_temp_plast;
extern uint16_t hr_temp_oil;
extern uint16_t hr_temp_wiring;
extern uint16_t hr_pressure;
extern uint16_t hr_accx;
extern uint16_t hr_accy;
extern uint16_t hr_accxy;
extern uint16_t hr_accz;
extern uint16_t hr_bn_err_packs;
extern uint16_t hr_misi_ushunt;
extern uint16_t hr_misi_udiv;
//service data
extern uint16_t hr_madc_firmware;
extern uint16_t hr_madc_serial;
extern uint16_t hr_madc_timeon;
extern uint16_t hr_bp_firmware;
extern uint16_t hr_bp_serial;
extern uint16_t hr_bp_timeon;
extern uint16_t hr_bn_firmware;
extern uint16_t hr_bn_serial;
extern uint16_t hr_bn_timeon;
//error codes
extern uint16_t hr_madc_selfcheckout;
extern uint16_t hr_fail;
extern uint16_t hr_emergency;
extern uint16_t hr_isol_meter_state;
extern uint16_t hr_bp_connection_status;
extern uint16_t hr_bp_selfcheckout;
extern uint16_t hr_bn_selfcheckout;

extern uint16_t hr_bn_technomode;

//RTC configure
extern uint16_t hr_bn_time_l;
extern uint16_t hr_bn_time_h;
//IP address config
extern uint16_t hr_bn_ip_l;
extern uint16_t hr_bn_ip_h;
extern uint16_t hr_bn_ipgw_l;
extern uint16_t hr_bn_ipgw_h;
extern uint16_t hr_bn_ipnm_l;
extern uint16_t hr_bn_ipnm_h;
extern uint16_t hr_bn_ipdns_l;
extern uint16_t hr_bn_ipdns_h;
//Modbus address config
extern uint16_t hr_bn_modbus_addr;








typedef struct
{
	uint16_t addr;
	uint16_t ** pool;
	uint16_t pool_size;
	uint8_t readonly;
}ModbusRegPool;





extern ModbusRegPool * pool_of_pools[POOL_CNT];
#endif /* MODBUS_REGMAP_H_ */

