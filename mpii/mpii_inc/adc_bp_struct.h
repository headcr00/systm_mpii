/**
 * @file adc_bp_struct.h
 *
 * @date 22 ���. 2017 �.
 * @author Shvabenland
 * @brief
 */

#ifndef ADC_BP_STRUCT_H_
#define ADC_BP_STRUCT_H_
#include "stm32f1xx.h"
typedef struct{

	//BP receive data
	uint8_t winding_t;
	uint8_t oil_t;
	uint8_t cpu_t;
	uint8_t pressure_lsb;
	uint8_t pressure_msb;
	uint8_t fluid_t;
	uint8_t u_rms;
	uint8_t accel_ampl_lsb;
	uint8_t accel_ampl_msb;
	uint8_t accel_rms_lsb;
	uint8_t accel_rms_msb;
	uint8_t errors_lsb;
	uint8_t errors_msb;
	uint8_t cs;

	uint32_t succ_packs_cnt;
	uint32_t error_packs_cnt;
	//MFU params
	uint32_t conn_status;
	float mean_mfu_voltage;
	//MISI and MPL params
	float line_voltage;
	float line_current;
	float resistance;
	uint8_t misi_state;
	uint8_t line_status; //Turned on or off
}BpDefStruct;

extern BpDefStruct bp_struct;

#endif /* ADC_BP_STRUCT_H_ */
