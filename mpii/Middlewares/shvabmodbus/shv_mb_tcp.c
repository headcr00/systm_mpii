/**
 * @file shv_mb_rtu.c
 *
 * @date 1 февр. 2018 г.
 * @author Shvabenland
 * @brief
 */
#include "stdlib.h"
#include "shv_modbus.h"


enum{
	NONE,
	READ_COILS,
	READ_DI,
	READ_HR,
	READ_IR,
	WRITE_SCOIL,
	WRITE_SREG,
	WRITE_MCOILS = 15,
	WRITE_MREGS = 16
};

static mb_error_code s_read_hr(ModbusRegPool * reg, uint16_t address, uint16_t reg_no, char * output_buffer)
{
	mb_error_code retval = MB_ENOERR;
	uint16_t reg_index =  address - reg->addr;
	if( ( address >= reg->addr ) && ( address + reg_no <= reg->addr + reg->pool_size) )
	{
		while (reg_no > 0)
		{
			*output_buffer = ( unsigned char )( **(reg->pool + reg_index) >> 8 );
			output_buffer++;
			*output_buffer = ( unsigned char )( **(reg->pool + reg_index) & 0xFF );
			output_buffer++;

			reg_index++;
			reg_no--;
		}
		return retval;
	}
	else
	{
		retval = MB_ENOREG;
		return retval;
	}
}

static mb_error_code s_write_hr(ModbusRegPool * reg, uint16_t address, uint16_t reg_no, char * input_buffer)
{
	mb_error_code retval = MB_ENOERR;
	uint16_t reg_index = address - reg->addr;

	reg_index = address - reg->addr;
	//we dont need no write at all
	if (1 == reg->readonly)
		return MB_ENOREG;
	if( ( address >= reg->addr ) && ( address + reg_no <= reg->addr + reg->pool_size) )
	{
		while( reg_no > 0 )
		{
			**(reg->pool+reg_index) = *input_buffer << 8;
			input_buffer++;
			**(reg->pool+reg_index) |= *input_buffer;
			input_buffer++;
			reg_index++;
			reg_no--;
		}
		return MB_ENOERR;
	}
	else
	{
		retval = MB_ENOREG;
		return retval;
	}
}

static void s_exception_handler(mb_error_code exception, char * output_buffer)
{
	output_buffer[7] |= 0x80;

	switch (exception){
	case MB_ENORES:
		output_buffer[8] = 0x07;
		break;
	case MB_ENOREG:
		output_buffer[8] = 0x02;
		break;
	}

}

static mb_error_code s_modbus_tcp_func_03_read_hr(char * input_buffer, char ** output_buffer, uint16_t * len)
{
	mb_error_code retval= MB_ENOREG;
	uint16_t reg_no;
	uint16_t address;

	address = input_buffer[8] << 8 | input_buffer[9];
	reg_no =  input_buffer[10] << 8 | input_buffer[11];
	*len = 9 + reg_no * 2;
	*output_buffer = pvPortMalloc(*len);

	if (*output_buffer == 0)
	{
		retval = MB_ENORES;
		*len = 9;
		*output_buffer = pvPortMalloc (*len);
		if (*output_buffer == 0)
			return MB_EPORTERR;
	}
	(*output_buffer)[0] = input_buffer[0]; //trans id
	(*output_buffer)[1] = input_buffer[1];

	(*output_buffer)[2] = input_buffer[2]; //protocol identifier
	(*output_buffer)[3] = input_buffer[3];

	(*output_buffer)[4] = (3 + reg_no * 2) >> 8; //message length
	(*output_buffer)[5] = (3 + reg_no * 2) & 0xFF;

	(*output_buffer)[6] = input_buffer[6]; //address of device
	(*output_buffer)[7] = input_buffer[7]; //func

	(*output_buffer)[8] = (reg_no * 2); // bytes to follow



	if (retval != MB_ENORES)
	{
		for (uint8_t i = 0; i < POOL_CNT; i++)
		{
			retval = s_read_hr(pool_of_pools[i], address, reg_no, (*output_buffer + 9));
			if (MB_ENOERR == retval)
				break;
		}
	}

	if (retval != MB_ENOERR)
	{
		s_exception_handler(retval, (*output_buffer));
		(*output_buffer)[4] = 3 >> 8; //message length
		(*output_buffer)[5] = (3) & 0xFF;
		*len = 9;
	}
	return retval;

}
static mb_error_code s_modbus_tcp_func_06_write_sreg(char * input_buffer, char ** output_buffer, uint16_t * len)
{
	mb_error_code retval= MB_ENOREG;
	uint16_t reg_no;
	uint16_t address;
	uint8_t offset;
	address = input_buffer[8] << 8 | input_buffer[9];
	reg_no =  input_buffer[10] << 8 | input_buffer[11];

	reg_no = 1;
	offset = 10;


	*len = 12;

	(*output_buffer) = malloc(*len);
	if ((*output_buffer) == 0)
	{
		retval = MB_ENORES;
		*len = 9;
		(*output_buffer) = malloc (*len);
		if ((*output_buffer) == 0)
			return MB_EPORTERR;
	}

	(*output_buffer)[0] = input_buffer[0]; //trans id
	(*output_buffer)[1] = input_buffer[1];

	(*output_buffer)[2] = input_buffer[2]; //protocol identifier
	(*output_buffer)[3] = input_buffer[3];

	(*output_buffer)[4] = (6) >> 8; //message length
	(*output_buffer)[5] = (6) & 0xFF;

	(*output_buffer)[6] = input_buffer[6]; //address of device
	(*output_buffer)[7] = input_buffer[7]; //func

	(*output_buffer)[8] = input_buffer[8]; //address of register
	(*output_buffer)[9] = input_buffer[9]; //func

	(*output_buffer)[10] = input_buffer[10]; //data to write
	(*output_buffer)[11] = input_buffer[11]; //func

	if (retval != MB_ENORES)
	{
		for (uint8_t i = 0; i < POOL_CNT; i++)
		{
			retval = s_write_hr(pool_of_pools[i], address, reg_no, input_buffer + offset);
			if (MB_ENOERR == retval)
				break;
		}
	}

	if (retval != MB_ENOERR)
	{
		s_exception_handler(retval, (*output_buffer));
		(*output_buffer)[4] = 3 >> 8; //message length
		(*output_buffer)[5] = (3) & 0xFF;
		*len = 9;
	}

	return retval;

}

static mb_error_code s_modbus_tcp_func_16_write_mregs(char * input_buffer, char ** output_buffer, uint16_t * len)
{
	mb_error_code retval= MB_ENOREG;
	uint16_t reg_no;
	uint16_t address;
	uint8_t offset = 7; //offset for data in single reg or multi reg
	address = input_buffer[8] << 8 | input_buffer[9];
	reg_no =  input_buffer[10] << 8 | input_buffer[11];

	offset = 13;

	*len = 12;

	(*output_buffer) = malloc(*len);
		if ((*output_buffer) == 0)
		{
			retval = MB_ENORES;
			*len = 9;
			(*output_buffer) = malloc (*len);
			if ((*output_buffer) == 0)
				return MB_EPORTERR;
		}

	(*output_buffer)[0] = input_buffer[0];
	(*output_buffer)[1] = input_buffer[1];

	if (retval != MB_ENORES)
	{
		for (uint8_t i = 0; i < POOL_CNT; i++)
		{
			retval = s_write_hr(pool_of_pools[i], address, reg_no, input_buffer + offset);
			if (MB_ENOERR == retval)
				break;
		}
	}

	(*output_buffer)[0] = input_buffer[0]; //trans id
	(*output_buffer)[1] = input_buffer[1];

	(*output_buffer)[2] = input_buffer[2]; //protocol identifier
	(*output_buffer)[3] = input_buffer[3];

	(*output_buffer)[4] = (6) >> 8; //message length
	(*output_buffer)[5] = (6) & 0xFF;

	(*output_buffer)[6] = input_buffer[6]; //address of device
	(*output_buffer)[7] = input_buffer[7]; //func

	(*output_buffer)[8] = input_buffer[8]; //address of register
	(*output_buffer)[9] = input_buffer[9]; //func

	(*output_buffer)[10] = input_buffer[10]; //data to write
	(*output_buffer)[11] = input_buffer[11]; //func


	if (retval != MB_ENOERR)
	{
		s_exception_handler(retval, (*output_buffer));
		(*output_buffer)[4] = 3 >> 8; //message length
		(*output_buffer)[5] = (3) & 0xFF;
		*len = 9;
	}

	return retval;
}


mb_error_code modbus_tcp_process_request(modbus_t * mb, char * input_buffer, char ** output_buffer, uint16_t * len)
{


	//apply function
	switch(*(input_buffer + 7))
	{
	case READ_COILS:
		break;
	case READ_DI:
		break;
	case READ_HR:
		return s_modbus_tcp_func_03_read_hr(input_buffer, output_buffer, len);
	case READ_IR:
		break;
	case WRITE_SCOIL:
		break;
	case WRITE_SREG:
		return s_modbus_tcp_func_06_write_sreg(input_buffer, output_buffer, len);
	case WRITE_MCOILS:
		break;
	case WRITE_MREGS:
		return s_modbus_tcp_func_16_write_mregs(input_buffer, output_buffer, len);
	default:
		break;
	}

	return MB_EPORTERR;

}
