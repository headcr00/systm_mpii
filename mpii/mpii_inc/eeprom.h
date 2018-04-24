

#ifndef EEPROM_H_
#define EEPROM_H_
#include "stm32f1xx.h"
enum{
	EEPROM1,
	EEPROM2,
	EEPROM_NONE
};

//opcodes
#define EEPROM_WREN 	0x06
#define EEPROM_WRDI 	0x04
#define EEPROM_RDSR 	0x05
#define EEPROM_WRSR 	0x01
#define EEPROM_READ 	0x03
#define EEPROM_WRITE 	0x02

#define EEPROM_SR_READY 	0x00
#define EEPROM_SR_BUSY	 	0x01
#define EEPROM_SR_WRE	 	0x02
#define EEPROM_SR_WRPEN	 	0x80

#define EEPROM_TIMEOUT 0xFFFF

extern SPI_HandleTypeDef hspi3;
//public functions
void eeprom_spi3_init();
void eeprom_write_page(uint8_t fl, uint32_t addr, uint8_t * buffer, uint16_t len);
void eeprom_read_page(uint8_t fl, uint32_t addr, uint8_t * buffer, uint16_t len);
uint8_t eeprom_read_status_register(uint8_t fl);
uint8_t eeprom_write_status_register(uint8_t fl);
#endif /* EEPROM_SPI_H_ */
