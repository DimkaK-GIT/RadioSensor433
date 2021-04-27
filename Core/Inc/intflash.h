#ifndef ___INTFLASH_H
#define ___INTFLASH_H

#include "mem.h"
//#include "stm32f4xx_hal.h"
#include "stm32f1xx_hal.h"

void flash_read(uint32_t adr, uint32_t *data, uint32_t count);
void flash_write(uint32_t adr, uint32_t *data, uint32_t count);
void flash_erase (uint32_t adr);
void flash_copy (uint32_t adr_des, uint32_t adr_src, uint32_t count);
uint32_t flashWordRead(uint32_t address);
uint16_t Prog_CRC16 (uint32_t adr,uint32_t count);
uint32_t Prog_CRC32_1(uint32_t adr, uint32_t size);
void WRCodeError(uint8_t codeError, char* report);
uint32_t FindAdrEmptyIntMem(void);

#endif /*___INTFLASH_H */
