
/* Includes ------------------------------------------------------------------*/
#include "intflash.h"
#include "stm32f1xx_hal.h"
#include "string.h"

extern void    FLASH_PageErase(uint32_t PageAddress);

//
union  
{
  uint32_t 	uint;
  uint8_t 	ch[4];
} uint_char;

/*----------------------------------------------------------------------------*/
// 
/*----------------------------------------------------------------------------*/
void flash_read(uint32_t adr, uint32_t *data, uint32_t count)
{
  
	uint32_t         				i;
  uint32_t        				address;
  HAL_StatusTypeDef				flash_ok = HAL_ERROR;
  
//  uint32_t  Consts[11];
  
  
  //
	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK)
	{
		flash_ok = HAL_FLASH_Unlock();
	}

	address = adr;
  for (i = 0; i < count; i++)
  {
    data[i] = (uint32_t)(*(__IO uint32_t*) address);
    address += 4;
  }

	//
	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK)
	{
		flash_ok = HAL_FLASH_Lock();
	}
}   


/*----------------------------------------------------------------------------*/
// 
/*----------------------------------------------------------------------------*/
void flash_write(uint32_t adr, uint32_t *data, uint32_t count)
{
  uint32_t                 i;
	uint32_t        				address;
  HAL_StatusTypeDef				flash_ok = HAL_ERROR;
  
	address = adr;
	//
	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK)
	{
		flash_ok = HAL_FLASH_Unlock();
	}
	
	//
	for (i = 0; i < count; i++)
	{
		flash_ok = HAL_ERROR;
		while(flash_ok != HAL_OK)
		{
			flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, address, (uint32_t)data[i]);
		}
		address += 4;
	}

	//
	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK)
	{
		flash_ok = HAL_FLASH_Lock();
	}
}
/*----------------------------------------------------------------------------*/
// 
/*----------------------------------------------------------------------------*/

void flash_erase (uint32_t adr)
{
	uint8_t Page;
  HAL_StatusTypeDef				flash_ok = HAL_ERROR;

  
	while(flash_ok != HAL_OK)
	{
		flash_ok = HAL_FLASH_Unlock();
	}

	FLASH_PageErase(adr);

	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK)
	{
		flash_ok = HAL_FLASH_Lock();
	}


}

/*----------------------------------------------------------------------------*/
// копирование блока
/*----------------------------------------------------------------------------*/
void flash_copy (uint32_t adr_des, uint32_t adr_src, uint32_t count)
{

  HAL_StatusTypeDef				flash_ok = HAL_ERROR;
	uint32_t adr_read,data;
	uint32_t i;
	
	//Открываем доступ к памяти
	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK)
	{
		flash_ok = HAL_FLASH_Unlock();
	}
	
	//  пословно копируем данные
	adr_read = adr_src;
	
  for (i = 0; i < count/4; i++)
  {
    data = (uint32_t)(*(__IO uint32_t*) adr_read);
		flash_ok = HAL_ERROR;
		while(flash_ok != HAL_OK)
		{
			flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, adr_des, data);
		}
		
    adr_read += 4;
		adr_des+=4;
  }

	
	//Закрываем доступ к памяти
	flash_ok = HAL_ERROR;
	while(flash_ok != HAL_OK)
	{
		flash_ok = HAL_FLASH_Lock();
	}

}
//---------------------------------------------------------------------------
// Чтение слова из памяти
//---------------------------------------------------------------------------
uint32_t flashWordRead(uint32_t address) 
{
   return (*(__IO uint32_t*) address);
}
//---------------------------------------------------------------------------
// Чтение и подсчет CRC кода программы
//---------------------------------------------------------------------------
uint16_t Prog_CRC16 (uint32_t adr,uint32_t count)
{
  
  uint32_t address;
  uint32_t data;
  uint16_t crc;
  uint8_t i;

  crc = 0xFFFF;
  
  for(address = adr; address < adr + count; address+=4)
  {
    data = flashWordRead(address);
    uint_char.uint = data;
    
    crc ^= uint_char.ch[0] << 8;
    for (i = 0; i < 8; i++)
       crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;

    crc ^= uint_char.ch[1] << 8;
    for (i = 0; i < 8; i++)
       crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;

    crc ^= uint_char.ch[2] << 8;
    for (i = 0; i < 8; i++)
       crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;

    crc ^= uint_char.ch[3] << 8;
    for (i = 0; i < 8; i++)
       crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
       
       
  }
  
  return(crc);
  
}

/*
  Name  : CRC-32
  Poly  : 0x04C11DB7    x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11
                       + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1
  Init  : 0xFFFFFFFF
  Revert: true
  XorOut: 0xFFFFFFFF
  Check : 0xCBF43926 ("123456789")
  MaxLen: 268 435 455 байт (2 147 483 647 бит) - обнаружение
   одинарных, двойных, пакетных и всех нечетных ошибок
*/
uint32_t Prog_CRC32_1(uint32_t adr, uint32_t size)
{
    uint32_t crc_table[256];
    uint32_t crc;
  	uint32_t i, j;
	  uint32_t addres, data;

    for (i = 0; i < 256; i++)
    {
        crc = i;
        for (j = 0; j < 8; j++)
            crc = crc & 1 ? (crc >> 1) ^ 0xEDB88320UL : crc >> 1;

        crc_table[i] = crc;
    };

    crc = 0xFFFFFFFFUL;

//    while (len--)
//        crc = crc_table[(crc ^ *buf++) & 0xFF] ^ (crc >> 8);
    for(addres = adr; addres < adr+size;addres+=4)
		{
      data = flashWordRead(addres);
      uint_char.uint = data;
			
  		crc = crc_table[(crc ^ uint_char.ch[0]++) & 0xFF] ^ (crc >> 8);
  		crc = crc_table[(crc ^ uint_char.ch[1]++) & 0xFF] ^ (crc >> 8);
  		crc = crc_table[(crc ^ uint_char.ch[2]++) & 0xFF] ^ (crc >> 8);
  		crc = crc_table[(crc ^ uint_char.ch[3]++) & 0xFF] ^ (crc >> 8);
		}
    return crc ^ 0xFFFFFFFFUL;
}


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
// Запись кода ошибки во внутреннюю память
/*----------------------------------------------------------------------------*/
void WRCodeError(uint8_t codeError, char* report)
{

}

