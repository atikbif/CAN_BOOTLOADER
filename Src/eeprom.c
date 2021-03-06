/*
 * eeprom.c
 *
 *  Created on: 11 ���. 2019 �.
 *      Author: User
 */

#include "eeprom.h"

#define EEPROM_START_ADDR   ADDR_FLASH_PAGE_255
#define EEPROM_PAGE_SIZE	2048

const uint64_t ee_key = 0xBAAAAAAAAAAAAAAA;

static uint32_t PAGEError = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;

static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))  {
	  page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else {
	  page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  return page;
}

static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;
  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))  {bank = FLASH_BANK_1;}
    else {bank = FLASH_BANK_2;}
  }
  else
  {
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE)) {bank = FLASH_BANK_2;}
    else {bank = FLASH_BANK_1;}
  }
  return bank;
}

static uint8_t find_last_written_var_addr(uint16_t* var_addr) {
	uint64_t word1, word2;
	uint8_t res = 0;
	*var_addr=0;
	uint16_t addr=16;

	while(addr+16<=EEPROM_PAGE_SIZE) {
		word1 = *(__IO uint64_t *)(EEPROM_START_ADDR+addr);
		word2 = *(__IO uint64_t *)(EEPROM_START_ADDR+addr+8);
		if(word1==(~word2)) {
			res=1;
			*var_addr=addr;
			addr+=16;
		}else break;
	}
	return res;
}

static uint8_t check_eeprom() {
	uint64_t key_word = *(__IO uint64_t *)(EEPROM_START_ADDR);
	if(key_word!=ee_key) return 0;
	uint16_t addr=16;
	find_last_written_var_addr(&addr);
	addr+=16;
	uint64_t word1, word2;
	while(addr+16<=EEPROM_PAGE_SIZE) {
		word1 = *(__IO uint64_t *)(EEPROM_START_ADDR+addr);
		word2 = *(__IO uint64_t *)(EEPROM_START_ADDR+addr+8);
		if((word1!=0xFFFFFFFFFFFFFFFF) || (word2!=0xFFFFFFFFFFFFFFFF)) return 0;
		addr+=16;
	}
	return 1;
}

uint8_t  init_eeprom() {
	if(check_eeprom()==0) {
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Banks       = GetBank(EEPROM_START_ADDR);
		EraseInitStruct.Page        = GetPage(EEPROM_START_ADDR);
		EraseInitStruct.NbPages     = 1;
		if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError)==HAL_OK) {
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM_START_ADDR, ee_key);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM_START_ADDR+8, 0);
			return 1;
		}
		return 0;
	}
	return 1;
}

uint64_t read_var() {
	uint8_t res = 0;
	uint16_t addr=0;
	res=find_last_written_var_addr(&addr);
	if(res) {
		return *(__IO uint64_t *)(EEPROM_START_ADDR+addr);
	}
	return 0;
}

void write_var(uint64_t value) {
	uint16_t addr=0;
	find_last_written_var_addr(&addr);
	if(addr == EEPROM_PAGE_SIZE-16) {
		// erase page
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Banks       = GetBank(EEPROM_START_ADDR);
		EraseInitStruct.Page        = GetPage(EEPROM_START_ADDR);
		EraseInitStruct.NbPages     = 1;
		if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError)==HAL_OK) {
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM_START_ADDR, ee_key);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM_START_ADDR+8, 0);
			addr=0;
		}
	}
	addr+=16;
	if(addr<=EEPROM_PAGE_SIZE-16) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM_START_ADDR+addr, value);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM_START_ADDR+addr+8, ~value);
	}
}
