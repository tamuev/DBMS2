/**
 *
 * Distributed BMS      EEPROM Storage Interface
 *
 * Copyright (C) 2025   Texas A&M University
 *
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#ifndef _STORAGE_H_
#define _STORAGE_H_

#include "utils/common.h"
#include "context.h"

#include "can.h"

#define EEPROM_WRITE_TIMEOUT            100     // todo: drop
#define EEPROM_READ_TIMEOUT             100     //

#define EEPROM_ADDR                     (0x50 << 1)
#define EEPROM_PAGE_SIZE                256

#define EEPROM_SETTINGS_ADDR            (0 * EEPROM_PAGE_SIZE)
#define EEPROM_CTRL_FAULT_MASK_ADDR     (1 * EEPROM_PAGE_SIZE)
#define EEPROM_INITIAL_CHARGE           (1 * EEPROM_PAGE_SIZE + 32)
#define EEPROM_BLACKBOX_META_ADDR       (2 * EEPROM_PAGE_SIZE)
#define EEPROM_BLACKBOX_BASE_ADDR       (3 * EEPROM_PAGE_SIZE)
#define EEPROM_DEBUG                    (4 * EEPROM_PAGE_SIZE)
#define EEPROM_BAD_THERM_MASK_ADDR      (5 * EEPROM_PAGE_SIZE)

#define ERR_CRC_MISMATCH 24

int WriteEEPROM(DbmsCtx* ctx, uint32_t addr, uint8_t* data, uint16_t len);
int ReadEEPROM(DbmsCtx* ctx, uint32_t addr, uint8_t* data, uint16_t len);

int SaveStoredObject(DbmsCtx* ctx, uint32_t mem_addr, void* object, size_t obj_size);
int LoadStoredObject(DbmsCtx* ctx, uint32_t mem_addr, void* object, size_t obj_size);

#endif