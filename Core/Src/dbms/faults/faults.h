/** 
 * 
 * Distributed BMS      Fault-Handling System
 *
 * Copyright (C) 2025   Texas A&M University
 * 
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#ifndef _FAULTS_H_ 
#define _FAULTS_H_

#include "../utils/common.h"
#include "../context.h"
#include "../settings.h"
#include "../vinterface.h"

/*
 *  Fault types, ordinal determines position in bitmask
 */
typedef enum
{
    CTRL_FAULT_VOLTAGE_OVER = 0, // 1...
    CTRL_FAULT_VOLTAGE_UNDER = 1,
    CTRL_FAULT_TEMP_OVER = 2,
    CTRL_FAULT_TEMP_UNDER = 3,
    CTRL_FAULT_CURRENT_OVER = 4,
    CTRL_FAULT_CURRENT_UNDER = 5,
    CTRL_FAULT_PACK_VOLTAGE_OVER = 6,
    CTRL_FAULT_PACK_VOLTAGE_UNDER = 7,
    CTRL_FAULT_MAX_DELTA_EXCEEDED = 8,
    CTRL_FAULT_STACK_FAULT = 9,
    CTRL_FAULT_CURRENT_PULSE = 10,
    CTRL_FAULT_CAN_FAIL = 11,
    CTRL_FAULT_STACK_DISCONNECT = 12,
    CTRL_FAULT_MONITOR = 13,
    CTRL_FAULT_TYPE_COUNT // Total number of fault types -- should be last
} CtrlFault;

typedef enum
{
    CTRL_KEEP_MAX,
    CTRL_KEEP_MIN,
    CTRL_KEEP_FIRST,
    CTRL_KEEP_LATEST,
    CTRL_KEEP_BITS
} CtrlFaultSaveMode;

typedef enum
{
    CTRL_TX_FAILS = 0,
    CTRL_ISENSE_DC,
    CTRL_BUSOFF
} CtrlCANFaults;

#define BIT(fault) (1U << (fault))

#define NONMASKABLE_FAULTS ((uint32_t) (    \
    BIT(CTRL_FAULT_VOLTAGE_OVER) |          \
    BIT(CTRL_FAULT_VOLTAGE_UNDER) |         \
    BIT(CTRL_FAULT_TEMP_OVER) |             \
    BIT(CTRL_FAULT_CURRENT_OVER) |          \
    BIT(CTRL_FAULT_PACK_VOLTAGE_OVER) |     \
    BIT(CTRL_FAULT_PACK_VOLTAGE_UNDER) |    \
    BIT(CTRL_FAULT_MAX_DELTA_EXCEEDED)      \
))

// #define NONMASKABLE_FAULTS 0

#define CLAMP_U16(x) ((uint16_t)((x) < 0 ? 0 : ((x) > 65535 ? 65535 : (x))))

void CtrlUpdateFaults(DbmsCtx* ctx);

void CtrlSetFault(DbmsCtx* ctx, CtrlFault fault, uint8_t cell, uint16_t value);
bool CtrlHasFault(DbmsCtx* ctx, CtrlFault fault);
bool CtrlHasStickyFault(DbmsCtx* ctx, CtrlFault fault);

bool CtrlHasAnyFaults(DbmsCtx* ctx);
bool CtrlHasAnyHardFaults(DbmsCtx* ctx);
bool CtrlHasAnyWarnings(DbmsCtx* ctx);
void CtrlClearAllFaults(DbmsCtx* ctx);

void CtrlSetFaultConfig(DbmsCtx* ctx, uint32_t warnings, uint32_t nonlatching);

void CheckVoltageFaults(DbmsCtx* ctx);
void CheckTemperatureFaults(DbmsCtx* ctx);
void CheckCurrentFaults(DbmsCtx* ctx);
void CheckStackFaults(DbmsCtx* ctx);

void SetFaultLine(DbmsCtx* ctx, bool faulted);
int LoadFaultState(DbmsCtx* ctx);
int SaveFaultState(DbmsCtx* ctx);

#endif