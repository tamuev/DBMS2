/**
 *
 * Distributed BMS      Configuration/Setting System
 *
 * Copyright (C) 2025   Texas A&M University
 *
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#ifndef _H_SETTINGS_H
#define _H_SETTINGS_H

#include "utils/common.h"
#include "context.h"
#include "storage.h"

// these objects are fwd declared

enum _UserSettingIndex
{
    __UNUSED = 0,

    QUIET_MS_BEFORE_SHUTDOWN,
    MAX_GROUP_VOLTAGE,
    MIN_GROUP_VOLTAGE,
    MAX_PACK_VOLTAGE,
    MIN_PACK_VOLTAGE,

    MAX_THERMISTOR_TEMP,
    MAX_CURRENT,

    MAX_V_DELTA,
    DYNAMIC_V_MIN,

    MS_BEFORE_FAULT_CHECKS,

    PULSE_LIMIT_CURRENT,
    PULSE_LIMIT_TIME_MS,
    OVERTEMP_MS,

    TEMP_CURVE_A,
    TEMP_CURVE_B,

    IGNORE_BAD_THERMS,

    CH_BAL_DELTA_BEGIN,
    CH_BAL_DELTA_END,
    CH_BAL_MIN_V,
    CH_BAL_T_IDX,
    CH_TARGET_V,
    CH_I,
    CH_AC_VOLTAGE,
    CH_ELCON_EFF,

    FAN_ON_TEMP,
    FAN_OFF_TEMP,

    LOW_PLAUSIBLE_TEMP,
    HIGH_PLAUSIBLE_TEMP,

    PRECHARGE_ON_TH,
    PRECHARGE_OFF_TH,

    QUITE_STACK_FAULT_TICKS,
    MS_BEFORE_CAN_FAIL,

    CAN_BUSOFF_RECOVERY_MS,
    CAN_BUSOFF_GOOD_MS,

    VOLTAGE_DELTA_TIMEOUT_MS,
    VOLTAGE_OVER_TIMEOUT_MS,
    VOLTAGE_UNDER_TIMEOUT_MS,

    __NUM_USER_DEFINED_SETTINGS // ALWAYS LAST
};

/**
 * This struct is legacy -- the settings used to be items in
 * the struct, but user defined settings were changed to
 * use an array
 */
struct _DbmsSettings
{

    uint32_t user_defined[__NUM_USER_DEFINED_SETTINGS];
};

uint32_t GetSetting(DbmsCtx* ctx, UserSettingIndex index);
void SetSetting(DbmsCtx* ctx, UserSettingIndex index, uint32_t new_val);
bool IsValidSettingIndex(DbmsCtx* ctx, uint32_t index);

int LoadSettings(DbmsCtx* ctx);
int SaveSettings(DbmsCtx* ctx);

void LoadFallbackSettings(DbmsCtx* ctx);

#endif