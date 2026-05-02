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
#include "settings.h"

uint32_t GetSetting(DbmsCtx* ctx, UserSettingIndex index)
{
    // CanLog(ctx, "GS %d %c\n", index, ctx->settings ? 'Y' : 'N');
    return ctx->settings->user_defined[index];
}

void SetSetting(DbmsCtx* ctx, UserSettingIndex index, uint32_t new_val)
{
    ctx->settings->user_defined[index] = new_val;
}

bool IsValidSettingIndex(DbmsCtx* ctx, uint32_t index)
{
    return index < sizeof(ctx->settings->user_defined);
}

int LoadSettings(DbmsCtx* ctx)
{
    return LoadStoredObject(ctx, EEPROM_SETTINGS_ADDR, ctx->settings, sizeof(DbmsSettings));
}

int SaveSettings(DbmsCtx* ctx)
{
    return SaveStoredObject(ctx, EEPROM_SETTINGS_ADDR, ctx->settings, sizeof(DbmsSettings));
}

void LoadFallbackSettings(DbmsCtx* ctx)
{
    ctx->settings->user_defined[QUIET_MS_BEFORE_SHUTDOWN] = 2000; // 2s

    ctx->settings->user_defined[MAX_GROUP_VOLTAGE] = 4200;  // 1v
    ctx->settings->user_defined[MIN_GROUP_VOLTAGE] = 2500;  // 5v
    ctx->settings->user_defined[MAX_PACK_VOLTAGE] = 600000; // 600v
    ctx->settings->user_defined[MIN_PACK_VOLTAGE] = 30000;  // 30v

    ctx->settings->user_defined[MAX_THERMISTOR_TEMP] = 45; // todo: ?
    ctx->settings->user_defined[MAX_CURRENT] = 50;       // todo: ?
    ctx->settings->user_defined[DYNAMIC_V_MIN] = 2600;
    ctx->settings->user_defined[MAX_V_DELTA] = 100;
    ctx->settings->user_defined[MS_BEFORE_FAULT_CHECKS] = 10000;

    ctx->settings->user_defined[PULSE_LIMIT_CURRENT] = 90;
    ctx->settings->user_defined[PULSE_LIMIT_TIME_MS] = 3000;

    ctx->settings->user_defined[OVERTEMP_MS] = 1000;

    ctx->settings->user_defined[TEMP_CURVE_A] = 50;
    ctx->settings->user_defined[TEMP_CURVE_B] = 65;

    ctx->settings->user_defined[IGNORE_BAD_THERMS] = 1;

    ctx->settings->user_defined[CH_BAL_DELTA_BEGIN] = 200.0;
    ctx->settings->user_defined[CH_BAL_DELTA_END] = 150.0;
    ctx->settings->user_defined[CH_BAL_MIN_V] = 3800;
    ctx->settings->user_defined[CH_BAL_T_IDX] = 2;
    ctx->settings->user_defined[CH_TARGET_V] = 4200;
    ctx->settings->user_defined[CH_I] = 18;
    ctx->settings->user_defined[CH_AC_VOLTAGE] = 240;
    ctx->settings->user_defined[CH_ELCON_EFF] = 90;

    ctx->settings->user_defined[FAN_T_TH] = 30; // todo: ?
    ctx->settings->user_defined[FAN_DUTY] = 100;

    ctx->settings->user_defined[LOW_PLAUSIBLE_TEMP] = 10;
    ctx->settings->user_defined[HIGH_PLAUSIBLE_TEMP] = 120;
  
    ctx->settings->user_defined[PRECHARGE_ON_TH] = 90;
    ctx->settings->user_defined[PRECHARGE_OFF_TH] = 10;
    
    ctx->settings->user_defined[QUITE_STACK_FAULT_TICKS] = 20;
    ctx->settings->user_defined[MS_BEFORE_CAN_FAIL] = 1000000;

    ctx->settings->user_defined[CAN_BUSOFF_RECOVERY_MS] = 1000;
    ctx->settings->user_defined[CAN_BUSOFF_GOOD_MS] = 100;
}
