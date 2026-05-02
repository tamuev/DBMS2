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
#include "faults.h"
#include "../blackbox.h"
#include "../settings.h"

CtrlFaultSaveMode fault_save_modes[CTRL_FAULT_TYPE_COUNT] = {
    [CTRL_FAULT_VOLTAGE_OVER] = CTRL_KEEP_MAX,
    [CTRL_FAULT_VOLTAGE_UNDER] = CTRL_KEEP_MIN,
    [CTRL_FAULT_TEMP_OVER] = CTRL_KEEP_MAX,
    [CTRL_FAULT_TEMP_UNDER] = CTRL_KEEP_MIN,
    [CTRL_FAULT_CURRENT_OVER] = CTRL_KEEP_MAX,
    [CTRL_FAULT_CURRENT_UNDER] = CTRL_KEEP_MIN,
    [CTRL_FAULT_PACK_VOLTAGE_OVER] = CTRL_KEEP_MAX,
    [CTRL_FAULT_PACK_VOLTAGE_UNDER] = CTRL_KEEP_MIN,
    [CTRL_FAULT_MAX_DELTA_EXCEEDED] = CTRL_KEEP_MAX,
    [CTRL_FAULT_STACK_FAULT] = CTRL_KEEP_FIRST,
    [CTRL_FAULT_CURRENT_PULSE] = CTRL_KEEP_MAX,
    [CTRL_FAULT_CAN_FAIL] = CTRL_KEEP_BITS,
    [CTRL_FAULT_STACK_DISCONNECT] = CTRL_KEEP_BITS
};

void CtrlUpdateFaults(DbmsCtx* ctx)
{
    ctx->faults.active_faults = 0;

    if (ctx->flags.req_fault_clear)
    {
        CtrlClearAllFaults(ctx);
        ctx->flags.req_fault_clear = false;
    }

    CheckVoltageFaults(ctx);
    ctx->profiling.times.T6 = GetUs(ctx);

    CheckCurrentFaults(ctx);
    ctx->profiling.times.T7 = GetUs(ctx);

    CheckTemperatureFaults(ctx);
    ctx->profiling.times.T8 = GetUs(ctx);

    CheckStackFaults(ctx);

    // Update fault line
    bool hard_fault = CtrlHasAnyHardFaults(ctx);

    SetFaultLine(ctx, hard_fault);

    if (!ctx->faults.had_fault && hard_fault)
    {
        ctx->flags.need_to_save_faults = true;
        ctx->flags.need_to_save_blackbox = true;
    }
    ctx->faults.had_fault = hard_fault;

    ctx->faults.historic_faults |= ctx->faults.active_faults;
    ctx->faults.latched_faults |= ctx->faults.active_faults & (~ctx->faults.nonlatching_config | NONMASKABLE_FAULTS);
}

/**
 * Sets a specific fault. Only call this from one of the Check___Faults functions!
 */
void CtrlSetFault(DbmsCtx* ctx, CtrlFault fault, uint8_t cell, uint16_t value)
{
    if (fault >= CTRL_FAULT_TYPE_COUNT) return;

    FaultData* data = &ctx->faults.fault_data[fault];

    bool save_value = false;
    switch (fault_save_modes[fault]) 
    {
        case CTRL_KEEP_FIRST:
            save_value = !CtrlHasStickyFault(ctx, fault);
            break;
        case CTRL_KEEP_LATEST:
            save_value = true;
            break;
        case CTRL_KEEP_MAX:
            save_value = (value > data->value) || !CtrlHasStickyFault(ctx, fault);
            break;
        case CTRL_KEEP_MIN:
            save_value = (value < data->value) || !CtrlHasStickyFault(ctx, fault);
            break;
        case CTRL_KEEP_BITS:
            save_value = (value & ~data->value);
            data->value |= value;
            break;
    }

    if (!CtrlHasStickyFault(ctx, fault) && data->n_throws < 255) data->n_throws++;
    if (save_value)
    {
        data->cell = cell;
        data->value = value;
    }

    ctx->faults.active_faults |= BIT(fault);
}

bool CtrlHasFault(DbmsCtx* ctx, CtrlFault fault)
{
    if (fault >= CTRL_FAULT_TYPE_COUNT) return false;
    return (ctx->faults.active_faults & BIT(fault));
}

bool CtrlHasStickyFault(DbmsCtx* ctx, CtrlFault fault)
{
    if (fault >= CTRL_FAULT_TYPE_COUNT) return false;
    return (ctx->faults.historic_faults & BIT(fault));
}


/**
 * Whether the controller has any active or latched hardfaults or warnings
 */
bool CtrlHasAnyFaults(DbmsCtx* ctx)
{
    return (ctx->faults.active_faults | ctx->faults.latched_faults) != 0;
}

/**
 * Whether the controller has any active or latched hardfaults
 */
bool CtrlHasAnyHardFaults(DbmsCtx* ctx)
{
    return ((ctx->faults.active_faults | ctx->faults.latched_faults) 
        & (~ctx->faults.warnings_config | NONMASKABLE_FAULTS)) != 0;
}

/**
 * Whether the controller has any active or latched warnings
 */
bool CtrlHasAnyWarnings(DbmsCtx* ctx)
{
    return ((ctx->faults.active_faults | ctx->faults.latched_faults) 
        & (ctx->faults.warnings_config & ~NONMASKABLE_FAULTS)) != 0;
}

void CtrlClearAllFaults(DbmsCtx* ctx)
{
    ctx->faults.active_faults = 0;
    ctx->faults.latched_faults = 0;
    ctx->faults.historic_faults = 0;

    for (size_t i = 0; i < sizeof(ctx->faults.fault_data) / sizeof(ctx->faults.fault_data[0]); ++i)
    {
        ctx->faults.fault_data[i].cell = 0xFF;
        ctx->faults.fault_data[i].n_throws = 0;
        ctx->faults.fault_data[i].value = 0;
    }

    ctx->flags.need_to_save_faults = true;
}

void CtrlSetFaultConfig(DbmsCtx* ctx, uint32_t warnings, uint32_t nonlatching)
{
    ctx->faults.warnings_config = (warnings & ~NONMASKABLE_FAULTS);
    ctx->faults.nonlatching_config = (nonlatching & ~NONMASKABLE_FAULTS);
    ctx->faults.latched_faults &= ~ctx->faults.nonlatching_config;
    ctx->flags.need_to_save_faults = true;

    CanLog(ctx, "cf w=%x nl=%x\n", ctx->faults.warnings_config, ctx->faults.nonlatching_config);
}

void SetFaultLine(DbmsCtx* ctx, bool faulted)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, !((bool)faulted));
    ctx->stats.fault_line_faulted = faulted;
}

int SaveFaultState(DbmsCtx* ctx)
{
    uint16_t faults_crc = CalcCrc16((uint8_t*)&ctx->faults, sizeof(ctx->faults));
    if (ctx->faults_crc != faults_crc)
    {
        ctx->faults_crc = faults_crc;
        SaveStoredObject(ctx, EEPROM_CTRL_FAULT_MASK_ADDR, &ctx->faults, sizeof(ctx->faults));
        CanLog(ctx, "Saved Faults!\n");
    }
    return 0;
}

int LoadFaultState(DbmsCtx* ctx)
{
    int status;
    // Right now this only references the controller mask. When "smart" / "verbose" faults
    // for other components on the bus are implemented, we will deal with this.
    if ((status = LoadStoredObject(ctx, EEPROM_CTRL_FAULT_MASK_ADDR,
            &ctx->faults, sizeof(ctx->faults))))
    {
        // todo: check an error here
    }
    ctx->faults.active_faults = 0;
    ctx->faults.warnings_config &= ~NONMASKABLE_FAULTS;
    ctx->faults.nonlatching_config &= ~NONMASKABLE_FAULTS;
    ctx->faults_crc = CalcCrc16((uint8_t*)&ctx->faults, sizeof(ctx->faults));
    ctx->faults.had_fault = CtrlHasAnyFaults(ctx);
    return status;
}
