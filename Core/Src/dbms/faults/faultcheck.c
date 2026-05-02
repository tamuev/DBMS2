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

void CheckVoltageFaults(DbmsCtx* ctx)
{
    uint32_t max_group_v = GetSetting(ctx, MAX_GROUP_VOLTAGE);
    uint32_t min_group_v = GetSetting(ctx, MIN_GROUP_VOLTAGE);
    uint32_t max_v_delta = GetSetting(ctx, MAX_V_DELTA);
    
    if (ctx->stats.max_v * 1000 > max_group_v) {
        CtrlSetFault(ctx, CTRL_FAULT_VOLTAGE_OVER, ctx->stats.max_v_cell, CLAMP_U16(ctx->stats.max_v * 1000));
    }
    if (ctx->stats.min_v * 1000 < min_group_v) {
        CtrlSetFault(ctx, CTRL_FAULT_VOLTAGE_UNDER, ctx->stats.min_v_cell, CLAMP_U16(ctx->stats.min_v * 1000));
    }
    if ((ctx->stats.max_v - ctx->stats.min_v) * 1000 > max_v_delta) {
        // Find cell voltage furthest from average
        uint16_t avg_mv = (uint16_t)(ctx->stats.avg_v * 1000.0f);
        uint16_t max_diff = 0;
        uint8_t cell = CELL_BYTE_NA;

        for (int i = 0; i < N_SIDES; i++)
        {
            for (int j = 0; j < N_GROUPS_PER_SIDE; j++)
            {
                uint16_t v = CLAMP_U16(ctx->cell_states[i].voltages[j]);
                uint16_t diff = v > avg_mv ? v - avg_mv : avg_mv - v;
                if (diff > max_diff)
                {
                    max_diff = diff;
                    cell = CELL_BYTE(i, j);
                }
            }
        }

        CtrlSetFault(ctx, CTRL_FAULT_MAX_DELTA_EXCEEDED, cell, CLAMP_U16((ctx->stats.max_v - ctx->stats.min_v) * 1000));
    }
}

void CheckTemperatureFaults(DbmsCtx* ctx)
{
    if (ctx->stats.max_t < GetSetting(ctx, MAX_THERMISTOR_TEMP))    // temp is ok 
    {
        ctx->timing.overtemp_last_ok_ts = HAL_GetTick();
    }
    else {
        if (HAL_GetTick() - ctx->timing.overtemp_last_ok_ts > GetSetting(ctx, OVERTEMP_MS))
            CtrlSetFault(ctx, CTRL_FAULT_TEMP_OVER, ctx->stats.max_t_cell, CLAMP_U16(ctx->stats.max_t * 1000.0f));
    }
}

void CheckCurrentFaults(DbmsCtx* ctx)
{
    int32_t current_ma = MAX(0, ctx->current_sensor.current_ma);

    int32_t at = GetSetting(ctx, TEMP_CURVE_A);
    int32_t bt = GetSetting(ctx, TEMP_CURVE_B);

    // Todo: make this cleaner
    if (ctx->stats.avg_t < at) {
        ctx->max_current_ma = GetSetting(ctx, MAX_CURRENT) * 1000;
    } else {
        if (bt == at) ctx->max_current_ma = 0;
        ctx->max_current_ma = (GetSetting(ctx, MAX_CURRENT) * 1000) * ((bt - ctx->stats.avg_t) / (bt - at));
    }

    // Do the comparison in ma
    if (current_ma > ctx->max_current_ma)
    {
        CtrlSetFault(ctx, CTRL_FAULT_CURRENT_OVER, CELL_BYTE_NA, CLAMP(current_ma, 0, 650000) / 10);
    }   

    if (current_ma > GetSetting(ctx, PULSE_LIMIT_CURRENT) * 1000)
    {
        ctx->timing.pl_pulse_t = HAL_GetTick() - ctx->timing.pl_last_ok_ts;
        if (ctx->timing.pl_pulse_t > GetSetting(ctx, PULSE_LIMIT_TIME_MS))
            CtrlSetFault(ctx, CTRL_FAULT_CURRENT_PULSE, CELL_BYTE_NA, CLAMP(current_ma, 0, 650000) / 10);
    }
    else 
    {
        ctx->timing.pl_pulse_t = 0;
        ctx->timing.pl_last_ok_ts = HAL_GetTick();
    }

    // TODO: need to make ma constructor
    // ctx->current_sensor.ima.current_mavg_ma = ma_calc_i32(ctx->current_sensor.ima.ma, ctx->current_sensor.current_ma);
}

void CheckStackFaults(DbmsCtx* ctx) 
{
    uint16_t disconnected_mask = 0;
    for (int i = 0; i < N_MONITORS; ++i)
    {
        if (ctx->stats.iters - ctx->stats.last_monitor_msg[i] > GetSetting(ctx, QUITE_STACK_FAULT_TICKS))
            disconnected_mask |= BIT(i);
    }

    if (disconnected_mask != 0)
    {
        CtrlSetFault(ctx, CTRL_FAULT_STACK_DISCONNECT, CELL_BYTE_NA, disconnected_mask);
        CanLog(ctx, "dm=%x\n", disconnected_mask);
    }

    if (GetUs(ctx) - ctx->stats.last_can_tx_ts >= GetSetting(ctx, MS_BEFORE_CAN_FAIL) * 1000)
    {
        CtrlSetFault(ctx, CTRL_FAULT_CAN_FAIL, CELL_BYTE_NA, BIT(CTRL_TX_FAILS));
    }

    if (GetUs(ctx) - ctx->stats.last_ivt_rx_ts >= GetSetting(ctx, MS_BEFORE_CAN_FAIL) * 1000)
    {
        CtrlSetFault(ctx, CTRL_FAULT_CAN_FAIL, CELL_BYTE_NA, BIT(CTRL_ISENSE_DC));
    }

    bool can_bus_off = (ctx->hw.can->Instance->ESR & CAN_ESR_BOFF) != 0;
    uint64_t now_us = GetUs(ctx);

    if (can_bus_off)
    {
        ctx->stats.last_can_bad_ts = now_us;
    }

    if (ctx->stats.last_can_bo_ts != 0)
    {
        if (now_us - ctx->stats.last_can_bad_ts >= GetSetting(ctx, CAN_BUSOFF_GOOD_MS) * 1000)
        {
            ctx->stats.last_can_bo_ts = 0;
        }
        else if (now_us - ctx->stats.last_can_bo_ts > GetSetting(ctx, CAN_BUSOFF_RECOVERY_MS) * 1000)
        {
            CtrlSetFault(ctx, CTRL_FAULT_CAN_FAIL, CELL_BYTE_NA, BIT(CTRL_BUSOFF));
        }
    }

    if (ctx->flags.dog_starved)
    {
        CtrlSetFault(ctx, CTRL_FAULT_DOG_STARVED, CELL_BYTE_NA, 0);
        ctx->flags.dog_starved = false;
    }
}
