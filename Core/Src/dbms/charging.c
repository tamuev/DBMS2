/**
 *
 * Distributed BMS      Charging State-Machine Controller
 *
 * Copyright (C) 2025   Texas A&M University
 *
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#include "charging.h"
#include "context.h"
#include "ledctl.h"
#include "stack.h"
#include "vinterface.h"

bool ElconConnected(DbmsCtx* ctx)
{
    return (HAL_GetTick() - ctx->elcon.heartbeat < GetSetting(ctx, QUIET_MS_BEFORE_SHUTDOWN)) && !CtrlHasAnyHardFaults(ctx);
}

bool ChargingTimeout(DbmsCtx* ctx)
{
    return HAL_GetTick() - ctx->charging.heartbeat < GetSetting(ctx, QUIET_MS_BEFORE_SHUTDOWN);
}

bool ChargingConnected(DbmsCtx* ctx)
{
    return ctx->flags.active && /*ctx->charging.allowed &&*/ ElconConnected(ctx) && ctx->j1772.pp_connect;
}

bool NeedsToBalance(DbmsCtx* ctx) // TODO: this is the start condition
{
    // TODO: condition for BMS temperature < some threshold
    // TODO: if we have a cell at max V
    return 1000 * (ctx->stats.max_v - ctx->stats.min_v) > GetSetting(ctx, CH_BAL_DELTA_BEGIN) ||
           (1000 * ctx->stats.max_v > GetSetting(ctx, CH_TARGET_V) &&
            1000 * (ctx->stats.max_v - ctx->stats.min_v) > GetSetting(ctx, CH_BAL_DELTA_END));
}

bool NeedsToBalanceMore(DbmsCtx* ctx)
{
    return (ctx->charging.pre_bal_max_v - ctx->charging.pre_bal_min_v) > GetSetting(ctx, CH_BAL_DELTA_END);
    // && 1000 * ctx->stats.min_v > GetSetting(ctx, CH_BAL_MIN_V);
}

bool ChargingComplete(DbmsCtx* ctx)
{
    return ctx->stats.max_v > GetSetting(ctx, CH_TARGET_V) &&
           1000 * (ctx->stats.max_v - ctx->stats.min_v) < GetSetting(ctx, CH_BAL_DELTA_END);
}

void ChargingComputeElconReq(DbmsCtx* ctx)
{
    ctx->elcon.v_req = MIN(GetSetting(ctx, CH_TARGET_V) * N_GROUPS_PER_SIDE * N_SIDES, 600000) / 1000;
    uint32_t ac_current = MIN(ctx->j1772.maxCurrentSupply, GetSetting(ctx, CH_I));
    float elcon_eff = CLAMP((int32_t)GetSetting(ctx, CH_ELCON_EFF), 0, 100) / 100.0;
    uint32_t power_lim = ac_current * GetSetting(ctx, CH_AC_VOLTAGE) * elcon_eff;
    ctx->elcon.i_req = power_lim / ctx->elcon.v_req;
}

void ChargingAccumulateVoltages(DbmsCtx* ctx)
{
    for (size_t side = 0; side < N_SIDES; side++)
    {
        for (size_t group = 0; group < N_GROUPS_PER_SIDE; group++)
        {
            ctx->charging.pre_bal_accumulator[side][group] += ctx->cell_states[side].voltages[group];
        }
    }
    ctx->charging.pre_bal_sample_count++;
}

void ChargingComputePreBalanceAverages(DbmsCtx* ctx)
{
    if (ctx->charging.pre_bal_sample_count == 0) return;

    for (size_t side = 0; side < N_SIDES; side++)
    {
        for (size_t group = 0; group < N_GROUPS_PER_SIDE; group++)
        {
            ctx->charging.pre_bal_average_v[side][group] =
                    ctx->charging.pre_bal_accumulator[side][group] / ctx->charging.pre_bal_sample_count;
        }
    }

    ctx->charging.pre_bal_min_v = 99999999.0f;
    ctx->charging.pre_bal_max_v = 0.0f;

    for (size_t side = 0; side < N_SIDES; side++)
    {
        for (size_t group = 0; group < N_GROUPS_PER_SIDE; group++)
        {
            ctx->charging.pre_bal_min_v =
                    MIN(ctx->charging.pre_bal_min_v, ctx->charging.pre_bal_average_v[side][group]);
            ctx->charging.pre_bal_max_v =
                    MAX(ctx->charging.pre_bal_max_v, ctx->charging.pre_bal_average_v[side][group]);
        }
    }
}

static uint32_t bal_times[] = {0, 10, 30, 60};
#define LOOKUP_BAL_TIMES(T)       (bal_times[MIN((uint8_t)T, (uint8_t)__N_BAL_TIMES - 1)])
#define TIME_IN_STATE_MS(CTX_PTR) (HAL_GetTick() - CTX_PTR->charging.state_enter_ts)

bool DoneBalancing(DbmsCtx* ctx)
{
    int32_t bal_t_idx = GetSetting(ctx, CH_BAL_T_IDX);
    int32_t times = LOOKUP_BAL_TIMES(bal_t_idx) * 1000 + 6000;
    // CanLog(ctx, "Times %d\n", times);
    return TIME_IN_STATE_MS(ctx) > times; // TODO: impl condition based on status
}

void ChargingEnterState(DbmsCtx* ctx, ChargingState new_state)
{
    if (ctx->charging.state == new_state) return;

    ctx->charging.prev_state = ctx->charging.state;
    ctx->charging.state = new_state;
    ctx->charging.state_enter_ts = HAL_GetTick();

    switch (new_state)
    {
    case CH_NO_CONN:
        CanLog(ctx, "Enter NC\n");
        for (int i = 0; i < N_SIDES; i++)
        {
            memset(ctx->cell_states[i].cells_to_balance, 0, sizeof(ctx->cell_states[i].cells_to_balance));
        }
        break;
    case CH_CHARGING:
        CanLog(ctx, "Enter Ch\n");
        for (int i = 0; i < N_SIDES; i++)
        {
            memset(ctx->cell_states[i].cells_to_balance, 0, sizeof(ctx->cell_states[i].cells_to_balance));
        }
        break;
    case CH_WAIT_1:
    case CH_WAIT_2:
        CanLog(ctx, "Enter W%d\n", new_state == CH_WAIT_1 ? 1 : 2);
        for (int i = 0; i < N_SIDES; i++)
        {
            memset(ctx->cell_states[i].cells_to_balance, 0, sizeof(ctx->cell_states[i].cells_to_balance));
            memset(ctx->charging.pre_bal_accumulator, 0, sizeof(ctx->charging.pre_bal_accumulator));
        }
        ctx->charging.pre_bal_sample_count = 0;
        break;
    case CH_BALANCING_EVENS:
        CanLog(ctx, "Enter BalE\n");
        StackComputeCellsToBalance(ctx, false, GetSetting(ctx, CH_BAL_DELTA_END));
        StackStartBalancing(ctx, false, GetSetting(ctx, CH_BAL_T_IDX));
        break;
    case CH_BALANCING_ODDS:
        CanLog(ctx, "Enter BalO\n");
        StackComputeCellsToBalance(ctx, true, GetSetting(ctx, CH_BAL_DELTA_END));
        // Sends balance timers and starts charging:
        StackStartBalancing(ctx, true, GetSetting(ctx, CH_BAL_T_IDX));
        break;
    case CH_COMPLETE:
        CanLog(ctx, "Enter Cmpl\n");
        for (int i = 0; i < N_SIDES; i++)
        {
            memset(ctx->cell_states[i].cells_to_balance, 0, sizeof(ctx->cell_states[i].cells_to_balance));
        }
        break;
    }
    SendCellsToBalance(ctx);
}

void ChargingUpdate(DbmsCtx* ctx)
{
    J1772ReadState(ctx);

    ctx->charging.only_balance = (HAL_GetTick() - ctx->charging.bal_loop_hb < 3000) && !CtrlHasAnyHardFaults(ctx);

    ctx->charging.conn = ChargingConnected(ctx);
    // if (charge_conn && !ctx->charging.conn)         ;       // can register an action here
    // else if (!charge_conn && ctx->charging.conn)    ;       // can register an action here
    // ctx->charging.conn = charge_conn;

    HAL_Delay(1);

    // CanLog(ctx, "CH = %d\tTH = %d\n", ctx->charging.state, GetSetting(ctx, CH_BAL_DELTA));

    // Seperate condition to detect no connection because it is a
    // global transition that would need to be implemented in every state
    if (!ctx->charging.conn && !ctx->charging.only_balance)
    {
        ChargingEnterState(ctx, CH_NO_CONN);
    }

    switch (ctx->charging.state) // Main state switch
    {
    case CH_NO_CONN:
        SendElconRequest(ctx, 0, 0, 1);

        if (ctx->charging.only_balance) ChargingEnterState(ctx, CH_WAIT_1);
        if (ctx->charging.conn) ChargingEnterState(ctx, CH_CHARGING);
        break;

    case CH_CHARGING:
        ctx->led_state = LED_CHARGING;

        ChargingComputeElconReq(ctx);
        SendElconRequest(ctx, ctx->elcon.v_req, ctx->elcon.i_req, 0);
        CanLog(ctx, "Elcon V=%d I=%d\n", ctx->elcon.v_req, ctx->elcon.i_req);

        // if (ctx->charging.only_balance) ChargingEnterState(ctx, CH_WAIT_1);
        if (TIME_IN_STATE_MS(ctx) > 1000) // TODO:?
        {
            if (NeedsToBalance(ctx)) ChargingEnterState(ctx, CH_WAIT_1);
        }

        break;
    case CH_WAIT_1:
        SendElconRequest(ctx, 0, 0, 1);
        ctx->led_state = LED_CHARGING_WAIT;
        if (TIME_IN_STATE_MS(ctx) > 3000)
        {
            ChargingComputePreBalanceAverages(ctx);
            ChargingEnterState(ctx, CH_BALANCING_EVENS);
        }
        else
        {
            if (TIME_IN_STATE_MS(ctx) > 1000) ChargingAccumulateVoltages(ctx);
        }
        break;
    case CH_WAIT_2:
        SendElconRequest(ctx, 0, 0, 1);
        ctx->led_state = LED_CHARGING_WAIT;
        if (TIME_IN_STATE_MS(ctx) > 3000)
        {
            ChargingComputePreBalanceAverages(ctx);
            // Check if we need more balancing
            if (NeedsToBalanceMore(ctx)) 
            {
                ChargingEnterState(ctx, CH_BALANCING_EVENS);
            }
            else 
            {
                if (ctx->charging.only_balance)
                    ChargingEnterState(ctx, CH_COMPLETE);
                else
                    ChargingEnterState(ctx, CH_CHARGING);
            }
        }
        else
        {
            ChargingAccumulateVoltages(ctx);
        }
        break;
    case CH_BALANCING_EVENS:
        ctx->led_state = LED_BALANCING_EVENS;
        SendElconRequest(ctx, 0, 0, 1);
        SendCellsToBalance(ctx);

        // Check if we actually have cells to balance
        bool balance_evens = StackNeedsToBalance(ctx, false, GetSetting(ctx, CH_BAL_DELTA_END));

        if (!balance_evens)
        {
            // No cells to balance, skip to odds
            ChargingEnterState(ctx, CH_BALANCING_ODDS);
        }
        else if (DoneBalancing(ctx))
        {
            // Always go to odds after evens
            ChargingEnterState(ctx, CH_BALANCING_ODDS);
        }
        break;
    case CH_BALANCING_ODDS:
        ctx->led_state = LED_BALANCING_ODDS;
        SendElconRequest(ctx, 0, 0, 1);
        SendCellsToBalance(ctx);

        // Check if we actually have cells to balance
        bool balance_odds = StackNeedsToBalance(ctx, true, GetSetting(ctx, CH_BAL_DELTA_END));

        if (!balance_odds)
        {
            // No cells to balance, skip to wait 2
            ChargingEnterState(ctx, CH_WAIT_2);
        }
        else if (DoneBalancing(ctx))
        {
            // Always go to wait 2 after odds
            ChargingEnterState(ctx, CH_WAIT_2);
        }

        break;
    case CH_COMPLETE:
        ctx->led_state = LED_CHARGING_COMPLETE;
        SendElconRequest(ctx, 0, 0, 1);

        break;
    }
}
