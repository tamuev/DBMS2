/**
 *
 * Distributed BMS      DBMS Main Controller
 *
 * Copyright (C) 2025   Texas A&M University
 *
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#include "dbms.h"
#include "context.h"
#include "faults/faults.h"
#include "ledctl.h"
#include "can.h"
#include "blackbox.h"
#include <math.h>


static DbmsSettings mem_settings;

//
//  Allocate space for pointer objects
//
void DbmsAlloc(DbmsCtx* ctx)
{
    ctx->settings = &mem_settings;
    memset(&ctx->stats, 0, sizeof(ctx->stats));
    BlackboxInit(ctx);
}

//
//  Initialization loop
//
void DbmsInit(DbmsCtx* ctx)
{
    int status = 0;
    ctx->flags.active = false;
    ctx->led_state = LED_INIT;

    if ((status = LoadSettings(ctx)) != HAL_OK)
    {
        CAN_REPORT_FAULT(ctx, status);
        if (status == ERR_CRC_MISMATCH)
        {
            // This is a bad situation, but we can still proceed
            // by loading fallback settings? Ideally these should
            // be updated enough to be ok
            LoadFallbackSettings(ctx);
            ctx->flags.need_to_sync_settings = true; // queue up a write
        }
        else
        {
        } // fatal error
    }

    HAL_TIM_Base_Start(ctx->hw.timer);

    if ((status = ConfigElconCan(ctx)) != HAL_OK)
    {
        CAN_REPORT_FAULT(ctx, status);
        ctx->led_state = LED_FIRMWARE_FAULT;
        return;
    }

    if ((status = ConfigCan(ctx)) != HAL_OK)
    {
        CAN_REPORT_FAULT(ctx, status);
        ctx->led_state = LED_FIRMWARE_FAULT;
        return;
    }

    //
    //  Load the fault state (we know this throws CRC mismatch)
    //
    if ((status = LoadFaultState(ctx)) != 0)
    {
        // CanLog(ctx, "fault st %d\n", status);
    }

    //
    //  Load the initial charge (we know this throws CRC mismatch)
    //
    if ((status = LoadQStats(ctx)) != 0)
    {
        // CanLog(ctx, "error loading initial charge %d\n", status);
    }

    if ((status = LoadStoredObject(ctx, EEPROM_BAD_THERM_MASK_ADDR, ctx->bad_therm_mask, sizeof(ctx->bad_therm_mask))) != 0)
    {
        // CanLog(ctx, "error loading bad therm mask %d\n", status);
    }
    ctx->initial_historic_accumulated_loss = ctx->qstats.historic_accumulated_loss;
    ctx->flags.need_to_reset_qstats = false;

    ctx->timing.last_rx_heartbeat = -GetSetting(ctx, QUIET_MS_BEFORE_SHUTDOWN);

    ReadEEPROM(ctx, EEPROM_DEBUG, &ctx->stats.n_int_shutdowns, 1);

    #ifdef HAS_FAN
    InitFan(ctx);
    #endif
    DataInit(ctx);
}

int DbmsPerformWakeup(DbmsCtx* ctx)
{
    int status = 0;
    HAL_Delay(2000);

    // TODO: check these all and make a critical couldnt wake stack fault
    if ((status = StackWake(ctx)) != 0)
    {
        CAN_REPORT_FAULT(ctx, status);
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status; // we are cooked
    }

    HAL_Delay(5);
    StackAutoAddr(ctx);
    HAL_Delay(5);
    StackSetNumActiveCells(ctx, 0x0A);
    HAL_Delay(5);
    StackSetupGpio(ctx);
    HAL_Delay(5);
    StackSetupVoltReadings(ctx); // todo: rn start
    HAL_Delay(5);
    StackConfigTimeout(ctx);

    HAL_Delay(5);
    StackBalancingConfig(ctx);

    HAL_Delay(5);

    ctx->current_sensor.q_offset = 0.0f;
    ctx->current_sensor.has_q_offset = false;

    if ((status = LoadStoredObject(ctx, EEPROM_CTRL_FAULT_MASK_ADDR, &ctx->faults, sizeof(ctx->faults))))
    {
        // todo: check an error here
    }
    if ((status = LoadFaultState(ctx)) != 0)
    {
        // todo: check an error here
    }

    // Bridge_Dev_Conf_FAULT_EN(ctx);
    // DelayUs(ctx, 5000);
    // Stack_Dev_Conf_FAULT_EN(ctx);

    ConfigCurrentSensor(ctx, 10);

    DelayUs(ctx, 5000);
    // SetFaultMasks(ctx);

    ctx->timing.wakeup_ts = HAL_GetTick();

    HAL_Delay(10);
    return status;
}

int DbmsPerformShutdown(DbmsCtx* ctx, bool shutdown_stack)
{
    int status = 0;
    if (shutdown_stack) {
        if ((status = StackShutdown(ctx)) != 0)
        {
            CAN_REPORT_FAULT(ctx, status);
            ctx->led_state = LED_FIRMWARE_FAULT;
            return status;
        }
    }
    ctx->led_state = LED_IDLE;

    SaveQStats(ctx);

    ctx->initial_historic_accumulated_loss = 0;
    //CanLog(ctx, "QD = %d\n", (uint32_t)(ctx->qstats.historic_accumulated_loss * 1000));

    HAL_Delay(200);
    return status;
}

/**
 * Handle active logic: poll voltages and temps, handle faults, etc.
 */
void DbmsHandleActive(DbmsCtx* ctx)
{
    if (!ctx->flags.active) return;
    ctx->profiling.times.T0 = GetUs(ctx);

    StackUpdateAllVoltReadings(ctx);
    HAL_Delay(10);
    ctx->profiling.times.T1 = GetUs(ctx);

    StackUpdateAllTempReadings(ctx);
    HAL_Delay(10);
    ctx->profiling.times.T2 = GetUs(ctx);
    ctx->profiling.times.T3 = GetUs(ctx);

    if (GetSetting(ctx, IGNORE_BAD_THERMS))
    {
        FillMissingTempReadings(ctx);
    }
        ctx->profiling.times.T4 = GetUs(ctx);

    StackCalcStats(ctx);
        ctx->profiling.times.T5 = GetUs(ctx);

    if (HAL_GetTick() - ctx->timing.wakeup_ts > GetSetting(ctx, MS_BEFORE_FAULT_CHECKS))
    {
        CtrlUpdateFaults(ctx);
        // PollFaultSummary(ctx);
    }

    if (ctx->stats.iters % 2 == 0){
        ctx->stats.n_rx_stack_frames_itvl = 0;
        ctx->stats.n_rx_stack_bad_crcs_itvl = 0;
    }

    ctx->profiling.times.T9 = GetUs(ctx);

    if (ctx->stats.iters % 10 == 0) {
        // send T1 to init one way delay with DCU
        // can transmit queue sets T1 for accurate send time
        uint8_t init_frame[8] = {0};
        CanTransmit(ctx, CANID_TX_DELAY, init_frame);
    }
}


void DbmsIter(DbmsCtx* ctx)
{
    int status = 0;
    ctx->stats.iters++;
    ctx->timing.iter_start_us = GetUs(ctx);
    // ctx->profiling.profiling.times.T0 = GetUs(ctx);

    if (ctx->flags.need_to_save_bad_therms)
    {
        if (SaveStoredObject(ctx, EEPROM_BAD_THERM_MASK_ADDR, (uint8_t*) ctx->bad_therm_mask, sizeof(ctx->bad_therm_mask)) != 0)
        {
            CAN_REPORT_FAULT(ctx, status);
        }
        ctx->flags.need_to_save_bad_therms = false;
    }

    /**
     * Handle blackbox data requested
     */
    if (ctx->blackbox.requested)
    {
        CanLog(ctx, "sending\n");
        if ((status = BlackboxSend(ctx)) != HAL_OK)
        {
            CAN_REPORT_FAULT(ctx, status);
        }

        ctx->blackbox.requested = false;
        ctx->blackbox.ready = false;
    }
    // ctx->profiling.profiling.times.T1 = GetUs(ctx);

    HAL_Delay(6);
    // ctx->profiling.profiling.times.T2 = GetUs(ctx);
    // Store the settings when required
    if (ctx->flags.need_to_sync_settings)
    {
        // should we also check that we are awake?
        // should we allow the user to configure
        // settings while we are "shutdown"
        // even though the controller should be on
        if ((status = SaveSettings(ctx)) != HAL_OK)
        {
            CAN_REPORT_FAULT(ctx, status);
            ctx->led_state = LED_FIRMWARE_FAULT;
        }
        ctx->flags.need_to_sync_settings = false;
    }
    // ctx->profiling.profiling.times.T3 = GetUs(ctx);

    // Store the Q0 value when required
    if (ctx->flags.need_to_reset_qstats)
    {
        if ((status = SaveQStats(ctx)) != HAL_OK)
        {
            CAN_REPORT_FAULT(ctx, status);
            ctx->led_state = LED_FIRMWARE_FAULT;
        }
        ctx->flags.need_to_reset_qstats = false;
    }

    if (ctx->stats.iters % 400 == 0)
    {
        PeriodicSaveQStats(ctx);
    }

    // Let everybody know that we are alive
    CanTxHeartbeat(ctx, CalcCrc16((uint8_t*)ctx->settings, sizeof(DbmsSettings)));
    // ctx->profiling.profiling.times.T4 = GetUs(ctx);
    /**
     * Active/shutdown switch based on main heartbeat
     * If it's been too long since we have recived a frame, we need to force a shutdown
     * otherwise we want to be active
     */
    // uint64_t cur_time = HAL_GetTick();

    // uint32_t quiet_ms = GetSetting(ctx, QUIET_MS_BEFORE_SHUTDOWN);

    // Handle GPIO-triggered shutdown immediately
    if (ctx->flags.shutdown_requested)
    {
        if (ctx->flags.active)
        {
            DbmsPerformShutdown(ctx, ctx->flags.shutdown_stack_requested);
            ctx->flags.active = false;
            uint64_t duration = GetUs(ctx) - ctx->stats.shutdown_start_us;
            CanLog(ctx, "Shutdown duration: %d us\n", (uint32_t)duration);
        }
        // keep shutdown_requested = true to prevent waking back up
        SetFaultLine(ctx, true);
        ctx->led_state = LED_IDLE;
    }
    else
    {
        if (!ctx->flags.active)
        {
            ctx->led_state = LED_INIT;
            ProcessLedAction(ctx);
            DbmsPerformWakeup(ctx);
        }
        if (CtrlHasAnyHardFaults(ctx))
            ctx->led_state = LED_ACTIVE_FAULT;
        else if (CtrlHasAnyWarnings(ctx))
            ctx->led_state = LED_ACTIVE_WARNING;
        else
            ctx->led_state = LED_ACTIVE;
        ctx->flags.active = true;
        DbmsHandleActive(ctx);
    }

    // TODO: unlatches CAN fail fault
    // if (CtrlHasFault(ctx, CTRL_FAULT_CAN_FAIL) && (GetUs(ctx) - ctx->stats.last_can_tx_ts < GetSetting(ctx, MS_BEFORE_CAN_FAIL)))
    // {
    //     CtrlClearFault(ctx, CTRL_FAULT_CAN_FAIL);
    // }

    // ctx->profiling.profiling.times.T5 = GetUs(ctx);

    ChargingUpdate(ctx);

    // Update the state of charge model
    UpdateModel(ctx);   // TODO: add condition for when we update this

    // Blackbox handler
    BlackboxUpdate(ctx);

    // Precharge handler
    PrechargeUpdate(ctx);

    // pin thath as an interrupt, then throw shutdown signal
    /**
     * Save faults and blackbox data to eeprom
     */
    if (ctx->flags.need_to_save_faults)
    {
        if ((status = SaveFaultState(ctx)) != HAL_OK)
        {
            CAN_REPORT_FAULT(ctx, status);
        }

        ctx->flags.need_to_save_faults = false;
    }
    if (ctx->flags.need_to_save_blackbox)
    {
        CanLog(ctx, "SAVRING BB!\n");
        if ((status = BlackboxSaveOnFault(ctx)) != HAL_OK)
        {
            CAN_REPORT_FAULT(ctx, status);
        }
        ctx->flags.need_to_save_blackbox = false;
    }
    // ctx->profiling.profiling.times.T7 = GetUs(ctx);

    /**
     * Transmit telemetry
     */
    SendPlexMetrics(ctx);
    if (ctx->stats.iters % 20)  // ~every 1s
    {
        SendCellTemps(ctx);
        // here transmit values of settings in can frame
    }
    ctx->flags.telem_enable = HAL_GetTick() - ctx->timing.last_rx_telembeat < 5000; // < GetSetting(ctx, QUIET_MS_BEFORE_SHUTDOWN))
    if (ctx->flags.telem_enable)
    {
        SendMetrics(ctx);               // TODO: resolve conflicting metrics

        // send a blackbox ready frame UNTIL the app requests. flag is set true when saved, false when app requests
        if(ctx->blackbox.ready)
        {
            uint8_t ready_frame[8] = {0};
            CanTransmit(ctx, CANID_TX_BLACKBOX_READY, ready_frame);
        }

        SendCellVoltages(ctx);
        SendCellTemps(ctx);
        SendFaultData(ctx);
    }
    // ctx->profiling.profiling.times.T8 = GetUs(ctx);
    /**
     * Handle LED states and such
     */
    #ifdef HAS_FAN
    UpdateFan(ctx);
    #endif

    if (ctx->stats.max_t > GetSetting(ctx, FAN_ON_TEMP))
    {
        ctx->flags.fan_on = true;
    }
    else if (ctx->stats.max_t < GetSetting(ctx, FAN_OFF_TEMP))
    {
        ctx->flags.fan_on = false;
    }

    ProcessLedAction(ctx);

    if (ctx->flags.active) {
        MonitorLedBlink(ctx);
    }

    // float oa1, oa2, ob1, ob2 = 0;
    // ReadMuxOutputs4x1(ctx, 1, &oa1, &oa2, &ob1, &ob2);
    // CanLog(ctx, "Mux test: %d %d %d %d\n", (int)oa1, (int)oa2, (int)ob1, (int)ob2);
    /**
     * Schedule the next loop
     */
    ctx->timing.iter_end_us = GetUs(ctx);
    ctx->stats.looptime = ctx->timing.iter_end_us - ctx->timing.iter_start_us;
    ctx->stats.end_delay = CalcIterDelay(ctx, ITER_TARGET_HZ);
    // ctx->profiling.profiling.times.T9 = GetUs(ctx);
    SetMuxChannels(ctx, ctx->stats.iters % 4);
    // HAL_Delay(1);
    HAL_Delay(ctx->stats.end_delay / 1000);
    DelayUs(ctx, ctx->stats.end_delay % 1000);
}

void DbmsCanRx(DbmsCtx* ctx, CanRxChannel channel, CAN_RxHeaderTypeDef rx_header, uint8_t rx_data[8])
{
    int status = 0;
    uint32_t can_id = (rx_header.IDE == CAN_ID_EXT) ? rx_header.ExtId : rx_header.StdId;
    ctx->stats.n_rx_can_frames++;
    uint64_t rx_timestamp = GET_US2();

    switch (can_id)
    {
    case CANID_RX_OLD_HEARTBEAT:
    case CANID_RX_HEARTBEAT:

        ctx->timing.last_rx_heartbeat = HAL_GetTick();

        uint64_t remote_ts = be64_to_u64(rx_data);
        SyncRealTime(ctx, remote_ts);

        break;
    case CANID_RX_SHUTDOWN_STACK:
        ctx->flags.shutdown_requested = true;
        ctx->flags.shutdown_stack_requested = true;
        break;
    case CANID_RX_CLEAR_SHUTDOWN:
        ctx->flags.shutdown_requested = false;
        ctx->flags.shutdown_stack_requested = false;
        break;
    case CANID_RX_TELEMBEAT:
        ctx->timing.last_rx_telembeat = HAL_GetTick();
        break;
    case CANID_RX_SET_CONFIG:
        if ((status = HandleCanConfig(ctx, rx_data, CFG_SET)) != 0)
        {
            if (status == ERR_CFGID_NOT_FOUND)
            {
            }
        }
        break;
    case CANID_RX_GET_CONFIG:
        if ((status = HandleCanConfig(ctx, rx_data, CFG_GET)) != 0)
        {
            if (status == ERR_CFGID_NOT_FOUND)
            {
            }
        }
        break;
    case CANID_ISENSE_CURRENT:
        ctx->current_sensor.current_ma = (int32_t)UnpackCurrentSensorData(rx_data);
        break;
    case CANID_ISENSE_VOLTAGE1:
        ctx->stats.last_ivt_rx_ts = GetUs(ctx);
        ctx->current_sensor.voltage1_mv = (int32_t)UnpackCurrentSensorData(rx_data);
        break;
    case CANID_ISENSE_POWER:
        ctx->current_sensor.power_w = (int32_t)UnpackCurrentSensorData(rx_data);
        break;
    case CANID_ISENSE_CHARGE:
        ctx->current_sensor.charge_as = (int32_t)UnpackCurrentSensorData(rx_data);
        if (!ctx->current_sensor.has_q_offset) {
            ctx->current_sensor.q_offset = ctx->current_sensor.charge_as;
            ctx->current_sensor.has_q_offset = true;
            CanLog(ctx, "q_offset %d\nmAs", (int)(ctx->current_sensor.q_offset * 1000));
        }
        // CanLog(ctx, "Got charge measurement\n");
        break;
    case CANID_ISENSE_ENERGY:
        ctx->current_sensor.energy_wh = (int32_t)UnpackCurrentSensorData(rx_data);
        break;
    case CANID_RX_CLEAR_FAULTS:
        ctx->flags.req_fault_clear = true;
        break;
    case CANID_RX_FAULTS_CONFIG:
        CtrlSetFaultConfig(ctx, be32_to_u32(rx_data + 0), be32_to_u32(rx_data + 4));
        break;
    case CANID_RX_SET_INITIAL_CHARGE:
        ctx->qstats.initial = be32_to_u32(rx_data) / 1e6f;
        CanLog(ctx, "Q0: %d\n", be32_to_u32(rx_data));
        ctx->qstats.accumulated_loss = 0;
        ctx->qstats.historic_accumulated_loss = 0;
        ctx->initial_historic_accumulated_loss = 0;
        // ctx->qstats.initial_set_ts = (int32_t)(GetRealTime(ctx) / 1000);
        ctx->qstats.initial_set_ts = 0;
        ctx->flags.need_to_reset_qstats = true;
        break;

    case CANID_RX_BLACKBOX_REQUEST:
        ctx->blackbox.requested = true;
        break;
    case CANID_RX_DELAY:
        ctx->delay.T2 = be32_to_u32(rx_data + 0);
        ctx->delay.T3 = be32_to_u32(rx_data + 4);
        ctx->delay.T4 = rx_timestamp;

        if (ctx->delay.T2 > ctx->delay.T3 || ctx->delay.T1 > ctx->delay.T4)
        {
            break;
        }

        ctx->delay.one_way_delay = ((ctx->delay.T4 - ctx->delay.T1) - (ctx->delay.T3 - ctx->delay.T2)) / 2;
        ctx->delay.clock_offset = ((ctx->delay.T2 - ctx->delay.T1) + (ctx->delay.T3 - ctx->delay.T4)) / 2;

        int64_t sample = (int64_t) ctx->delay.one_way_delay;
        int64_t err = sample - (int64_t) ctx->delay.mu;
        ctx->delay.mu = (uint64_t) ((int64_t) ctx->delay.mu + (int64_t) (ALPHA * err));
        int64_t abs_err = err < 0 ? -err : err;
        ctx->delay.st_dev = (uint64_t) ((1 - BETA) * ctx->delay.st_dev + BETA * abs_err);

        break;

    case CANID_RX_DO_BAL_LOOP:
        ctx->charging.bal_loop_hb = HAL_GetTick();
        break;

    case CANID_RX_SET_BAD_THERM:;
        uint8_t bt_side = rx_data[0] >> 4;
        uint8_t bt_therm = rx_data[0] & 0x0F;
        if (rx_data[1])
            ctx->bad_therm_mask[bt_side] |= (1 << bt_therm);
        else
            ctx->bad_therm_mask[bt_side] &= ~(1 << bt_therm);
        ctx->flags.need_to_save_bad_therms = true;
        break;

// TODO: remove this
#ifdef DEBUG_DO_OVERWRITE_TEMPS_OVER_CAN
    case CANID_DEBUG_OVERWRITE_TEMPS:
        uint32_t temp_milli_deg_C = be32_to_u32(rx_data);
        float temp_deg_C = temp_milli_deg_C / 1000.0;
        for (uint8_t i = 0; i < N_SIDES; i++)
        {
            for (uint8_t j = 0; j < N_TEMPS_PER_SIDE; j++)
                ctx->cell_states[i].temps[j] = temp_deg_C;
        }
        break;
#endif

    case CANID_ELCON_RX:
        HandleElconHeartbeat(ctx, rx_data);
        break;
    case CANID_CHARGING_HB:
        CanLog(ctx, "Charging HB\n");
        ctx->charging.heartbeat = HAL_GetTick();
        break;
    // case 0x181:
    //     CanLog(ctx, "got it\n");
    //     break;
    // case 0x061:
    //     CanLog(ctx, "this too\n");
    //     break;
    default:
        ctx->stats.n_unmatched_can_frames++;
        break;
    }
}

//
//  Handle fatal HAL error
//
void DbmsErr(DbmsCtx* ctx)
{
    ctx->led_state = LED_FIRMWARE_FAULT;
    DbmsPerformShutdown(ctx, true);
}

void DbmsClose(DbmsCtx* ctx)
{
    ctx->led_state = LED_IDLE;
}
