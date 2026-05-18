/**
 *
 * Distributed BMS      CAN-Based Vehicle Interface
 *
 * Copyright (C) 2025   Texas A&M University
 *
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#include "vinterface.h"

/*****************************
 *      HEARTBEAT
 *****************************/

int CanTxHeartbeat(DbmsCtx* ctx, uint16_t settings_crc)
{
    // TODO: rethink this
    static uint8_t frame[] = {N_SEGMENTS, N_SIDES_PER_SEG, N_GROUPS_PER_SIDE, N_TEMPS_PER_SIDE, 0, 0, 0, 0};
    frame[6] = (settings_crc & 0xff00) >> 8;
    frame[7] = (settings_crc & 0xff);
    return CanTransmit(ctx, CANID_TX_HEARTBEAT, frame);
}

/*****************************
 *      CONFIG
 *****************************/

int HandleCanConfig(DbmsCtx* ctx, uint8_t* rx_data, CanConfigAction action)
{
    // This is an inefficient frame format ):
    uint8_t cfg_id = rx_data[0];
    int32_t cfg_set = 0, cfg_get = 0;

    cfg_set |= (rx_data[4] << 3 * 8);
    cfg_set |= (rx_data[5] << 2 * 8);
    cfg_set |= (rx_data[6] << 1 * 8);
    cfg_set |= (rx_data[7] << 0 * 8);

#ifdef ACK_CFG
    if (action == CFG_SET)
        CanTransmit(ctx, CANID_TX_CFG_ACK, rx_data);
#endif

    if (!IsValidSettingIndex(ctx, cfg_id)) return ERR_CFGID_NOT_FOUND;

    if (action == CFG_SET)
    {
        CanLog(ctx, "CFG %d %d\n", cfg_id, cfg_set);
        SetSetting(ctx, cfg_id, cfg_set);
        ctx->flags.need_to_sync_settings = true;
    }
    else
    {
        cfg_get = GetSetting(ctx, cfg_id);
        uint8_t frame[] = {cfg_id, 0, 0, 0, 0, 0, 0, 0};
        frame[4] = ((cfg_get >> 3 * 8) & 0xFF);
        frame[5] = ((cfg_get >> 2 * 8) & 0xFF);
        frame[6] = ((cfg_get >> 1 * 8) & 0xFF);
        frame[7] = ((cfg_get >> 0 * 8) & 0xFF);
        CanTransmit(ctx, CANID_TX_GET_CONFIG, frame);
    }

    return 0;
}

/*****************************
 *      METRICS
 *****************************/

int SendMetric(DbmsCtx* ctx, uint16_t id, uint64_t value)
{
    static uint8_t data[8] = {0};
    data[0] = (value >> 56) & 0xFF;
    data[1] = (value >> 48) & 0xFF;
    data[2] = (value >> 40) & 0xFF;
    data[3] = (value >> 32) & 0xFF;
    data[4] = (value >> 24) & 0xFF;
    data[5] = (value >> 16) & 0xFF;
    data[6] = (value >> 8)  & 0xFF;
    data[7] = (value >> 0)  & 0xFF;
    return CanTransmit(ctx, (CANID_METRIC & 0xFFFFF000) | (id & 0xFFF), data);
}

#define F2I_K(F, K) ((int)((F) * (K)))

int SendMetrics(DbmsCtx* ctx)
{
    SendMetric(ctx, 0, ctx->stats.iters);

    SendMetric(ctx, 1, ctx->stats.n_tx_can_frames);
    SendMetric(ctx, 2, ctx->stats.n_rx_can_frames);
    SendMetric(ctx, 3, ctx->stats.n_unmatched_can_frames);

    SendMetric(ctx, 4, ctx->current_sensor.current_ma);
    SendMetric(ctx, 5, ctx->current_sensor.voltage1_mv);

    // SendMetric(ctx, 6, 0);
    // SendMetric(ctx, 7, 0);
    SendMetric(ctx, 8, ctx->current_sensor.power_w);
    SendMetric(ctx, 9, ctx->current_sensor.charge_as);
    SendMetric(ctx, 10, ctx->current_sensor.energy_wh);

    SendMetric(ctx, 11, ctx->stats.n_tx_can_fail);
    SendMetric(ctx, 12, ctx->stats.n_tx_can_drop_queue_full);

    SendMetric(ctx, 13, ctx->stats.looptime);
    SendMetric(ctx, 14, ctx->stats.end_delay);

    SendMetric(ctx, 15, ctx->faults.active_faults);

    SendMetric(ctx, 16, ctx->stats.n_rx_stack_frames);
    SendMetric(ctx, 17, ctx->stats.n_rx_stack_bad_crcs);

    SendMetric(ctx, 18, ctx->stats.n_eeprom_writes);
    SendMetric(ctx, 19, ctx->faults_crc);

    SendMetric(ctx, 20, F2I_K(ctx->qstats.initial, 1e6));
    SendMetric(ctx, 21, F2I_K(ctx->qstats.accumulated_loss, 1e6));
    SendMetric(ctx, 22, F2I_K(ctx->model.Q, 1e6));
    SendMetric(ctx, 23, F2I_K(ctx->model.z_oc, 1e6));
    SendMetric(ctx, 24, F2I_K(ctx->model.V_oc, 1e6));
    SendMetric(ctx, 25, F2I_K(ctx->model.R_oc, 1e6));
    SendMetric(ctx, 26, F2I_K(ctx->model.z_rc, 1e6));
    SendMetric(ctx, 27, F2I_K(ctx->model.R_rc, 1e6));
    SendMetric(ctx, 28, 0);
    SendMetric(ctx, 29, 0);
    SendMetric(ctx, 30, F2I_K(ctx->model.R_rc, 1e6));
    SendMetric(ctx, 31, F2I_K(ctx->model.R_cell, 1e6));
    SendMetric(ctx, 32, F2I_K(ctx->model.I_lim, 1e6));

    SendMetric(ctx, 33, ctx->qstats.initial_set_ts);
    SendMetric(ctx, 34, (uint32_t)(GetRealTime(ctx) / 1000));   // conv to S

    SendMetric(ctx, 35, F2I_K(ctx->stats.max_t, 1e3));
    SendMetric(ctx, 36, F2I_K(ctx->stats.min_t, 1e3));
    SendMetric(ctx, 37, F2I_K(ctx->stats.avg_t, 1e3));
    SendMetric(ctx, 38, F2I_K(ctx->stats.max_v, 1e4));
    SendMetric(ctx, 39, F2I_K(ctx->stats.min_v, 1e4));
    SendMetric(ctx, 40, F2I_K(ctx->stats.avg_v, 1e4));

    SendMetric(ctx, 41, F2I_K(ctx->qstats.historic_accumulated_loss, 1e6));

    SendMetric(ctx, 42, ctx->timing.pl_pulse_t);
    SendMetric(ctx, 43, ctx->timing.pl_last_ok_ts);
    SendMetric(ctx, 44, ctx->max_current_ma);

    SendMetric(ctx, 45, ctx->elcon.heartbeat);
    SendMetric(ctx, 46, ctx->elcon.voltage_out);
    SendMetric(ctx, 47, ctx->elcon.current_out);
    SendMetric(ctx, 48, ctx->elcon.status_flags);
    SendMetric(ctx, 49, ctx->stats.fault_line_faulted);

    SendMetric(ctx, 50, ctx->j1772.cp_duty_cycle);
    SendMetric(ctx, 51, ctx->j1772.pp_connect);
    SendMetric(ctx, 52, ctx->j1772.charge_en_req);
    SendMetric(ctx, 53, ctx->j1772.pp_raw_voltage);
    SendMetric(ctx, 54, ctx->charging.conn);
    SendMetric(ctx, 55, ctx->j1772.maxCurrentSupply);
    SendMetric(ctx, 56, ctx->charging.state);

    SendMetric(ctx, 57, ctx->elcon.v_req);
    SendMetric(ctx, 58, ctx->elcon.i_req);

    SendMetric(ctx, 59, F2I_K(ctx->stats.max_v - ctx->stats.min_v, 1e4));
    SendMetric(ctx, 60, ctx->profiling.times.T1 - ctx->profiling.times.T0);
    SendMetric(ctx, 61, ctx->profiling.times.T2 - ctx->profiling.times.T1);
    SendMetric(ctx, 62, ctx->profiling.times.T3 - ctx->profiling.times.T2);
    SendMetric(ctx, 63, ctx->profiling.times.T4 - ctx->profiling.times.T3);
    SendMetric(ctx, 64, ctx->profiling.times.T5 - ctx->profiling.times.T4);
    SendMetric(ctx, 65, ctx->profiling.times.T6 - ctx->profiling.times.T5);
    SendMetric(ctx, 66, ctx->profiling.times.T7 - ctx->profiling.times.T6);
    SendMetric(ctx, 67, ctx->profiling.times.T8 - ctx->profiling.times.T7);
    SendMetric(ctx, 68, ctx->profiling.times.T9 - ctx->profiling.times.T8);

    SendMetric(ctx, 69, F2I_K(ctx->stats.pack_v, 1e4));
    SendMetric(ctx, 70, ctx->precharged);
    if (ctx->stats.n_rx_stack_frames_itvl != 0)
        SendMetric(ctx, 71, (ctx->stats.n_rx_stack_bad_crcs_itvl * 100) / ctx->stats.n_rx_stack_frames_itvl);
    SendMetric(ctx, 72, ctx->stats.n_int_shutdowns);

    for (int i = 0; i < N_MONITORS; i++)
    {
        SendMetric(ctx, 80 + i, ctx->stats.monitor_bad_crcs[i]);
    }

    SendMetric(ctx, 90, ctx->delay.T1);
    SendMetric(ctx, 91, ctx->delay.T2);
    SendMetric(ctx, 92, ctx->delay.T3);
    SendMetric(ctx, 93, ctx->delay.T4);

    SendMetric(ctx, 94, ctx->delay.one_way_delay);
    SendMetric(ctx, 95, ctx->delay.mu);
    SendMetric(ctx, 96, ctx->delay.st_dev);

    SendMetric(ctx, 97, ctx->stats.n_can_bussoffs);

    for (int i = 0; i < N_SIDES; i++)
    {
        SendMetric(ctx, 98 + i, ctx->bad_therm_mask[i]);
    }

    SendMetric(ctx, 100, ctx->stack_dir);
    SendMetric(ctx, 101, ctx->n_fwd_monitors);
    SendMetric(ctx, 102, ctx->n_rev_monitors);
    SendMetric(ctx, 103, ctx->stack_readdressed);

    return 0;
}

void SendPlex32(DbmsCtx* ctx, uint32_t id, uint32_t m)
{
    uint8_t data[8] = {0};
    data[0] = (m >> 3*8) & 0xff;
    data[1] = (m >> 2*8) & 0xff;
    data[2] = (m >> 1*8) & 0xff;
    data[3] = (m >> 0*8) & 0xff;
    CanTransmit(ctx, id, data);
}

void SendPlex32x2(DbmsCtx* ctx, uint32_t id, uint32_t v1, uint32_t v2)
{
    uint8_t data[8] = {0};

    data[0] = (v1 >> 3*8) & 0xff;
    data[1] = (v1 >> 2*8) & 0xff;
    data[2] = (v1 >> 1*8) & 0xff;
    data[3] = (v1 >> 0*8) & 0xff;

    data[4] = (v2 >> 3*8) & 0xff;
    data[5] = (v2 >> 2*8) & 0xff;
    data[6] = (v2 >> 1*8) & 0xff;
    data[7] = (v2 >> 0*8) & 0xff;

    CanTransmit(ctx, id, data);
}

void SendPlex16x4(DbmsCtx* ctx, uint16_t id, uint16_t v1, uint16_t v2, uint16_t v3, uint16_t v4)
{
    uint8_t data[8] = {0};

    data[0] = (v1 >> 8) & 0xff;
    data[1] = (v1 >> 0) & 0xff;

    data[2] = (v2 >> 8) & 0xff;
    data[3] = (v2 >> 0) & 0xff;

    data[4] = (v3 >> 8) & 0xff;
    data[5] = (v3 >> 0) & 0xff;

    data[6] = (v4 >> 8) & 0xff;
    data[7] = (v4 >> 0) & 0xff;

    CanTransmit(ctx, id, data);
}

void SendPlexMetrics(DbmsCtx* ctx)
{
    SendPlex32x2(ctx, 0x10, F2I_K(CLAMP(ctx->model.z_oc, 0.0, 1.0), 100000), 0);

    int32_t pack_v = ctx->stats.avg_v * (N_SIDES * N_GROUPS_PER_SIDE);

    SendPlex32x2(ctx, 0x11, ctx->faults.active_faults | ctx->faults.latched_faults, ctx->current_sensor.current_ma / 1000);
    // SendPlex32x2(ctx, 0x12, ctx->isense.ima.current_mavg_ma / 1000, ctx->pl_pulse_t);
    SendPlex32x2(ctx, 0x12, pack_v, ctx->stats.iters);
    SendPlex32x2(ctx, 0x13, ctx->stats.avg_t, ctx->current_sensor.charge_as);

#ifndef F2I_K
#define F2I_K(F, K) ((int)((F) * (K)))
#endif

    SendPlex16x4(ctx, 0x14, F2I_K(ctx->stats.max_t, 1), F2I_K(ctx->stats.min_t, 1), 0, 0);

    SendPlex16x4(ctx, 0x15, F2I_K(ctx->stats.max_v, 1000), F2I_K(ctx->stats.min_v, 1000), F2I_K(ctx->stats.avg_v, 1000),
                 F2I_K(pack_v, 10));

    SendPlex32x2(ctx, 0x16, (uint32_t) ctx->delay.one_way_delay, (uint32_t) ctx->delay.mu);
    SendPlex32x2(ctx, 0x17, (uint32_t) ctx->delay.st_dev, 0);

    SendPlex32x2(ctx, 0x18, F2I_K(ctx->model.Q, 1e6), F2I_K(ctx->model.z_rc, 1e6));
    SendPlex32x2(ctx, 0x19, F2I_K(ctx->model.V_oc, 1e6), F2I_K(ctx->model.R_oc, 1e6));
    SendPlex32x2(ctx, 0x1A, F2I_K(ctx->model.R_rc, 1e6), 0);
    SendPlex32x2(ctx, 0x1B, 0, F2I_K(ctx->model.R_rc, 1e6));
    SendPlex32x2(ctx, 0x1C, F2I_K(ctx->model.R_cell, 1e6), F2I_K(ctx->model.I_lim, 1e6));
    SendPlex32x2(ctx, 0x1D, F2I_K(ctx->model.P_lim, 1e3), ctx->current_sensor.current_ma / N_P_GROUP);
    SendPlex16x4(ctx, 0x1E, ctx->stats.n_can_bussoffs, 0, 0, 0);
}


/*****************************
 *      LOGGING
 *****************************/


void CanLog(DbmsCtx* ctx, const char* fmt, ...)
{
    if (!ctx->flags.telem_enable) return;

    char buffer[CAN_LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len < 0) return;

    for (int i = 0; i < len; i += 8)
    {
        int32_t id = CANID_CONSOLE_C0;
        uint8_t chunk[8] = {0}; // zero out to pad shorter chunks
        int chunk_size = (len - i >= 8) ? 8 : (len - i);
        memcpy(chunk, &buffer[i], chunk_size);
        id |= (ctx->stats.n_logging_frames & 0xFFF);
        CanTransmit(ctx, id, chunk);

        ctx->stats.n_logging_frames++;
    }
}


/*****************************
 *      CELL DATA
 *****************************/

#define PAD_BUFFER(SZ, K) (((SZ / K) + 1) * K)
#define CLAMP_U16(x) ((uint16_t)((x) < 0 ? 0 : ((x) > 65535 ? 65535 : (x))))

int SendCellDataBuffer(DbmsCtx* ctx, uint32_t id_base, uint16_t side_n, uint16_t* buffer, size_t len)
{
    int status = 0;
    uint8_t frame[8];
    for (size_t j = 0; j < len; j += 4)
    {
        uint32_t id = id_base | (side_n << 4) | (j / 4);
        memcpy_eswap2(frame, buffer + j, 4 * sizeof(uint16_t));
        if ((status = CanTransmit(ctx, id, frame)) != 0) return status;
    }
    return status;
}

int SendCellVoltages(DbmsCtx* ctx)
{
    uint16_t buffer[PAD_BUFFER(N_GROUPS_PER_SIDE, 3)] = {0};

    for (size_t i = 0; i < N_SIDES; i++)
    {
        for (size_t j = 0; j < N_GROUPS_PER_SIDE; j++)
        {
            buffer[j] = CLAMP_U16((long)lroundf(ctx->cell_states[i].voltages[j] * 10.0f));
        }

        SendCellDataBuffer(ctx, CANID_CELLSTATE_VOLTS, i, buffer, ARRAY_LEN(buffer));
    }
    return 0;   // dont really care if it fails or not
}

int SendCellTemps(DbmsCtx* ctx)
{
    uint16_t buffer[PAD_BUFFER(N_GROUPS_PER_SIDE, 3)] = {0};

    for (size_t i = 0; i < N_SIDES; i++)
    {
        for (size_t j = 0; j < N_GROUPS_PER_SIDE; j++)
        {
            buffer[j] = CLAMP_U16((long)lroundf(ctx->cell_states[i].temps[j] * 1000.0f));
        }
        SendCellDataBuffer(ctx, CANID_CELLSTATE_TEMPS, i, buffer, ARRAY_LEN(buffer));
    }
    return 0;
}

int SendCellsToBalance(DbmsCtx* ctx)
{
    uint16_t buffer[PAD_BUFFER(N_GROUPS_PER_SIDE, 3)] = {0};

    for (size_t i = 0; i < N_SIDES; i++)
    {
        for (size_t j = 0; j < N_GROUPS_PER_SIDE; j++)
        {
            buffer[j] = ctx->cell_states[i].cells_to_balance[j] ? 1 : 0;
        }
        SendCellDataBuffer(ctx, CANID_CELLSTATE_BALANCE, i, buffer, ARRAY_LEN(buffer));
    }
    return 0;
}

void SendFaultData(DbmsCtx* ctx)
{
    SendPlex32x2(ctx, CANID_TX_FAULTS_MASKS1, ctx->faults.active_faults, ctx->faults.latched_faults);
    SendPlex32x2(ctx, CANID_TX_FAULTS_MASKS2, ctx->faults.historic_faults, NONMASKABLE_FAULTS);
    SendPlex32x2(ctx, CANID_TX_FAULTS_MASKS3, ctx->faults.warnings_config, ctx->faults.nonlatching_config);

    FaultData buf[2];

    for (int i = 0; i < CTRL_FAULT_TYPE_COUNT; i += 2) {
        buf[0] = ctx->faults.fault_data[i];
        buf[1] = ctx->faults.fault_data[i + 1];

        // endian swap
        buf[0].value = __builtin_bswap16(buf[0].value);
        buf[1].value = __builtin_bswap16(buf[1].value);

        uint32_t id = CANID_TX_FAULTS_DATA + (i / 2);

        CanTransmit(ctx, id, (uint8_t*) buf);
    }
}