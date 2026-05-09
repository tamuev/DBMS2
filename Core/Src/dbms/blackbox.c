/**
 *
 * Distributed BMS      Blackbox (Snapshot Saving Utility)
 *
 * Copyright (C) 2025   Texas A&M University
 *
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#include "blackbox.h"
#include "context.h"
#include "storage.h"
#include "can.h"

#define BLACKBOX_QUEUE_SIZE 10

static Snapshot snapshot_queue[BLACKBOX_QUEUE_SIZE];

void BlackboxInit(DbmsCtx* ctx)
{
    memset(snapshot_queue, 0, sizeof(snapshot_queue));

    ctx->blackbox.head = 0;
    ctx->blackbox.count = 0;
}

void BlackboxUpdate(DbmsCtx* ctx)
{
    ctx->blackbox.head = (ctx->blackbox.head + 1) % BLACKBOX_QUEUE_SIZE;

    if (ctx->blackbox.count < BLACKBOX_QUEUE_SIZE)
    {
        ctx->blackbox.count++;
    }

    PopulateBlackboxInfo(ctx, &snapshot_queue[ctx->blackbox.head]);
}

void PopulateBlackboxInfo(DbmsCtx* ctx, Snapshot* blackbox)
{
    {
        memset(blackbox, 0, sizeof(Snapshot));

        // Iteration counter
        blackbox->iter = ctx->stats.iters;

        // Current sensor data
        blackbox->current_ma = ctx->current_sensor.current_ma;
        blackbox->voltage_mv = ctx->current_sensor.voltage1_mv;

        // Charge stats
        blackbox->qd = ctx->qstats.accumulated_loss;

        // Current limits and resistance
        blackbox->current_limit_a = ctx->model.I_lim;
        blackbox->dcir = ctx->model.R_cell;
        blackbox->total_resistance = ctx->model.R0 + ctx->model.R1 + ctx->model.R2;

        // Temperature stats
        blackbox->avg_temp = ctx->stats.avg_t;
        blackbox->max_temp = ctx->stats.max_t;
        blackbox->min_temp = ctx->stats.min_t;

        // Cell voltage delta
        blackbox->cell_delta_v = ctx->stats.max_v - ctx->stats.min_v;

        // Voltage stats
        blackbox->high_voltage = ctx->stats.max_v;
        blackbox->low_voltage = ctx->stats.min_v;
        blackbox->avg_voltage = ctx->stats.avg_v;

        // Faults
        blackbox->controller_mask = ctx->faults.active_faults;
        blackbox->bridge_fault_summary = ctx->faults.bridge_fault_summary;
        blackbox->bridge_faults = ctx->faults.bridge_faults;
        memcpy(blackbox->monitor_fault_summary, ctx->faults.monitor_fault_summary, N_MONITORS);
    }
}

int SendSnapshot(DbmsCtx* ctx, uint8_t idx)
{
    int status;
    Snapshot temp_snapshot;

    uint32_t snapshot_addr = EEPROM_BLACKBOX_BASE_ADDR + idx * 100;

    if ((status = LoadStoredObject(ctx, snapshot_addr, &temp_snapshot, sizeof(Snapshot))) != HAL_OK)
    {
        CanLog(ctx, "here %d", idx);
        return status;
    }

    uint8_t* blackbox_ptr = (uint8_t*)&temp_snapshot;

    for (uint16_t i = 0; i < sizeof(Snapshot); i += 6)
    {
        uint8_t frame[8] = {0};

        frame[0] = idx;

        // frame number for this snapshot
        frame[1] = (i / 6) & 0xFF;

        // copy next 6 bytes (or remaining bytes if less than 6)
        uint8_t bytes_to_copy = (i + 6 <= sizeof(Snapshot)) ? 6 : (sizeof(Snapshot) - i);
        for (uint8_t j = 0; j < bytes_to_copy; j++)
        {
            frame[j + 2] = blackbox_ptr[i + j];
        }

        status = CanTransmit(ctx, CANID_TX_BLACKBOX, frame);
        if (status != HAL_OK)
        {
            return status;
        }
        HAL_Delay(10);
    }

    return status;
}

int BlackboxSend(DbmsCtx* ctx)
{
    int status;

    struct {
        uint8_t head;
        uint8_t count;
    } blackbox_meta;


    if ((status = LoadStoredObject(ctx, EEPROM_BLACKBOX_META_ADDR, &blackbox_meta, sizeof(blackbox_meta))) != HAL_OK)
    {
        return status;
    }

    // Send all snapshots in chronological order
    for (uint8_t i = 0; i < blackbox_meta.count; ++i)
    {
        if ((status = SendSnapshot(ctx, i)) != HAL_OK)
        {
            return status;
        }
    }

    // send snapshot size so the app can reconstruct frames
    uint16_t snapshot_size = sizeof(Snapshot);
    uint8_t size_frame[8] = {
        snapshot_size & 0xFF,
        (snapshot_size >> 8) & 0xFF,
        0, 0, 0, 0, 0, 0
    };
    CanTransmit(ctx, CANID_TX_BLACKBOX_SIZE, size_frame);


    return status;
}

int BlackboxSaveOnFault(DbmsCtx* ctx)
{
    int status = 0;

    for (uint8_t i = 0; i < ctx->blackbox.count; ++i)
    {
        // index in q
        uint8_t idx = (ctx->blackbox.head + 1 + i) % BLACKBOX_QUEUE_SIZE;

        uint32_t addr = EEPROM_BLACKBOX_BASE_ADDR + i * 100;

        if ((status = SaveStoredObject(ctx, addr, &snapshot_queue[idx], sizeof(Snapshot))) != HAL_OK)
        {
            return status;
        }
    }

    // save metadata (head, count) when done
    struct { uint8_t head; uint8_t count; } meta = { ctx->blackbox.head, ctx->blackbox.count };
    status = SaveStoredObject(ctx, EEPROM_BLACKBOX_META_ADDR, &meta, sizeof(meta));

    ctx->blackbox.ready = true;
    return status;
}