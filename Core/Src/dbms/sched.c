/** 
 * 
 * Distributed BMS      Timing/Scheduling Utlities
 *
 * Copyright (C) 2025   Texas A&M University
 * 
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#include "sched.h"

/**
 * Blocking delay in microseconds using the free-running timer.
 */
void DelayUs(DbmsCtx* ctx, uint32_t us)
{
#ifdef SIM
    usleep(us);
#else
    uint64_t start = GetUs(ctx);
    while ((GetUs(ctx) - start) < (uint64_t)us)
    {
        __NOP(); // optional: gives the pipeline a breather
    }
#endif
}

/**
 * get time in microseconds from the hardware timer (monotonic, 64-bit)
 * Assumes timer tick = 1 us. Works with 16- or 32-bit counters.
 */
uint64_t GetUs(DbmsCtx* ctx)
{
#ifdef SIM
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
#else
    TIM_HandleTypeDef* h = ctx->hw.timer;
    // Persist across calls (one instance per process; if you have multiple timers,
    // consider keying these by 'h' or storing in ctx).
    static uint32_t last_cnt = 0;
    static uint64_t high_us = 0;
    static uint32_t rollover = 0;

    // Read current counter first
    uint32_t now = __HAL_TIM_GET_COUNTER(h);

    // Lazy-initialize rollover on first call
    if (rollover == 0)
    {
        // ARR is inclusive; rollover span is ARR+1 counts
        rollover = __HAL_TIM_GET_AUTORELOAD(h) + 1u;
        last_cnt = now;
    }

    // Detect wrap: counter went from near ARR back to 0
    if (now < last_cnt)
    {
        high_us += (uint64_t)rollover; // add one full period (in microseconds)
    }

    last_cnt = now;
    return high_us + (uint64_t)now;
#endif
}

/**
 * Calculate how many microseconds to wait so each loop iteration lasts ~1/target_hz seconds.
 * Uses iter_start_us and iter_end_us captured around your work.
 * Returns 0 if you’re already late.
 */
uint32_t CalcIterDelay(DbmsCtx* ctx, uint32_t hz)
{
    if (hz == 0) return 0; // avoid div-by-zero; no pacing requested

    // Round to nearest microsecond period: T = round(1e6 / hz)
    const uint64_t period_us = (1000000ull + (hz / 2u)) / hz;

    // Elapsed time this iteration
    const uint64_t elapsed_us = ctx->timing.iter_end_us - ctx->timing.iter_start_us;

    // Remaining time (saturate at 0)
    if (elapsed_us >= period_us) return 0;
    return (uint32_t)(period_us - elapsed_us);
}

/**
 * Real Time Module
 * ----------------
 * Provides MS accurate "real time"
 * based on synchronization signals 
 * from a remote.
 */

// Register a synchronization signal
void SyncRealTime(DbmsCtx* ctx, uint64_t remote_ts)
{
    ctx->realtime.global_ts = remote_ts;
    ctx->realtime.local_ts = HAL_GetTick();
}

// Get the current global time
uint64_t GetRealTime(DbmsCtx* ctx)
{
    uint64_t delta = HAL_GetTick() - ctx->realtime.local_ts;
    return ctx->realtime.global_ts + delta;
}

void InitGetUS2()
{
    // init DWT for microsecond timestamps
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}