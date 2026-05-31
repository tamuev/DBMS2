/**
 *
 * Distributed BMS      Fan Controller
 *
 * Copyright (C) 2025   Texas A&M University
 *
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#include "fan.h"

int InitFan(DbmsCtx* ctx)
{
    HAL_TIM_PWM_Start(ctx->hw.timer_pwm_1, TIM_CHANNEL_4);
    return 0;
}

int UpdateFan(DbmsCtx* ctx)
{
    int duty = 0;

    if (ctx->stats.avg_t > GetSetting(ctx, FAN_ON_TEMP))
    {
        // duty = GetSetting(ctx, FAN_DUTY);
        duty = 100;
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(ctx->hw.timer_pwm_1);
    uint32_t ccr = 0;

    if (duty <= 0)
    {
        ccr = 0;
    }
    else if (duty >= 100)
    {
        ccr = arr;
    }
    else
    {
        ccr = (uint32_t)((duty * (arr + 1U)) / 100U);
        if (ccr > arr) ccr = arr;
    }

    __HAL_TIM_SET_COMPARE(ctx->hw.timer_pwm_1, TIM_CHANNEL_4, ccr);

    return 0;
}