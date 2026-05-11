/** 
 * 
 * Distributed BMS      Precharge Control 
 *
 * Copyright (C) 2025   Texas A&M University
 * 
 *                      Justus Languell  <justus@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */

#include "precharge.h"

void PrechargeSet(DbmsCtx* ctx)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, ctx->precharged);
}

void PrechargeUpdate(DbmsCtx* ctx)
{
  int ivt_mv = ctx->current_sensor.voltage1_mv;
  float pack_mv = ctx->stats.pack_v;
  float threshold = 0.8; //GetSetting(ctx, ctx->precharged ? PRECHARGE_OFF_TH : PRECHARGE_ON_TH);
 
  ctx->precharged = pack_mv > 50000 && (ivt_mv > pack_mv * threshold) && !ctx->stats.fault_line_faulted ;
 
  PrechargeSet(ctx);
}