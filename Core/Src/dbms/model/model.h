/** 
 * 
 * Distributed BMS      Equivilant Circuit Model
 *
 * Copyright (C) 2025   Texas A&M University
 * 
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#ifndef _MODEL_H_
#define _MODEL_H_

#include "../context.h"
#include "data.h"
#include "../utils/lut.h"
#include "../storage.h"
#include "../vinterface.h"

// 2026 cell (4.9 Ah) — flat bounds until temperature characterization is available
#define Q_BOUND_L_OC    4.9
#define Q_BOUND_H_OC    4.9
#define Q_BOUND_L_RC    4.9
#define Q_BOUND_H_RC    4.9
#define MIN_OC_I        4.5     // 1 coulomb

extern const float TEMPS[N_TEMPS];

void UpdateModel(DbmsCtx* ctx);
int LoadQStats(DbmsCtx* ctx);
int SaveQStats(DbmsCtx* ctx);
int PeriodicSaveQStats(DbmsCtx *ctx);

float F_Q_top(DbmsCtx* ctx, float ocv, float T_bar);

#endif