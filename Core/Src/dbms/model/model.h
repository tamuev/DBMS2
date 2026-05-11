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

#define Q_BOUND_L_OC    4.3696
#define Q_BOUND_H_OC    4.3967
#define Q_BOUND_L_RC    4.197
#define Q_BOUND_H_RC    4.3914
#define T_H_PT          45
#define T_L_PT          25
#define MIN_OC_I        4.5     // 1 coulomb

void UpdateModel(DbmsCtx* ctx);

int LoadQStats(DbmsCtx* ctx);
int SaveQStats(DbmsCtx* ctx);
int PeriodicSaveQStats(DbmsCtx *ctx);

#endif