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
#include "model.h"

float Temp_C_to_K(float T_C)
{
    return T_C + 273.15;
}

float Temp_K_to_C(float T_K)
{
    return T_K - 273.15;
}

//
//  State of charge calc given the starting
//  charge and the accumulated lost charge
//
float F_Q(float Q0, float Qd) // calculates remaining charge
{
    return Q0 - Qd;
}

//  
//  Find the max current of the battery for 
//  OC or RC path based on Q_h and Q_l
//
float F_Q_max(float T_bar, float Q_h, float Q_l)
{
    return ((T_bar - T_L_PT) / (T_H_PT - T_L_PT)) * (Q_h - Q_l) + Q_l;
}

//
//  Calc % SOC based on Q and Q_max for
//  OC and ECM paths
//
float F_Z(float Q, float Q_max)
{
    return CLAMP(Q / Q_max, 0.0f, 1.0f);
}

// 
//  Forward evil LUT (Z -> ?)
//  Where the input is at descrete
//  intervals
//
float LookupFromZ(float z, float T_bar, float* x_t_lows, float* x_t_highs, float n)
{
    T_bar = CLAMP(T_bar, T_L_PT, T_H_PT);

    float i_true = n - z * n;
    int i_low = floorf(i_true);
    int i_high = ceilf(i_true);
    float i_k = i_true - i_low; 

    i_low = CLAMP(i_low, 0, n - 1);
    i_high = CLAMP(i_high, 0, n - 1);

    float a = x_t_lows[i_low];
    float b = x_t_lows[i_high];
    float x_t_low = (b-a) * i_k + a;

    a = x_t_highs[i_low];
    b = x_t_highs[i_high];
    float x_t_high = (b-a) * i_k + a;

    return (x_t_high - x_t_low) * 
            ( (T_bar - T_L_PT) / (T_H_PT - T_L_PT) ) + x_t_low;
}

//
//  Calc OCV 
//
float F_OCV(float z, float T_bar)
{
    return LookupFromZ(z, T_bar, (float*)ocv_t_low, (float*)ocv_t_high, N_OCV_ENTRIES-1);
}

//
//  Calc ECM RC metrics
//
float F_RC_R(float z, float T_bar, int r_n)
{
    r_n = CLAMP(r_n, 0, 3-1);
    return LookupFromZ(z, T_bar, (float*)rc_t_low[r_n], (float*)rc_t_high[r_n], N_RC_ENTRIES-1);
}

//
//  Calc DCIR Resistance
//
float F_DCIR(float V_oc, float V_pack, float I)
{
    // TOOD: clamp I (>0)
    return (V_oc - V_pack) / I;
}

// 
//  Calc I Limit
//
float F_I_Limit(float V_min, float V_dyn_min, float R_cell)
{
    // TOOD: clamp R_pack (>0)
    return ((V_min - V_dyn_min) / R_cell) * N_P_GROUP;
}

void ComputeModel(Model* m, float T_bar, float I, float Q0, float Qd, float V_pack, float V_dyn_min, float V_min)
{
    m->Q = F_Q(Q0, Qd);
    
    // OC Path
    float Q_max_oc = F_Q_max(T_bar, Q_BOUND_H_OC, Q_BOUND_L_OC);
    m->z_oc = F_Z(m->Q, Q_max_oc);
    m->V_oc = F_OCV(m->z_oc, T_bar);
    if (I > MIN_OC_I)
    {
        m->R_oc = F_DCIR(m->V_oc, V_pack, I);
    }
    else m->R_oc = 0;
    
    // RC/ECM Path
    float Q_max_rc = F_Q_max(T_bar, Q_BOUND_H_RC, Q_BOUND_L_RC);
    m->z_rc = F_Z(m->Q, Q_max_rc);
    m->R0 = F_RC_R(m->z_rc, T_bar, 0);
    m->R1 = F_RC_R(m->z_rc, T_bar, 1);
    m->R2 = F_RC_R(m->z_rc, T_bar, 2);

    m->R_rc = ((m->R0 + m->R1 + m->R2));

    m->R_cell = (MAX(m->R_oc, m->R_rc));

    m->I_lim = F_I_Limit(V_min, V_dyn_min, m->R_cell);

    m->P_lim = m->I_lim * V_min;
}


void UpdateModel(DbmsCtx* ctx)
{
  

    float v_pack = (ctx->current_sensor.voltage1_mv / 1e3f) / (N_SIDES * N_GROUPS_PER_SIDE);
    float current = (ctx->current_sensor.current_ma / 1000.0) / N_P_GROUP;
    ctx->qstats.accumulated_loss = (ctx->current_sensor.charge_as - ctx->current_sensor.q_offset) / 3600.0;    // convert to Ah

    float total_accumulated_loss = (ctx->initial_historic_accumulated_loss + ctx->qstats.accumulated_loss) / 3.0;

    // CanLog("DY %d\n", GetSetting(ctx, DYNAMIC_V_MIN));
    float V_dyn_min = GetSetting(ctx, DYNAMIC_V_MIN) / 1000.0f;
    // ^ this should probably be asserted as positive
    //temp 
    // ctx->qstats.accumulated_loss = 0;S
    // ctx->qstats.initial = 3.9;

    // CanLog(ctx, "Avg T: %d\n",(int)(avg_t * 1000.0));

    ComputeModel(&ctx->model, ctx->stats.avg_t, current, ctx->qstats.initial, total_accumulated_loss, v_pack, V_dyn_min, ctx->stats.min_v);
}


int LoadQStats(DbmsCtx* ctx)
{
    int status;
    if ((status = LoadStoredObject(ctx, EEPROM_INITIAL_CHARGE, &ctx->qstats, sizeof(ctx->qstats))) != 0)
    {
        // crc mismatch
    }
    ctx->initial_historic_accumulated_loss = ctx->qstats.historic_accumulated_loss;
    ctx->qstats.accumulated_loss = 0;

    CanLog(ctx, "load hist: %d\n", (int)(ctx->qstats.historic_accumulated_loss * 1000));

    return status;
}

int SaveQStats(DbmsCtx* ctx)
{
    ctx->qstats.historic_accumulated_loss = ctx->initial_historic_accumulated_loss + ctx->qstats.accumulated_loss;
    CanLog(ctx, "saving hist: %d\n", (int)(ctx->qstats.historic_accumulated_loss * 1000));
    return SaveStoredObject(
            ctx, EEPROM_INITIAL_CHARGE, &ctx->qstats, sizeof(ctx->qstats));
}

int PeriodicSaveQStats(DbmsCtx* ctx) {
    CanLog(ctx, "psave\n");
    CanLog(ctx, "hist0: %d hist: %d acc: %d\n", 
        (int) (ctx->initial_historic_accumulated_loss * 1000), 
        (int) (ctx->qstats.historic_accumulated_loss * 1000), 
        (int) (ctx->qstats.accumulated_loss * 1000));
    if (ctx->initial_historic_accumulated_loss + ctx->qstats.accumulated_loss - ctx->qstats.historic_accumulated_loss > 0.1) {
        return SaveQStats(ctx);
    }
    return 0;
}