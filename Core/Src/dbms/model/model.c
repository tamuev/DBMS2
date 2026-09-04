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
    T_bar = CLAMP(T_bar, TEMPS[0], TEMPS[N_TEMPS - 1]);
    return ((T_bar - TEMPS[0]) / (TEMPS[N_TEMPS - 1] - TEMPS[0])) * (Q_h - Q_l) + Q_l;
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
//  Forward LUT (Z -> ?)
//  Interpolates a [N_TEMPS x cols] table by SOC (z) and temperature (T_bar)
//
float LookupFromZ(float z, float T_bar, const float* table, int n, int cols)
{
    T_bar = CLAMP(T_bar, TEMPS[0], TEMPS[N_TEMPS - 1]);

    // SOC interpolation index (table is high-SOC-first)
    float i_true = (float)n - z * (float)n;
    int i_low = (int)floorf(i_true);
    int i_high = (int)ceilf(i_true);
    float i_k = i_true - i_low;

    i_low = CLAMP(i_low, 0, n - 1);
    i_high = CLAMP(i_high, 0, n - 1);

    // Find bracketing temperature indices
    int t_idx = 0;
    for (int i = 0; i < N_TEMPS - 1; i++)
    {
        if (TEMPS[i + 1] <= T_bar) t_idx = i + 1;
    }
    if (t_idx >= N_TEMPS - 1) t_idx = N_TEMPS - 2;
    int t_idx_next = t_idx + 1;
    float T_frac = (T_bar - TEMPS[t_idx]) / (TEMPS[t_idx_next] - TEMPS[t_idx]);

    // Interpolate along SOC at both bracketing temperatures
    float a = table[t_idx * cols + i_low];
    float b = table[t_idx * cols + i_high];
    float val_lo = (b - a) * i_k + a;

    a = table[t_idx_next * cols + i_low];
    b = table[t_idx_next * cols + i_high];
    float val_hi = (b - a) * i_k + a;

    // Interpolate between temperatures
    return (val_hi - val_lo) * T_frac + val_lo;
}

//
//  Binary Search for ReverseLookupOCV
//  Looks for the closest OCV values to our input OCV in O(nlogn)
//
void find_bracket_desc(const float* table, int min, int max, float target, int* idx_ge, int* idx_le)
{
    int lo = min, hi = max; // max is first entry of next temperature range
    while (lo < hi)
    {
        int mid = lo + (hi - lo) / 2;
        if (table[mid] >= target)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid;
        }
    }

    *idx_ge = (lo > min) ? lo - 1 : min; // Can exactly match target
    *idx_le = (lo < max) ? lo : max - 1;
    // NOTE: ge as greater element meaning to the left/lower index because list is descending so greater elements to the
    // le as in lesser element = right bound
}

//
//  Calculate current charge
//  Interpolates both temperature (Celsius) and OCV values to obtain z and then Q_top
//
float ReverseLookupOCV(DbmsCtx* ctx, float ocv, float T_bar, const float* table, int cols)
{
    // === Temperature === //
    T_bar = CLAMP(T_bar, TEMPS[0], TEMPS[N_TEMPS - 1]); // Force temp into acceptable range

    int high_t_idx = 1;

    while (high_t_idx < N_TEMPS - 1 &&
       T_bar > TEMPS[high_t_idx])
    {
        high_t_idx++;
    }

    int low_t_idx = high_t_idx - 1;

    float T_frac = (T_bar - TEMPS[low_t_idx]) /
                   (TEMPS[high_t_idx] -
                    TEMPS[low_t_idx]); // For later interpolation between z (SoC) values of different temperatures

    // === Low Temp OCV === //
    float clamped_ocv = CLAMP(ocv, table[high_t_idx * cols - 1], table[low_t_idx * cols]);

    int low_t_idx_ge;
    int low_t_idx_le;
    find_bracket_desc(table, low_t_idx * cols, high_t_idx * cols, clamped_ocv, &low_t_idx_ge,
                      &low_t_idx_le); // Binary search for closest OCV values

    float low_ocv_ge = table[low_t_idx_ge];
    float low_ocv_le = table[low_t_idx_le];
    float low_z_ge = 1.0f - (SOC_STEP * (low_t_idx_ge % cols)); // Calculate SoC from index, NOTE SoC is descending so 1 - ...
    float low_z_le = 1.0f - (SOC_STEP * (low_t_idx_le % cols));

    float low_z;
    if (low_ocv_ge == low_ocv_le) // Check if perfect match
        low_z = low_z_ge; 
    else
        low_z = low_z_le +
        ((ocv - low_ocv_le) / (low_ocv_ge - low_ocv_le)) *
        SOC_STEP;
    // NOTE: low_ocv_ge is larger element because table is descending even though its index (low_t_idx_ge) is left bound

    // === High Temp OCV === //
    clamped_ocv = CLAMP(ocv, table[(high_t_idx + 1) * cols - 1], table[high_t_idx * cols]);
    int high_t_idx_ge;
    int high_t_idx_le;
    find_bracket_desc(table, high_t_idx * cols, (high_t_idx + 1) * cols, clamped_ocv, &high_t_idx_ge, &high_t_idx_le);

    float high_ocv_ge = table[high_t_idx_ge];
    float high_ocv_le = table[high_t_idx_le];
    float high_z_ge = 1.0f - (SOC_STEP * (high_t_idx_ge % cols));
    float high_z_le = 1.0f - (SOC_STEP * (high_t_idx_le % cols));

    float high_z;
    if (high_ocv_ge == high_ocv_le)
        high_z = high_z_ge;
    else
        high_z = high_z_le +
        ((ocv - high_ocv_le) / (high_ocv_ge - high_ocv_le)) *
        SOC_STEP;

    // === Temperature Interpolation === //
    float final_z = T_frac * (high_z - low_z) + low_z;            // Final SoC
    float q_max = F_Q_max(T_bar, Q_BOUND_H_OC, Q_BOUND_L_OC);
    float Q_cur = final_z * q_max; // Calculate current charge from SoC in Ah
    final_z = CLAMP(final_z, 0.0f, 1.0f); // Clamp just in case

    return Q_cur;
}

//
//  Calc Current Charge
//
float F_Q_top(DbmsCtx* ctx, float ocv, float T_bar)
{
    return ReverseLookupOCV(ctx, ocv, T_bar, (const float*)ocv_table, N_OCV_ENTRIES);
}

//
//  Calc OCV
//
float F_OCV(float z, float T_bar)
{
    return LookupFromZ(z, T_bar, (const float*)ocv_table, N_OCV_ENTRIES - 1, N_OCV_ENTRIES);
}

//
//  Calc DCIR from lookup table
//
float F_DCIR_lookup(float z, float T_bar)
{
    return LookupFromZ(z, T_bar, (const float*)dcir_table, N_DCIR_ENTRIES - 1, N_DCIR_ENTRIES);
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

    // RC/ECM Path (single DCIR lookup)
    float Q_max_rc = F_Q_max(T_bar, Q_BOUND_H_RC, Q_BOUND_L_RC);
    m->z_rc = F_Z(m->Q, Q_max_rc);
    m->R_rc = F_DCIR_lookup(m->z_rc, T_bar);

    m->R_cell = (MAX(m->R_oc, m->R_rc));

    m->I_lim = F_I_Limit(V_min, V_dyn_min, m->R_cell);

    m->P_lim = m->I_lim * V_min;
}

void UpdateModel(DbmsCtx* ctx)
{
    float v_pack = (ctx->current_sensor.voltage1_mv / 1e3f) / (N_SIDES * N_GROUPS_PER_SIDE);
    float current = (ctx->current_sensor.current_ma / 1000.0) / N_P_GROUP;
    ctx->qstats.accumulated_loss =
            (ctx->current_sensor.charge_as - ctx->current_sensor.q_offset) / 3600.0; // convert to Ah

    float total_accumulated_loss = (ctx->initial_historic_accumulated_loss + ctx->qstats.accumulated_loss) / 3.0;

    // CanLog("DY %d\n", GetSetting(ctx, DYNAMIC_V_MIN));
    float V_dyn_min = GetSetting(ctx, DYNAMIC_V_MIN) / 1000.0f;
    // ^ this should probably be asserted as positive
    // temp
    // ctx->qstats.accumulated_loss = 0;S
    // ctx->qstats.initial = 3.9;

    // CanLog(ctx, "Avg T: %d\n",(int)(avg_t * 1000.0));

    ComputeModel(&ctx->model, ctx->stats.avg_t, current, ctx->qstats.initial, total_accumulated_loss, v_pack, V_dyn_min,
                 ctx->stats.min_v);
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
    // CanLog(ctx, "saving hist: %d\n", (int)(ctx->qstats.historic_accumulated_loss * 1000));
    return SaveStoredObject(ctx, EEPROM_INITIAL_CHARGE, &ctx->qstats, sizeof(ctx->qstats));
}

int PeriodicSaveQStats(DbmsCtx* ctx)
{
    CanLog(ctx, "psave\n");
    CanLog(ctx, "hist0: %d hist: %d acc: %d\n", (int)(ctx->initial_historic_accumulated_loss * 1000),
           (int)(ctx->qstats.historic_accumulated_loss * 1000), (int)(ctx->qstats.accumulated_loss * 1000));
    if (ctx->initial_historic_accumulated_loss + ctx->qstats.accumulated_loss - ctx->qstats.historic_accumulated_loss >
        0.1)
    {
        return SaveQStats(ctx);
    }
    return 0;
}