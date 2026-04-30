/**
 *
 * Distributed BMS      Elcon Interface
 *
 * Copyright (C) 2025   Texas A&M University
 *
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#include "elcon.h"

void HandleElconHeartbeat(DbmsCtx* ctx, uint8_t* data)
{
    ctx->elcon.heartbeat = HAL_GetTick();
    // CanLog(ctx, "t %d\n", ctx->elcon.heartbeat);

    ctx->elcon.voltage_out = be16_to_u16(data) / 10;
    ctx->elcon.current_out = be16_to_u16(data+2) / 10;

    // CanLog(ctx, "%d\n", (int) ctx->elcon.voltage_out);
    // CanLog(ctx, "%d\n", (int) ctx->elcon.current_out);

    ctx->elcon.status_flags = data[5];
}

void SendElconRequest(DbmsCtx* ctx, int16_t v_req, int16_t i_req, bool en)
{
    uint8_t frame[8] = {0};
    ctx->elcon.v_req = v_req;
    ctx->elcon.i_req = i_req;
    v_req *= 10;
    i_req *= 10;
    frame[0] = (v_req >> 8) & 0xFF;
    frame[1] = v_req & 0xFF;
    frame[2] = (i_req >> 8) & 0xFF;
    frame[3] = i_req & 0xFF;
    frame[4] = en;
    int err = CanChargeTransmit(ctx, CANID_ELCON_TX, frame);
}