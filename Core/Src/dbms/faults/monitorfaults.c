/**
 *
 * Distributed BMS      Fault-Handling System
 *
 * Copyright (C) 2025   Texas A&M University
 *
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
*/

#include "monitorfaults.h"

void PollFaultSummary(DbmsCtx* ctx)
{
    static uint8_t raw[512];
    
    uint8_t data_size = 1 * sizeof(uint8_t);
    uint8_t expected_rx_size = RX_FRAME_SIZE(data_size) * N_MONITORS;

    StackRead(ctx, raw, FAULT_SUMMARY_REG, data_size, expected_rx_size);

    for (size_t i = 0; i < N_MONITORS; i++)
    {
        IncStackCrcStats(ctx, true, i);
        // TODO: test without on new battery to see if this is necessary
        uint8_t* data = rx_buffer_v + (i * RX_FRAME_SIZE(data_size));
        for (int j = 0; (data[0] != (FAULT_SUMMARY_REG & 0xFF)) && (j < 1024); j++) { data++; }
        
        RxStackFrameVoltages* clean_frame = (RxStackFrameVoltages*)(data - 3);
        if (clean_frame->crc == CALC_CRC_Rx(clean_frame))
            ctx->monitor_faults[clean_frame->devaddr].summary = clean_frame->data[0];
        else
            IncStackCrcStats(ctx, false, i);
        CanLog(ctx, "fs: %d", ctx->monitor_faults[clean_frame->devaddr].summary);
}