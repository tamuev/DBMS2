/** 
 * 
 * Distributed BMS      Bridge I/O Interface
 *
 * Copyright (C) 2025   Texas A&M University
 * 
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#ifndef _BRIDGE_H_
#define _BRIDGE_H_

#include "utils/common.h"

#include "faults/faults.h"
#include "context.h"
#include "model/data.h"

#include "can.h"
#include "vinterface.h"
#include "stack.h"

// TODO: optimize the shit out of this
#define STACK_SEND_TIMEOUT 6
#define STACK_RECV_TIMEOUT 20

#define APBxCLK 42000000    // TODO: fix legacy name

#define RX_FRAME_SIZE(SIZE) (SIZE+6)

typedef struct _TxStackFrame1Dev TxStackFrame1Dev;
typedef struct _TxStackFrameNDev TxStackFrameNDev;

int SendUARTFrame(DbmsCtx* ctx, uint8_t* buf, size_t len);

/**
 * @brief Sends a data frame to the stack via UART with
 * a CRC appended to the end for verification
 * 
 * @param ctx Context pointer
 * @param buf The data for the frame to send
 * @param len The length of the frame data
 * @return Error code
 */
int SendStackFrame(DbmsCtx* ctx, void* frame, size_t len);

int SendStackFrame1Dev(DbmsCtx* ctx, TxStackFrame1Dev frame);
int SendStackFrameNDev(DbmsCtx* ctx, TxStackFrameNDev frame);


/**
 * @brief Utility function to set the baud rate of the UART
 * Used for shutdown and wakeup
 * 
 * @param brr Some timing property
 */
void SetBrr(uint64_t brr);

/**
 * @brief Send a "blip", where we pull the line low for a little bit.
 * Right now the length of the blip is set by BRR.
 *
 * @todo Make it accept param as us or something more intuitive.
 * 
 * @param ctx Context pointer
 * @param brr Some timing poperty
 * @return Error code
 */
int SendStackBlip(DbmsCtx* ctx, uint64_t brr);

#endif