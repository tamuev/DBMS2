/**
 *
 * Distributed BMS      CAN I/O Interface
 *
 * Copyright (C) 2025   Texas A&M University
 *
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#ifndef _CAN_H_
#define _CAN_H_

#include "utils/common.h"
#include "context.h"

#include "ledctl.h"
#include "sched.h"
#include "settings.h"

#define CAN_STD_ID_MASK         0x7FF
#define CAN_EXT_ID_MASK         0x1FFFFFFF
#define CAN_LOG_BUFFER_SIZE     512             // max formatted string length
#define CAN_TX_WAIT_US          200             // polling sleep step
#define CAN_TX_TIMEOUT_US       3000

#define ERR_CFGID_NOT_FOUND     54


#define CANID_ISENSE_COMMAND            0x411
#define CANID_ISENSE_CURRENT            0x521
#define CANID_ISENSE_VOLTAGE1           0x522
#define CANID_ISENSE_VOLTAGE2           0x523
#define CANID_ISENSE_VOLTAGE3           0x524
#define CANID_ISENSE_POWER              0x526
#define CANID_ISENSE_CHARGE             0x527
#define CANID_ISENSE_ENERGY             0x528

/* CAN Band 1 */

#define CANID_TX_HEARTBEAT          0x0B1
#define CANID_RX_HEARTBEAT          0x0B2
#define CANID_RX_OLD_HEARTBEAT      0x541 // TODO: Remove when LV updates this

#define CANID_TX_GET_CONFIG         0x0B3
#define CANID_TX_CFG_ACK            0x0B4
#define CANID_RX_SET_CONFIG         0x0B5
#define CANID_RX_GET_CONFIG         0x0B6

#define CANID_RX_CLEAR_FAULTS       0x0B7
#define CANID_RX_SET_INITIAL_CHARGE 0x0B8
#define CANID_CHARGING_HB           0x0B9

#define CANID_RX_TELEMBEAT          0x0BF

#define CANID_RX_SHUTDOWN_STACK     0x0BA
#define CANID_RX_CLEAR_SHUTDOWN     0x0BB

#define CANID_RX_DO_BAL_LOOP        0x0BE

#define CANID_RX_BOOTLOADER         0x0B00B007 // booboot

#define CANID_RX_BLACKBOX_REQUEST   0x0B006000
#define CANID_TX_BLACKBOX           0x0B007000
#define CANID_TX_BLACKBOX_SIZE      0x0B008000
#define CANID_TX_BLACKBOX_READY     0x0B009000

#define CANID_RX_DELAY              0x0BC
#define CANID_TX_DELAY              0x0BD

#define CANID_RX_SET_BAD_THERM      0x0B0

#define CANID_RX_PLEX_DIST          0x625
#define CANID_TX_DISTANCE           0x626
#define CANID_RX_SET_DISTANCE       0x627

/* Extended IDs */

#define CANID_CELLSTATE_VOLTS       0x0B002000
#define CANID_CELLSTATE_TEMPS       0x0B003000
#define CANID_CELLSTATE_BALANCE     0x0B005000
#define CANID_METRIC                0x0B001000
#define CANID_CONSOLE_C0            0x0B004000
#define CANID_TX_FAULTS_MASKS1      0x0B00F010
#define CANID_TX_FAULTS_MASKS2      0x0B00F011
#define CANID_TX_FAULTS_MASKS3      0x0B00F012
#define CANID_RX_FAULTS_CONFIG      0x0B00F013
#define CANID_TX_FAULTS_DATA        0x0B00F020

/* Undefined yet */

#define CANID_FATAL_ERROR           0x50B

#define CANID_DEBUG_OVERWRITE_TEMPS     0x581

#define CANID_ELCON_TX                  0x1806E5F4
#define CANID_ELCON_RX                  0x18FF50E5

typedef enum
{
    CAN_RX_0,
    CAN_RX_1,
} CanRxChannel;

typedef enum
{
    CFG_SET,
    CFG_GET
} CanConfigAction;

int ConfigElconCan(DbmsCtx* ctx);

int ConfigCan(DbmsCtx* ctx);

int CanTransmit(DbmsCtx* ctx, uint32_t id, uint8_t data[8]);

int CanChargeTransmit(DbmsCtx* ctx, uint32_t id, uint8_t data[8]);

#endif
