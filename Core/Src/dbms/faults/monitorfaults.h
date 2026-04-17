#ifndef _MON_FAULTS_H_ 
#define _MON_FAULTS_H_

#include "../utils/common.h"
#include "../context.h"
#include "../settings.h"
#include "../vinterface.h"
#include "../dbms.h"

#define FAULT_SUMMARY_MASK  0x41

struct FaultSummary {
    uint8_t summary;
    uint8_t comm[3];
    uint8_t otp;
    uint8_t sys;
    uint8_t prot1;
    // prot2
    uint8_t pwr[3];
}

#define PROT(summary)   summary & 0x80
#define OTP(summary)    summary & 0x20
#define COMM(sumary)    summary & 0x10
#define OTUT(summary)   summary & 0x08
#define OVUV(summary)   summary & 0x04
#define SYS(summary)    summary & 0x02
#define PWR(summary)    summary & 0x01

void PollFaultSummary(DbmsCtx* ctx);
void PollOTP(DbmsCtx* ctx);
void PollComm(DbmsCtx* ctx);
void PollOTUT(DbmsCtx* ctx);
void PollOVUV(DbmsCtx* ctx);
void PollSYS(DbmsCtx* ctx);
void PollPWR(DbmsCtx* ctx);

void CheckFaultSummary(DbmsCtx* ctx);