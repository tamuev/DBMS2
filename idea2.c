#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>


#define N_SEGMENTS          1       // number of segments in the stack
#define N_SIDES_PER_SEG     2       // number of sides per segment
#define N_MONITORS_PER_SIDE 1       // number of monitors per side
#define N_GROUPS_PER_SIDE   13      // number of voltages per side
#define N_TEMPS_PER_MONITOR 13       // number of temps per monitor chip 
#define N_P_GROUP           3       // number of cells per parallel group

#define N_TEMPS_PER_SIDE (N_MONITORS_PER_SIDE * N_TEMPS_PER_MONITOR)
#define N_SIDES (N_SEGMENTS * N_SIDES_PER_SEG)
#define N_MONITORS (N_SEGMENTS * N_SIDES_PER_SEG * N_MONITORS_PER_SIDE)
#define N_STACKDEVS (N_MONITORS + 1) // technically "bus devs"

// External dependencies
typedef struct {
    struct { int uart; } hw;
    struct { int n_rx_stack_frames, n_rx_stack_bad_crcs; } stats;
    struct 
    {
        float voltages[N_GROUPS_PER_SIDE];
        float temps[N_TEMPS_PER_SIDE];
        bool cells_to_balance[N_GROUPS_PER_SIDE];  
    } cell_states[N_SIDES];
} DbmsCtx;

uint16_t CalcCrc(uint8_t*, int);
int HAL_UART_Transmit(int, char*, int, int);


/**
 * IMPORTANT: make sure struct has no padding...
 * ... which is any struct that goes over the wire.
 */
#define PACKED __attribute__((packed))

/**
 * Swapping endian-ness is super important when using 
 * on-the-wire structs. We need an endian-swap (eswap)
 * function for 16 and 32 bit. 
 * 
 * For runtime variables, we use the __builtin_bswap* 
 * functions, as these compile to the rev* instructions.
 * But for constants, we use the traditional method
 * so it compiles in a constexpr context. We can detect 
 * if something is constant using __builtin_constant_p.
 */
// #define ESWAP16_CONST(x) \
//     (uint16_t)((((uint16_t)(x) & 0x00FFu) << 8) | \
//                (((uint16_t)(x) & 0xFF00u) >> 8))
// #define ESWAP32_CONST(x) \
//     ((((uint32_t)(x) & 0x000000FFu) << 24) | \
//      (((uint32_t)(x) & 0x0000FF00u) << 8)  | \
//      (((uint32_t)(x) & 0x00FF0000u) >> 8)  | \
//      (((uint32_t)(x) & 0xFF000000u) >> 24))

// #define ESWAP16(x)  (__builtin_constant_p(x) ? ESWAP16_CONST(x) : __builtin_bswap16(x))
// #define ESWAP32(x)  (__builtin_constant_p(x) ? ESWAP32_CONST(x) : __builtin_bswap32(x))

#define ESWAP16(x) __builtin_bswap16((uint16_t)(x))
#define ESWAP32(x) __builtin_bswap32((uint32_t)(x))

#define STACK_SEND_TIMEOUT  10
#define STACK_RECV_TIMEOUT  10

#define STACK_V_UV_PER_BIT 190.73
#define STACK_T_UV_PER_BIT 152.59

/**
 * Important data
 */
#define MAX_TX_DATA 8
#define MAX_RX_DATA 128

#define REQ_SINGLE_DEV_READ         0   // 0x08
#define REQ_SINGLE_DEV_WRITE        1   // 0x09
#define REQ_STACK_READ              2   // 0x0A
#define REQ_STACK_WRITE             3   // 0x0B
#define REQ_BROADCAST_READ          4   // 0x0C
#define REQ_BROADCAST_WRITE         5   // 0x0D
#define REQ_BROADCAST_WRITE_REV     6   // 0x0E


typedef struct PACKED _TxStackFrame 
{
    uint8_t     __cmd   : 1;                /* must be set to 1 */
    uint8_t     reqtype : 3;                /* one of REQ_* */
    uint8_t     __res   : 1;                /* reserved bit */
    uint8_t     len     : 3;                
    uint8_t     devaddr;
    uint16_t    regaddr;
    uint8_t     data[MAX_TX_DATA];
    uint16_t    __crc;                      /* extra buffer for CRC if all data bytes
                                                are used. So the real location of CRC
                                                is at f->data + f->len */
} TxStackFrameDEV;

typedef struct PACKED _TxStackFrame 
{
    uint8_t     __cmd   : 1;                /* must be set to 1 */
    uint8_t     reqtype : 3;                /* one of REQ_* */
    uint8_t     __res   : 1;                /* reserved bit */
    uint8_t     len     : 3;                
    uint16_t    regaddr;
    uint8_t     data[MAX_TX_DATA];
    uint16_t    __crc;                      /* extra buffer for CRC if all data bytes
                                                are used. So the real location of CRC
                                                is at f->data + f->len */
} TxStackFrameSTACK;

typedef struct PACKED _RxStackFrame
{
    uint8_t     __cmd   : 1;                /* must be set to 0 */
    uint8_t     len     : 7;                
    uint8_t     devaddr;
    uint16_t    regaddr;
    uint8_t     data[MAX_RX_DATA];
    uint16_t    __crc;                      /* extra buffer for CRC if all data bytes
                                                are used. So the real location of CRC
                                                is at f->data + f->len */
} RxStackFrame;

/**
 * Helpful macros
 */

#define CRC(F)          *((uint16_t*)((F).data + (F).len))

#define CALC_CRC(F)     CalcCrc((uint8_t*)(&F), (4 + (F).len))

#define FRAME_LEN_SD(F)    ((F).len + 6)
#define FRAME_LEN_STK(F)    ((F).len + 5)

#define MAKE_TX_FRAME_SINGLE_DEV(REQTYPE, DEVADDR, REGADDR, ...)   \
((TxStackFrameDEV){                                         \
    .__cmd   = 1,                                           \
    .reqtype = (REQTYPE),                                   \
    .__res   = 0,                                           \
    .len     = sizeof((uint8_t[]){ __VA_ARGS__ }),          \
    .devaddr = (DEVADDR),                                   \
    .regaddr = ESWAP16(REGADDR),                            \
    .data    = { __VA_ARGS__ },                             \
    .__crc   = 0                                            \
})

#define MAKE_TX_FRAME_STACK(REQTYPE, REGADDR, ...)          \
((TxStackFrameSTACK){                                       \
    .__cmd   = 1,                                           \
    .reqtype = (REQTYPE),                                   \
    .__res   = 0,                                           \
    .len     = sizeof((uint8_t[]){ __VA_ARGS__ }),          \
    .regaddr = ESWAP16(REGADDR),                            \
    .data    = { __VA_ARGS__ },                             \
    .__crc   = 0                                            \
})

#define DATA(...) __VA_ARGS__

// example: 
static uint8_t frame1[] = {0xB2, 0x00, 0x0E, 0x09, 0x2D, 0x09, 0x00, 0x00};
// becomes:
static TxStackFrame frame2 = MAKE_TX_FRAME(REQ_STACK_WRITE, 0, 0x0E09, DATA(0x2D, 0x09) );


/**
 * We need a new send function now
 */
int SendStackFrame(DbmsCtx* ctx, TxStackFrame frame)
{
    CRC(frame) = ESWAP16(CALC_CRC(frame));
    return HAL_UART_Transmit(ctx->hw.uart, (uint8_t*)&frame, FRAME_LEN(frame), STACK_SEND_TIMEOUT);
}

/**
 * And we can use this like
 */
int StackSetupGpio(DbmsCtx* ctx)
{
    SendStackFrame(ctx, MAKE_TX_FRAME(REQ_STACK_WRITE, 0, 0x0E09, /*data=*/ 0x2D, 0x09));

    SendStackFrame(ctx, MAKE_TX_FRAME(REQ_STACK_WRITE, 0x03, 0x0A01));
}

int ReadStackFrame(DbmsCtx* ctx, RxStackFrame* frame, size_t datalen)
{
    int status = 0;
    if ((status = HAL_UART_Receive(ctx->hw.uart, (uint8_t*)frame, 6 + datalen, STACK_RECV_TIMEOUT)) != 0) 
    { 
        return 1;
    }     
    ctx->stats.n_rx_stack_frames++;

    if (CRC(*frame) != CALC_CRC(*frame))
    {
        ctx->stats.n_rx_stack_bad_crcs++;
        return 1;
    }
    return 0;
}


/**
 * For recv we can abstract away a lot of the logic
 */
void StackUpdateAllVoltReadings(DbmsCtx* ctx)
{
    int regaddr = (0x68 + 2 * (16 - N_GROUPS_PER_SIDE) << 8) + (N_GROUPS_PER_SIDE * 2 - 1);
    SendStackFrame(ctx, MAKE_TX_FRAME(REQ_STACK_READ, 0x05, regaddr));

    // // can we read segment by segment? or do we have to read them all at once 
    // for (int i = 0; i < N_MONITORS; i++)
    // {
    //     RxStackFrame frame;
    //     ReadStackFrame(ctx, &frame, N_GROUPS_PER_SIDE * sizeof(uint16_t));

    //     uint16_t* voltages = (uint16_t*)(frame.data);   
    //     for (size_t j = 0; j < N_GROUPS_PER_SIDE; j++)
    //     {
    //         uint16_t raw = ESWAP16(voltages[j]);
    //         ctx->cell_states[frame.devaddr / 2].voltages[j] = (raw * STACK_V_UV_PER_BIT) / 1000.0; // floating mV
    //     }
    // }

    // if we read them all at once:
    uint8_t raw[4096];
    HAL_Read(raw);      // read into the buffer
    int offset = 0;
    for (int i = 0; i < N_MONITORS; i++)
    {
        RxStackFrame* frame = raw[offset];
        // CRC check
        ReadStackFrame(ctx, &frame, N_GROUPS_PER_SIDE * sizeof(uint16_t));
        offset += FRAME_LEN(frame);


    // You get the idea... prob need some little code edits here and there
}

/**
 * For recv we can abstract away a lot of the logic
 */
void StackUpdateAllVoltReadings(DbmsCtx* ctx)
{
    int regaddr = (0x68 + 2 * (16 - N_GROUPS_PER_SIDE) << 8) + (N_GROUPS_PER_SIDE * 2 - 1);
    SendStackFrame(ctx, MAKE_TX_FRAME(REQ_STACK_READ, 0x05, regaddr));

    // // can we read segment by segment? or do we have to read them all at once 
    // for (int i = 0; i < N_MONITORS; i++)
    // {
    //     RxStackFrame frame;
    //     ReadStackFrame(ctx, &frame, N_GROUPS_PER_SIDE * sizeof(uint16_t));

    //     uint16_t* voltages = (uint16_t*)(frame.data);   
    //     for (size_t j = 0; j < N_GROUPS_PER_SIDE; j++)
    //     {
    //         uint16_t raw = ESWAP16(voltages[j]);
    //         ctx->cell_states[frame.devaddr / 2].voltages[j] = (raw * STACK_V_UV_PER_BIT) / 1000.0; // floating mV
    //     }
    // }

    // if we read them all at once:
    uint8_t raw[4096];
    HAL_Read(raw);      // read into the buffer
    int offset = 0;

    // OPTION 1

    // process the raw buffer
    RxStackFrame_V frames[N_MONITORS]; // Since there are only 2 types of frames we receive, having 2 structs to account for different
    // data lengths made sense

    /*This function parses through and populates 'frames' with pointers to the start of each valid frame 
    (without any trash bytes from EMI)
    It also does CRC checks and increments all crc related counters
    search_byte is the byte we want to look for, if not provided, we will simply look for the response frame byte with 1 data byte
    */ 
    parsebuffer(ctx, &raw, &frames, N_MONITORS /*n_frames*/, datalen, /*optional: */ search_byte);

    for (int i = 0; i < N_MONITORS; i++)
    {
        RxStackFrame* frame = frames[i];
        
        offset += FRAME_LEN(frame);

    // OPTION 2:

    // option 1 involved going through the buffer essentially twice,

    void StackUpdateAllTempReadings(DbmsCtx* ctx)
    {
        int status = 0;
        static uint8_t rx_buffer_t[1024];

        // uint8_t frame2[] = {0xA0, 0x05, 0x8E, ((N_TEMPS_PER_MONITOR / 3) + 2) * 2 - 1, 0x00, 0x00};
        // replace with appropriate sending new sending logic

        // if ((status = SendStackFrameSetCrc(ctx, frame2, sizeof(frame2))) != 0) { }

        size_t data_size = ((N_TEMPS_PER_MONITOR / 3) + 2) * sizeof(int16_t);
        size_t expected_rx_size = RX_FRAME_SIZE(data_size) * N_MONITORS;
        
        // TODO: redo the RX path for stack
        if ((status = HAL_UART_Receive(ctx->hw.uart, rx_buffer_t, expected_rx_size+1, STACK_RECV_TIMEOUT)) != 0) { } 
        for (int i = 0; i < N_MONITORS; i++)
        {
            ctx->stats.n_rx_stack_frames++;
            ctx->stats.n_rx_stack_frames_itvl++;
            ctx->faults.monitor_total_frames[i]++;

            uint8_t* data = rx_buffer_t + (i * RX_FRAME_SIZE(data_size));
            for (int j = 0; data[0] != frame2[2] && j < 1024; j++)
            {
                data++;
            }
            data++;
            RxStackFrame_V* frame = data - 4;
            // uint8_t addr = *(data-3);
            // uint16_t f_crc = (data[data_size]) + (data[data_size+1] << 8);
            // uint16_t c_crc = CalcCrc16(data-4, data_size+4);
            if (CRC(*(frame)) != CALC_CRC(*(frame)))
            {
                ctx->stats.n_rx_stack_bad_crcs++;
                ctx->stats.n_rx_stack_bad_crcs_itvl++;
                ctx->faults.monitor_bad_crcs[i]++;
                continue;
            }
            // for (int j = 0; j < 4; j++){ // GPIO1 = GPIO3, GPIO2 = GPIO4
            //     data[j + 4] = data[j];
            // }
            // data += 4;
            // uint8_t offset = ctx->mux_selector;
            // uint8_t temps = (offset == 0 ? 4 : 3);
            for (size_t j = 0; j < temps; j++)
            {
                uint16_t raw = (frame->data[j * sizeof(uint16_t)] << 8) + (frame->data[j * sizeof(uint16_t) + 1]);
                ctx->cell_states[addr-1].temps[4 * j + offset] = ThermVoltToTemp(ctx, MAX(0, raw * STACK_T_UV_PER_BIT / 1000000.0));
            }
            // problem with this option is the readibility doesnt get much better
        }
    }
}