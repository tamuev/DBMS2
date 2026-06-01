/**
 *
 * Distributed BMS      Global BMS Context
 *
 * Copyright (C) 2025   Texas A&M University
 *
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#ifndef _H_CTX_
#define _H_CTX_

#include "utils/common.h"

// USER DEFINED
#define ITER_TARGET_HZ      20      // how many iterations per second to target

#define SPLIT_STACK_OPS     1       // 1 = divide stack ops in half, every-other-iter, 0 = do not

#define N_SEGMENTS          5       // number of segments in the stack
#define N_SIDES_PER_SEG     2       // number of sides per segment
#define N_MONITORS_PER_SIDE 1       // number of monitors per side
#define N_GROUPS_PER_SIDE   13      // number of voltages per side
#define N_TEMPS_PER_MONITOR 13       // number of temps per monitor chip
#define N_P_GROUP           3       // number of cells per parallel group

// #define HAS_FAN
// DONT CHANGE AFTER THIS

#define N_TEMPS_PER_SIDE (N_MONITORS_PER_SIDE * N_TEMPS_PER_MONITOR)
#define N_SIDES (N_SEGMENTS * N_SIDES_PER_SEG)
#define N_MONITORS (N_SEGMENTS * N_SIDES_PER_SEG * N_MONITORS_PER_SIDE)
#define N_STACKDEVS (N_MONITORS + 1) // technically "bus devs"
#define N_TEMPS_POLL_PER_MONITOR ((N_TEMPS_PER_MONITOR / 4) + 1) // Mux boards

#define ADDR_BCAST_TO_STACK(BCAST_ADDR) (BCAST_ADDR - 1)
#define ADDR_STACK_TO_BCAST(STACK_ADDR) (STACK_ADDR + 1)

#define CELL_BYTE_NA 0xFF
#define CELL_BYTE(side, cell) ((((side) & 0xF) << 4) | ((cell) & 0xF))

#define N_THERM_V_TO_T_ENTRIES      121
#define N_TEMPS                     5
#define N_OCV_ENTRIES               201
#define N_DCIR_ENTRIES              201
#define N_C_ENTRIES                 101
#define N_I_MA_MEMORIZED            100

#define ALPHA                       0.25
#define BETA                        0.125

// DANGER:  THESE DEBUGS WILL PREVENT THE CONTROLLER FROM WORKING NORMALLY
// #define DEBUG_DO_OVERWRITE_TEMPS_OVER_CAN
// END DANGER ZONE

typedef enum _DbmsState
{
    DBMS_ACTIVE = 0,
    DBMS_SHUTDOWN,
    DBMS_CHARGING
} DbmsState;

typedef enum _ChargingState
{
    CH_NO_CONN = 0,
    CH_CHARGING,
    CH_WAIT_1,
    CH_BALANCING_ODDS,
    CH_BALANCING_EVENS,
    CH_WAIT_2,
    CH_COMPLETE
} ChargingState;

// Hardware context stores prts
// to peripheral interfaces
typedef struct _HwCtx
{
    ADC_HandleTypeDef* adc;
    TIM_HandleTypeDef* timer;
    TIM_HandleTypeDef* timer_pwm_1;
    UART_HandleTypeDef* uart;
    I2C_HandleTypeDef* i2c;

    CAN_HandleTypeDef* elcon_can; // charger bus -- 500k
    CAN_HandleTypeDef* can; // vehicle bus -- 1m
    CAN_TxHeaderTypeDef can_tx_header;
    uint32_t can_tx_mailbox;
} HwCtx;

typedef struct _CellMonitorState
{
    float voltages[N_GROUPS_PER_SIDE];
    float temps[N_TEMPS_PER_SIDE];
    bool cells_to_balance[N_GROUPS_PER_SIDE];   // TODO: change to bitmask? prob not
} CellMonitorState;

// fwd definition -- settings.h
typedef enum _UserSettingIndex UserSettingIndex;
typedef struct _DbmsSettings DbmsSettings;

// fwd definition for enum -- led_controller.h
typedef int32_t LedState;

typedef struct _Stats
{
    uint64_t iters;
    // #define N_HISTORIC_LOOPTIMES 16
    //         wrap_queue_t looptimes_q;
    //         uint32_t looptimes_d[N_HISTORIC_LOOPTIMES];

    uint32_t shutdown_start_us;

    uint32_t n_tx_can_frames;
    uint32_t n_rx_can_frames;
    uint32_t n_unmatched_can_frames;
    uint32_t n_tx_can_drop_queue_full;
    uint32_t n_tx_can_fail;
    uint32_t n_tx_queued;

    uint32_t looptime;
    uint32_t end_delay;

    uint32_t n_rx_stack_frames;
    uint32_t n_rx_stack_bad_crcs;
    uint32_t n_rx_stack_frames_itvl;
    uint32_t n_rx_stack_bad_crcs_itvl;
    // uint32_t n_overruns;

    uint32_t n_eeprom_writes;
    uint8_t n_int_shutdowns;

    float min_v;
    float max_v;
    float avg_v;
    float min_t;
    float max_t;
    float avg_t;

    uint8_t min_v_cell;
    uint8_t max_v_cell;
    uint8_t min_t_cell;
    uint8_t max_t_cell;

    float pack_v;

    uint32_t elcon_rx;

    uint64_t n_logging_frames;
    bool fault_line_faulted;

    uint16_t monitor_bad_crcs[N_MONITORS];
    uint16_t monitor_total_frames[N_MONITORS];
    uint64_t last_can_tx_ts;
    uint64_t last_ivt_rx_ts;

    uint32_t can_esr_reg;
    uint64_t last_can_bo_ts;
    uint64_t last_can_bad_ts;
    uint16_t n_can_bussoffs;

    uint64_t last_monitor_msg[N_MONITORS]; // Last iter that a message was received from each monitor

    uint32_t distance;       // Current lifetime total = distance_base + plex_session
    uint32_t distance_base;    // D0: lifetime total at the start of this power cycle
    uint32_t plex_session;     // Most recent session distance received from the plex (0x624)
} Stats;

typedef struct _Model   // Outputs from the ECM model
{
    float Q;            // Charge
    float z_oc;         // % Charge (OC path)
    float z_rc;         // % Charge (RC path)
    float V_oc;         // Open Circuit Voltage
    float R_oc;

    float R_rc;

    float R_cell;
    float I_lim;
    float P_lim;
} Model;

typedef struct _Snapshot
{
    uint64_t iter;

    // Current sensor data
    int32_t current_ma;
    int32_t voltage_mv;

    // Charge stats
    float qd;

    // Current limits and resistance
    float current_limit_a;
    float dcir;
    float total_resistance;

    // Temperature stats
    float avg_temp;
    float max_temp;
    float min_temp;

    // Cell voltage delta
    float cell_delta_v;

    // Voltage stats
    float high_voltage;
    float low_voltage;
    float avg_voltage;

    // Faults
    uint32_t controller_mask;
    uint8_t bridge_fault_summary;
    uint32_t bridge_faults;
    uint8_t monitor_fault_summary[N_MONITORS];

} Snapshot;

typedef struct _CurrentSensor {
    int32_t current_ma;
    int32_t voltage1_mv;
    int32_t power_w;
    int32_t charge_as;
    float q_offset;
    bool has_q_offset;
    int32_t energy_wh;

    struct {
        ma_t ma;
        int32_t buf[N_I_MA_MEMORIZED];
        int32_t current_mavg_ma;
    } ima;      // charge = I, ma = moving average (ang milliamps, so sometimes mavg)

} CurrentSensor;

typedef struct _Profiling {
    struct
    {
        uint64_t T0;
        uint64_t T1;
        uint64_t T2;
        uint64_t T3;
        uint64_t T4;
        uint64_t T5;
        uint64_t T6;
        uint64_t T7;
        uint64_t T8;
        uint64_t T9;
        uint64_t T10;
    } times;
} Profiling;

typedef struct _Delay {
    // used for calculating one-way delay (bus latency from DCU)
    uint64_t T1;
    uint64_t T2;
    uint64_t T3;
    uint64_t T4;
    uint64_t mu;
    uint64_t st_dev;
    uint64_t one_way_delay;
    uint64_t clock_offset;
} Delay;

typedef struct _LutData {
    LTE lut_therm_v_to_t[N_THERM_V_TO_T_ENTRIES];
} LutData;

typedef struct _Realtime {
    uint64_t global_ts;
    uint64_t local_ts;
} Realtime;

typedef struct _Timing {
    int64_t last_rx_heartbeat;
    uint64_t last_rx_telembeat;
    uint64_t iter_start_us;
    uint64_t iter_end_us;
    uint64_t m_led_blink_ts;
    uint64_t batch_telem_ts;
    uint64_t wakeup_ts;
    uint64_t pl_last_ok_ts;
    uint64_t pl_pulse_t;
    uint64_t overtemp_last_ok_ts;
} Timing;

typedef struct _Flags {
    bool active;
    bool need_to_reset_qstats;
    bool telem_enable;
    bool need_to_save_faults;
    bool need_to_save_blackbox;
    bool req_fault_clear;
    bool need_to_sync_settings;
    bool m_led_on;
    bool has_balanced;
    bool shutdown_requested;
    bool shutdown_stack_requested;
    bool need_to_save_bad_therms;
    bool fan_on;
} Flags;

typedef struct _ChargeStats {
    float initial;
    float historic_accumulated_loss;
    float accumulated_loss;
    uint32_t initial_set_ts;
} ChargeStats;

typedef struct __attribute__((packed)) _FaultData {
    uint8_t cell;
    uint8_t n_throws;
    uint16_t value;
} FaultData;

typedef struct _FaultState {
    uint32_t active_faults;         // Currently active fault/warning conditions - Resets when condition is clear
    uint32_t latched_faults;        // Currently latched fault/warning conditions - Resets with app clear
    uint32_t historic_faults;       // Historic faults/warnings even if non-latching - Resets with app clear

    // Fault config
    uint32_t warnings_config;       // Which faults are considered warnings
    uint32_t nonlatching_config;    // Which faults are non-latching

    FaultData fault_data[32];

    uint8_t bridge_fault_summary;
    uint32_t bridge_faults;
    uint8_t monitor_fault_summary[N_MONITORS];
    bool had_fault;
} FaultState;

typedef struct _Blackbox {
    uint8_t head;
    uint8_t count;
    bool requested;
    bool ready;
} Blackbox;

typedef struct _Elcon {
    uint64_t heartbeat;
    int32_t voltage_out;
    int32_t current_out;
    uint8_t status_flags;
    int16_t v_req;
    int16_t i_req;
} Elcon;

typedef struct _J1772 {
    uint32_t cp_duty_cycle;
    uint32_t pp_raw_voltage;
    bool pp_connect;
    bool charge_en_req;
    int64_t last_cp_pwm_read;
    int64_t pwm_ts;
    bool pwm_recieved;
    uint32_t maxCurrentSupply;
} J1772;

typedef struct _Charging {
    int64_t heartbeat;
    int64_t bal_loop_hb;
    int64_t state_enter_ts;
    ChargingState prev_state;
    ChargingState state;
    float pre_bal_accumulator[N_SIDES][N_GROUPS_PER_SIDE];
    float pre_bal_average_v[N_SIDES][N_GROUPS_PER_SIDE];
    float pre_bal_min_v;
    float pre_bal_max_v;
    size_t pre_bal_sample_count;
    bool allowed;
    bool conn;
    bool only_balance;
} Charging;

typedef struct _DbmsCtx
{
    HwCtx hw;
    LutData data;
    LedState led_state;
    DbmsSettings* settings;
    CellMonitorState cell_states[N_SIDES];
    uint16_t bad_therm_mask[N_SIDES];
    uint8_t mux_selector;
    Realtime realtime;
    Timing timing;
    Flags flags;
    Stats stats;
    CurrentSensor current_sensor;
    uint32_t max_current_ma;
    ChargeStats qstats;
    float initial_historic_accumulated_loss;
    FaultState faults;
    uint16_t faults_crc;
    Model model;
    uint16_t can_log_ordering_index;
    uint32_t last_can_err;
    bool precharged;
    Blackbox blackbox;
    Elcon elcon;
    J1772 j1772;
    Charging charging;
    Profiling profiling;
    Delay delay;
} DbmsCtx;


#endif
