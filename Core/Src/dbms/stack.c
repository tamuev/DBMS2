/** 
 * 
 * Distributed BMS      Stack Controller Module
 *
 * Copyright (C) 2025   Texas A&M University
 * 
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#include "stack.h"

static uint8_t bad_therms[] = {
    CELL_BYTE(5, 12),
    CELL_BYTE(8, 11)
    // CELL_BYTE(3, 12),
    // CELL_BYTE(6, 11)
};

bool IsThermBad(uint8_t side, uint8_t group)
{
    for (int i = 0; i < sizeof(bad_therms); ++i) 
    {
        if (CELL_BYTE(side, group) == bad_therms[i]) return false;
    }
    return false;
}

/**
 * Sends a wake blip to the battery stack
 * 
 * @param ctx Context pointer 
 * @return Error code 
 */
int SendStackWakeBlip(DbmsCtx* ctx)
{
    return SendStackBlip(ctx, 25700 / 2);
}
/**
 * Sends a shutdown blip to the battery stack
 * 
 * @param ctx Context pointer 
 * @return Error code 
 */
int SendStackShutdownBlip(DbmsCtx* ctx)
{
    return SendStackBlip(ctx, 128000);
}

/**
 * @brief Wake the battery stack
 * 
 * @param ctx Context pointer 
 * @return Error code 
 */
int StackWake(DbmsCtx* ctx)
{
    int status = 0;
    for (int i = 0; i < 2; i++)
    {
        if ((status = SendStackWakeBlip(ctx)) != 0)
        {
            CAN_REPORT_FAULT(ctx, status);
            return status;
        }
        if ((status = SINGLE_DEV_WRITE(ctx, 0, REG_CONTROL1, DATA(CTRL1_SEND_WAKE))) != 0)
        {
            CAN_REPORT_FAULT(ctx, status);
            return status;
        }
        HAL_Delay(15 + 12 * N_STACKDEVS);
    }
    return status;
}


/**
 * @brief Shuts down the battery stack
 * Use this when we turn the vehicle off, or when going to sleep
 * We could also use this in response to a critical fault
 * 
 * @param ctx Context pointer 
 * @return Error code 
 */
int StackShutdown(DbmsCtx* ctx)
{
    int status = 0;
    for (int i = 0; i < 2; i++)
    {
        if ((status = BROADCAST_WRITE(ctx, REG_CONTROL1, DATA(CTRL1_GOTO_SHUTDOWN))) != 0)
        {
            CAN_REPORT_FAULT(ctx, status);
            return status;
        }
        if ((status = SendStackShutdownBlip(ctx)) != 0)
        {
            CAN_REPORT_FAULT(ctx, status);
            return status;
        }
        HAL_Delay(100);
    }
    return status;
}

void SendOtpEccDatain(DbmsCtx* ctx)
{
    for (int i = 0; i < 8; i++)
    {
        BROADCAST_WRITE(ctx, REG_OTP_ECC_DATAIN1 + i, DATA(0x00));
        HAL_Delay(1);
    }
}

void SendAutoAddr(DbmsCtx* ctx)
{    
    for (int i = 0; i <= N_STACKDEVS; i++)
    {
        BROADCAST_WRITE(ctx, REG_DIR0_ADDR, DATA(0x00 + i));
        HAL_Delay(1);
    }
}

void SendSetStackTop(DbmsCtx* ctx)
{
    // Sets all devices as stack devices
    BROADCAST_WRITE(ctx, REG_COMM_CTRL, DATA(COMM_STACK_DEV));
    HAL_Delay(1);
    // Sets bridge device as non-stack device and bottom of stack
    SINGLE_DEV_WRITE(ctx, 0, REG_COMM_CTRL, DATA(0x00));
    HAL_Delay(1);
    // Sets top of stack
    SINGLE_DEV_WRITE(ctx, N_STACKDEVS-1, REG_COMM_CTRL, DATA(COMM_STACK_DEV | COMM_TOP_STACK));
    HAL_Delay(1);
}

void ReadOtpEccDatain(DbmsCtx* ctx)
{
    for (int i = 0; i < 8; i++)
    {
        BROADCAST_READ(ctx, REG_OTP_ECC_TEST + i, 1);
        HAL_Delay(1);
    }
}


/**
 * @brief Full auto addressing procedure
 * 
 * @param ctx Context pointer 
 */
void StackAutoAddr(DbmsCtx* ctx)
{
    SendOtpEccDatain(ctx); // step 1

    BROADCAST_WRITE(ctx, REG_CONTROL1, DATA(CTRL1_ADDR_WR));
    HAL_Delay(1);

    SendAutoAddr(ctx); // step 3

    SendSetStackTop(ctx); // step 5

    ReadOtpEccDatain(ctx); // step 6
}

/**
 * @brief Set the number of active cells in the stack
 * 
 * @param ctx Context pointer 
 * @param n_active_cells 
 */
void StackSetNumActiveCells(DbmsCtx* ctx, uint8_t n_active_cells)
{
    BROADCAST_WRITE(ctx, REG_ACTIVE_CELL, DATA(n_active_cells)); // Should this be a stack write?
    HAL_Delay(1);
}


/*****************************
 *  VOLTAGE/TEMP READINGS 
 *****************************/

/**
 * @brief Enable the voltage tap ADCs
 * 
 * @param ctx Context pointer 
 */
void StackSetupVoltReadings(DbmsCtx* ctx)
{
    BROADCAST_WRITE(ctx, REG_ADC_CTRL1, DATA(ADC_CONTINUOUS_RUN | ADC_MAIN_GO));
    HAL_Delay(1);
}

/**
 * @brief Request and populate voltage readings for the whole stack
 * 
 * @param ctx Context pointer 
 */
void StackUpdateAllVoltReadings(DbmsCtx* ctx)
{
    static uint8_t rx_buffer_v[1024];
    int j;
    memset(rx_buffer_v, 0, sizeof(rx_buffer_v));
    size_t data_size = N_GROUPS_PER_SIDE * sizeof(int16_t);
    size_t expected_rx_size = RX_FRAME_SIZE(data_size) * N_MONITORS;

    StackRead(ctx, rx_buffer_v, STACK_V_REG_START, data_size, expected_rx_size);

    for (size_t i = 0; i < N_MONITORS; i++)
    {
        IncStackCrcStats(ctx, true, i);
        // TODO: test without on new battery to see if this is necessary
        uint8_t* data = rx_buffer_v + (i * RX_FRAME_SIZE(data_size));
        for (j = 0; (data[0] != (STACK_V_REG_START & 0xFF)) && (j < 1024); j++) { data++; }
        if (j >= 1024 || j < 3) continue;
        RxStackFrameVoltages* clean_frame = (RxStackFrameVoltages*)(data - 3);
        if (clean_frame->crc == CALC_CRC_Rx(clean_frame))
            UpdateVoltages(ctx, clean_frame);
        else{
            IncStackCrcStats(ctx, false, i);
        }
    }
}

/**
 * @brief Configure's stack GPIO for temp mux and LEDs
 * 
 * @param ctx Context pointer 
 */
void StackSetupGpio(DbmsCtx* ctx)
{
    STACK_WRITE(ctx, REG_GPIO_CONF1, STACK_GPIO_DATA(
        STACK_GPIO_ADC_OTUT_IN,
        STACK_GPIO_ADC_OTUT_IN,
        STACK_GPIO_OUT_LOW,
        STACK_GPIO_OUT_LOW,
        STACK_GPIO_ADC_OTUT_IN,
        STACK_GPIO_ADC_OTUT_IN,
        STACK_GPIO_DISABLED,
        STACK_GPIO_DISABLED
    ));
    HAL_Delay(10);
    // Setting up TSREF to active
    STACK_WRITE(ctx, REG_CONTROL2, DATA(CTRL2_TSREF_EN));
    DelayUs(ctx, 10);
 }

/**
 * @brief Configure the heartbeat timeout
 * 
 * @param ctx Context pointer 
 */
void StackConfigTimeout(DbmsCtx* ctx)
{
    BROADCAST_WRITE(ctx, REG_COMM_TIMEOUT_CONF, DATA(0x0A)); // Shutdown after 2 seconds of no comms
    DelayUs(ctx, 10);
}


void StackUpdateAllTempReadings(DbmsCtx* ctx)
{
    static uint8_t rx_buffer_t[1024];
    int j;
    memset(rx_buffer_t, 0, sizeof(rx_buffer_t));
    size_t data_size = (N_TEMPS_POLL_PER_MONITOR + 2) * sizeof(int16_t); // +2 for GPIO mismatch
    size_t expected_rx_size = RX_FRAME_SIZE(data_size) * N_MONITORS;

    StackRead(ctx, rx_buffer_t, STACK_T_REG_START, data_size, expected_rx_size);
    
    for (size_t i = 0; i < N_MONITORS; i++)
    {
        IncStackCrcStats(ctx, true, i);
        // TODO: test without on new battery to see if this is necessary
        uint8_t* data = rx_buffer_t + (i * RX_FRAME_SIZE(data_size));
        for (j = 0; data[0] != (STACK_T_REG_START & 0xFF) && j < 1024; j++) { data++; }
        if (j >= 1024 || j < 3) continue;
        RxStackFrameTemps* clean_frame = (RxStackFrameTemps*)(data - 3); 
        if (clean_frame->crc == CALC_CRC_Rx(clean_frame))
            UpdateTemps(ctx, clean_frame);
        else{
            IncStackCrcStats(ctx, false, i);
        }
    }
}

void UpdateTemps(DbmsCtx* ctx, RxStackFrameTemps* frame)
{
    uint8_t* moved_data = &(frame->data[4]);
    memmove(moved_data, frame->data, 4); // GPIO1 = GPIO3, GPIO2 = GPIO4
    uint8_t offset = ctx->mux_selector;
    uint8_t temps = (offset == 0 ? N_TEMPS_POLL_PER_MONITOR : N_TEMPS_POLL_PER_MONITOR-1);
    for (size_t j = 0; j < temps; j++)
    {
        uint16_t raw = (moved_data[j * sizeof(int16_t)] << 8) + moved_data[j * sizeof(int16_t) + 1];
        ctx->cell_states[ADDR_BCAST_TO_STACK(frame->devaddr)].temps[N_TEMPS_POLL_PER_MONITOR*j + offset] = 
            ThermVoltToTemp(ctx, MAX(0, raw * STACK_T_UV_PER_BIT / 1000000.0));
    }
}

void UpdateVoltages(DbmsCtx* ctx, RxStackFrameVoltages* frame)
{
    if (ADDR_BCAST_TO_STACK(frame->devaddr) >= N_MONITORS) return;

    ctx->stats.last_monitor_msg[ADDR_BCAST_TO_STACK(frame->devaddr)] = ctx->stats.iters;

    for (size_t j = 0; j < N_GROUPS_PER_SIDE; j++)
    {
        uint16_t raw = (frame->data[j * sizeof(int16_t)] << 8) + frame->data[j * sizeof(int16_t) + 1];
        ctx->cell_states[ADDR_BCAST_TO_STACK(frame->devaddr)].voltages[N_GROUPS_PER_SIDE - j - 1] = (raw * STACK_V_UV_PER_BIT) / 1000.0; // floating mV
    }
}

int StackRead(DbmsCtx* ctx, uint8_t* raw, uint16_t start_reg, uint8_t data_size, int expected_size)
{
    int status = 0;
    if ((status = STACK_READ(ctx, start_reg, data_size)) != 0) 
    {
        return status;
    }
    __HAL_UART_CLEAR_FEFLAG(ctx->hw.uart);
    __HAL_UART_CLEAR_OREFLAG(ctx->hw.uart);
    __HAL_UART_CLEAR_NEFLAG(ctx->hw.uart);
    volatile uint8_t dummy = ctx->hw.uart->Instance->DR;
    (void) dummy;
    // TODO: redo the RX path for stack
    if ((status = HAL_UART_Receive(ctx->hw.uart, raw, expected_size+2, STACK_RECV_TIMEOUT)) != 0)
    {
    }
    return status;
}

/**
 * @brief Replace missing thermistor readings with the average of the valid cells
 * 
 * @param ctx Context pointer 
 */
void FillMissingTempReadings(DbmsCtx* ctx)
{
    int n = 0;
    float sum = 0;

    uint32_t low_plaus = GetSetting(ctx, LOW_PLAUSIBLE_TEMP);
    uint32_t high_plaus = GetSetting(ctx, HIGH_PLAUSIBLE_TEMP);

    for (int i = 0; i < N_SIDES; i++)
    {
        for (int j = 0; j < N_TEMPS_PER_SIDE; j++)
        {
            float t = ctx->cell_states[i].temps[j];
            // TODO: remove (3, 7) blacklisted
            if ((t >= low_plaus && t < high_plaus) || (i == 3 && j == 7))
            {
                n++;
                sum += t;
            }
        }
    }

    float avg = sum / n;
    for (int i = 0; i < N_SIDES; i++)
    {
        for (int j = 0; j < N_TEMPS_PER_SIDE; j++)
        {
             float t = ctx->cell_states[i].temps[j];
            if ((t < low_plaus || t >= high_plaus) || (i == 3 && j == 7))
            {
                ctx->cell_states[i].temps[j] = avg;
            }
        }
    }
}

/**
 * @brief Compute min, avg, max voltages and temperatures 
 * 
 * @param ctx Context pointer 
 */
void StackCalcStats(DbmsCtx* ctx)
{
    float v_min = 999999.0f, v_max = 0.0f, v_sum = 0.0f;
    float t_min = 999.0f,    t_max = 0.0f, t_sum = 0.0f;
    uint8_t v_max_cell = CELL_BYTE_NA, 
            v_min_cell = CELL_BYTE_NA, 
            t_max_cell = CELL_BYTE_NA, 
            t_min_cell = CELL_BYTE_NA;

    for (int i = 0; i < N_SIDES; i++)
    {
        for (int j = 0; j < N_GROUPS_PER_SIDE; j++)
        {
            float v = ctx->cell_states[i].voltages[j];

            if (v < v_min)
            {
                v_min = v;
                v_min_cell = CELL_BYTE(i, j);
            }

            if (v > v_max)
            {
                v_max = v;
                v_max_cell = CELL_BYTE(i, j);
            }

            v_sum += v;
        }

        for (int j = 0; j < N_TEMPS_PER_SIDE; j++)
        {
            if (IsThermBad(i, j)) continue;

            float t = ctx->cell_states[i].temps[j];

            if (t < t_min)
            {
                t_min = t;
                t_min_cell = CELL_BYTE(i, j);
            }

            if (t > t_max)
            {
                t_max = t;
                t_max_cell = CELL_BYTE(i, j);
            }

            t_sum += t;
        }
    }

    ctx->stats.min_v = v_min / 1000.0f;
    ctx->stats.max_v = v_max / 1000.0f;
    ctx->stats.avg_v = v_sum / (N_SIDES * N_GROUPS_PER_SIDE) / 1000.0f;
    ctx->stats.pack_v = v_sum;

    ctx->stats.min_t = t_min;
    ctx->stats.max_t = t_max;
    ctx->stats.avg_t = t_sum / (N_SIDES * N_TEMPS_PER_SIDE - sizeof(bad_therms));

    ctx->stats.min_v_cell = v_min_cell;
    ctx->stats.max_v_cell = v_max_cell;
    ctx->stats.min_t_cell = t_min_cell;
    ctx->stats.max_t_cell = t_max_cell;
}

void IncStackCrcStats(DbmsCtx* ctx, bool good, int monitor_id)
{
    if (good)
    {
        ctx->stats.n_rx_stack_frames++;
        ctx->stats.n_rx_stack_frames_itvl++;
        ctx->stats.monitor_total_frames[monitor_id]++;
    }
    else
    {
        ctx->stats.n_rx_stack_bad_crcs++;
        ctx->stats.n_rx_stack_bad_crcs_itvl++;
        ctx->stats.monitor_bad_crcs[monitor_id]++;
    }
}

/*****************************
 *   LED CONTROL
 *****************************/

// TODO: document + reeval confusing names

int ToggleMonitorLeds(DbmsCtx* ctx, bool on)
{
    // Turns on or off LED connected to GPIO8 on all monitor chips
    // Note for future:
    int status = 0;
    StackGPIOMode mode = on ? STACK_GPIO_OUT_HIGH : STACK_GPIO_OUT_LOW;
    if ((status = STACK_WRITE(ctx, REG_GPIO_CONF4, DATA(mode))) != 0)
    {
        return status;
    }
    return status;
}

void MonitorLedBlink(DbmsCtx* ctx)
{
    //if (!ctx->flags.active) return;

    uint64_t curr_ts = GetUs(ctx) - ctx->timing.m_led_blink_ts;

    if (ctx->flags.m_led_on)
    {
        if (curr_ts > 100000)
        {
            ToggleMonitorLeds(ctx, false);
            ctx->flags.m_led_on = false;
            ctx->timing.m_led_blink_ts = GetUs(ctx);
        }
    }
    else
    {
        if (curr_ts > 1000000)
        {
            ToggleMonitorLeds(ctx, true);
            ctx->flags.m_led_on = true;
            ctx->timing.m_led_blink_ts = GetUs(ctx);
        }
    }
}

/*****************************
 *  BALANCING 
 *****************************/

void StackBalancingConfig(DbmsCtx* ctx)
{
    // Setting balancing method to auto balancing and to stop at fault
    STACK_WRITE(ctx, REG_BAL_CTRL2, DATA(BAL2_FLTSTOP_EN | BAL2_OTCB_EN | BAL2_AUTO_BAL));
}

void StackSetDeviceBalanceTimers(DbmsCtx* ctx, uint8_t side, bool odds, StackBalanceTimes bal_time_idx)
{
    bool* cells_to_bal = ctx->cell_states[side].cells_to_balance;

    uint16_t base_reg = REG_CB_CELL1_CTRL;
    uint8_t bal_time = MIN((uint8_t)bal_time_idx, (uint8_t)__N_BAL_TIMES);
    // Write each cell timer individually
    for (size_t i = 0; i < N_GROUPS_PER_SIDE; ++i)
    {
        uint16_t reg_addr = base_reg - i; // registers decrement
        uint8_t timer_val = i % 2 == odds && cells_to_bal[i] ? bal_time : 0x00;
        
        SINGLE_DEV_WRITE(ctx, ADDR_STACK_TO_BCAST(side), reg_addr, DATA(timer_val));
        HAL_Delay(2);  // small delay between writes
    }
}

void StackStartDeviceBalancing(DbmsCtx* ctx, uint8_t dev_addr)
{
    // this is the final trigger - balancing doesn't start until bal_ctrl2 is set
    //#define BAL_CTRL2_BAL_GO_MASK 0x02    
    SINGLE_DEV_WRITE(ctx, dev_addr, REG_BAL_CTRL2, DATA(BAL2_BAL_GO));
    HAL_Delay(8);
}

void StackStartBalancing(DbmsCtx* ctx, bool odds, int32_t bal_time)
{
    for(size_t side = 0; side < N_SIDES; ++side)
    {
        uint8_t dev_addr = ADDR_STACK_TO_BCAST(side);
        StackSetDeviceBalanceTimers(ctx, side, odds, bal_time);
        StackStartDeviceBalancing(ctx, dev_addr);
    }
}

void StackComputeCellsToBalance(DbmsCtx* ctx, bool odds, int32_t threshold_mv)
{
    // if any segment needs balancing - if this is false at the end we can skip balancing
    float balance_threshold = ctx->charging.pre_bal_min_v + threshold_mv;
    CanLog(ctx, "minv = %d maxv = %d dv = %d chbalth = %d\n", 
        (int)ctx->charging.pre_bal_min_v, 
        (int)ctx->charging.pre_bal_max_v, 
        (int)(ctx->charging.pre_bal_max_v-ctx->charging.pre_bal_min_v), 
        (int)balance_threshold);
    // if (ctx->stats.max_v > balance_threshold) return false;
    for (size_t side = 0; side < N_SIDES; side++)
    {
        for (size_t group = 0; group < N_GROUPS_PER_SIDE; group++)
        {
            float voltage = ctx->charging.pre_bal_average_v[side][group];
            ctx->cell_states[side].cells_to_balance[group] = voltage > balance_threshold && group % 2 == odds;
            CanLog(ctx, "%d ", (int) ctx->charging.pre_bal_average_v[side][group]);
        }   
        CanLog(ctx, "\n");
    }
}

bool StackNeedsToBalance(DbmsCtx* ctx, bool odds, int32_t threshold_mv)
{
    float balance_threshold = 1000 * ctx->stats.min_v + threshold_mv;
    for (size_t side = 0; side < N_SIDES; side++)
    {
        for (size_t group = 0; group < N_GROUPS_PER_SIDE; group++)
        {
            float voltage = ctx->charging.pre_bal_average_v[side][group];
            if(voltage > balance_threshold && group % 2 == odds) return true;
        }
    }
    return false;
}

void StackDumpCellsToBalance(DbmsCtx* ctx)
{
    for (size_t side_index = 0; side_index < N_SIDES; ++side_index)
    {
        CanLog(ctx, "Side ");
        // Convert side_index to string manually
        char side_str[8];
        side_str[0] = '0' + side_index;
        side_str[1] = ':';
        side_str[2] = ' ';
        side_str[3] = '\0';
        CanLog(ctx, side_str);
        
        bool found_any = false;
        for (size_t group = 0; group < N_GROUPS_PER_SIDE; ++group)
        {
            if (ctx->cell_states[side_index].cells_to_balance[group])
            {
                char cell_str[8];
                cell_str[0] = 'C';
                cell_str[1] = '0' + (group / 10);      // Tens digit
                cell_str[2] = '0' + (group % 10);      // Ones digit
                cell_str[3] = ' ';
                cell_str[4] = '\0';
                CanLog(ctx, cell_str);
                found_any = true;
            }
        }
        
        if (!found_any)
        {
            CanLog(ctx, "None");
        }
        
        CanLog(ctx, "\n");
    }
}

/*****************************
 *  ANALOG MUX CONTROL
 *****************************/

/**
 * @brief Set the active channel on the analog multiplexers
 * 
 * Each monitor chip has two 4:1 analog multiplexers (Mux A and Mux B) that expand
 * the number of thermistors that can be read. The mux select lines are controlled
 * via GPIO pins on the monitor ics.
 * 
 * GPIO Configuration (register 0x0E):
 * - Bits [1:0]: Mux A select (S0, S1)
 * - Bits [3:2]: Mux B select (S0, S1) 
 * - Bits [7:4]: Other GPIO config (kept constant)
 * 
 * @param ctx Context pointer
 * @param dev_number Monitor chip device address
 * @param channel Mux channel to select (0-3)
 * @return 0 on success, -1 on invalid channel
 */
int SetMuxChannels(DbmsCtx* ctx, uint8_t channel)
{
    uint8_t gpio_value;

    switch(channel)
    {
        case 0:
            gpio_value = 0x2D; // 0b00101101 - Channel 0: S1=0, S0=1 for both muxes
            break;
        case 1:
            gpio_value = 0x2C; // 0b00101100 - Channel 1: S1=0, S0=0 for both muxes
            break;
        case 2:
            gpio_value = 0x25; // 0b00100101 - Channel 2: S1=1, S0=1 for both muxes
            break;
        case 3:
            gpio_value = 0x24; // 0b00100100 - Channel 3: S1=1, S0=0 for both muxes
            break;
        default:
            return -1;
    }
    
    ctx->mux_selector = channel;
    return STACK_WRITE(ctx, 0x000F, DATA(gpio_value));
}
