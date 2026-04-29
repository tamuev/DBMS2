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
#include "can.h"
#include "context.h"

#define CAN_TX_QUEUE_SIZE 512

typedef struct
{
    uint32_t id;        // up to 29-bit CAN ID
    uint32_t mask;      // 29-bit or 11-bit mask
    bool extended;      // true = extended (29-bit)
} CanFilterMask;

typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
    //uint8_t prio ??
} CanTxQueueItem;

typedef struct {
    CanTxQueueItem buffer[CAN_TX_QUEUE_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
} CanTxQueue;

static CanTxQueue tx_queue = {0};

// cant change the interrupt function sigantures, store a global ctx ptr and set on init
static DbmsCtx* g_can_ctx = NULL;

/**
 * Configure CAN RX filters
 */
int ConfigCanFilters(CAN_HandleTypeDef *hcan, const CanFilterMask *filters, size_t count, const int base_bank)
{
    int status = HAL_OK, bank = 0;
    size_t i = 0;

    while (i < count)
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank = base_bank + bank;
        f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        f.FilterMode = CAN_FILTERMODE_IDMASK;
        f.FilterActivation = ENABLE;
        f.SlaveStartFilterBank = base_bank;

        // Check if this entry (or next) is extended -> use 32-bit mode
        if (filters[i].extended)
        {
            f.FilterScale = CAN_FILTERSCALE_32BIT;
            f.FilterIdHigh     = (filters[i].id   << 3) >> 16;
            f.FilterIdLow      = ((filters[i].id  << 3) & 0xFFFF) | (1 << 2);
            f.FilterMaskIdHigh = (filters[i].mask << 3) >> 16;
            f.FilterMaskIdLow  = ((filters[i].mask << 3) & 0xFFFF) | (1 << 2);

            status = HAL_CAN_ConfigFilter(hcan, &f);
            if (status != HAL_OK)
                return status;
            bank++;
            i++;
        }
        else
        {
            f.FilterScale = CAN_FILTERSCALE_16BIT;
            // First standard ID
            f.FilterIdHigh     = filters[i].id   << 5;
            f.FilterMaskIdHigh = filters[i].mask << 5;
            i++;

            // Second standard ID (if available and not extended)
            if (i < count && !filters[i].extended) {
                f.FilterIdLow      = filters[i].id   << 5;
                f.FilterMaskIdLow  = filters[i].mask << 5;
                i++;
            } else {
                f.FilterIdLow = 0;
                f.FilterMaskIdLow = 0;
            }

            status = HAL_CAN_ConfigFilter(hcan, &f);
            if (status != HAL_OK)
                return status;
            bank++;
        }
    }

    return HAL_OK;
}


int ConfigCan1(DbmsCtx* ctx)
{
    g_can_ctx = ctx;
    int status = 0;
    const int base_bank = 0;
    CanFilterMask masks[] =
    {
        { 0x0B0, 0x7F0, false },
        { 0x500, 0x700, false }
    };

    if ((status = ConfigCanFilters(ctx->hw.can1, masks, sizeof(masks)/sizeof(masks[0], base_bank))) != 0)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }

    // Start CAN
    if ((status = HAL_CAN_Start(ctx->hw.can1)) != HAL_OK)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }

    // Enable interrupts
    if ((status = HAL_CAN_ActivateNotification(ctx->hw.can1,
                                            CAN_IT_RX_FIFO0_MSG_PENDING |
                                            CAN_IT_RX_FIFO1_MSG_PENDING |
                                            CAN_IT_TX_MAILBOX_EMPTY)) != HAL_OK)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }

    // Configure TX header (example)
    ctx->hw.can1_tx_header.StdId = 0x500;
    ctx->hw.can1_tx_header.IDE = CAN_ID_STD;
    ctx->hw.can1_tx_header.RTR = CAN_RTR_DATA;
    ctx->hw.can1_tx_header.DLC = 8;
    ctx->hw.can1_tx_header.TransmitGlobalTime = DISABLE;

    return status;
}

int ConfigCan2(DbmsCtx* ctx)
{
    g_can_ctx = ctx;
    int status = 0;
    const int base_bank = 14;

    CanFilterMask charger_masks[] = {
        { CANID_ELCON_TX, 0x1FFFFFFF, true },
        { CANID_ELCON_RX, 0x1FFFFFFF,  true }
    }

    if ((status = ConfigCanFilters(ctx->hw.can1, charger_masks, sizeof(charger_masks)/sizeof(charger_masks[0], base_bank))) != 0)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }

    // Start CAN
    if ((status = HAL_CAN_Start(ctx->hw.can2)) != HAL_OK)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }

    // Enable interrupts
    if ((status = HAL_CAN_ActivateNotification(ctx->hw.can2,
                                            CAN_IT_RX_FIFO0_MSG_PENDING |
                                            CAN_IT_RX_FIFO1_MSG_PENDING |
                                            CAN_IT_TX_MAILBOX_EMPTY)) != HAL_OK)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }
}

static void SendFromQueue(CAN_HandleTypeDef *hcan)
{
    while (tx_queue.count > 0 && HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
    {
        CanTxQueueItem* entry = &tx_queue.buffer[tx_queue.tail];

        if (entry->header.StdId == CANID_TX_DELAY)
        {
            if (g_can_ctx)
                g_can_ctx->delay.T1 = GET_US2();
        }

        uint32_t mailbox;
        int32_t result = HAL_CAN_AddTxMessage(hcan, &entry->header, entry->data, &mailbox);

        if (result == HAL_OK)
        {
            tx_queue.tail = (tx_queue.tail + 1) % CAN_TX_QUEUE_SIZE;
            tx_queue.count--;

            if (g_can_ctx)
                g_can_ctx->stats.n_tx_can_frames++;
        }
        else
        {
            if (g_can_ctx)
            {
                g_can_ctx->stats.n_tx_can_fail++;
                g_can_ctx->last_can_err = HAL_CAN_GetError(hcan);
            }
            break;
        }
    }
}


int CanTransmit(DbmsCtx* ctx, uint32_t id, uint8_t data[8])
{
    CAN_TxHeaderTypeDef* hdr = &ctx->hw.can1_tx_header;

    // Determine if extended or standard ID
    if (id > CAN_STD_ID_MASK)
    {
        hdr->IDE = CAN_ID_EXT;
        hdr->ExtId = id & CAN_EXT_ID_MASK;
    }
    else
    {
        hdr->IDE = CAN_ID_STD;
        hdr->StdId = id & CAN_STD_ID_MASK;
    }

    hdr->RTR = CAN_RTR_DATA;
    hdr->DLC = 8;
    hdr->TransmitGlobalTime = DISABLE;

    __disable_irq();

    if (tx_queue.count == 0 && HAL_CAN_GetTxMailboxesFreeLevel(ctx->hw.can1) > 0U)
    {
        if (id == CANID_TX_DELAY)
        {
            ctx->delay.T1 = GET_US2();
        }
        int32_t result = HAL_CAN_AddTxMessage(ctx->hw.can1, hdr, data, &ctx->hw.can1_tx_mailbox);

        if (result != HAL_OK)
        {
            ctx->stats.n_tx_can_fail++;
            // ctx->led_state = LED_COMM_ERROR;
            ctx->last_can_err = HAL_CAN_GetError(ctx->hw.can1);
        }
        else
        {
            ctx->stats.n_tx_can_frames++;
            ctx->stats.last_can_tx_ts = GetUs(ctx);
        }
        __enable_irq();
        return result;
    }
    if (tx_queue.count >= CAN_TX_QUEUE_SIZE)
    {
        tx_queue.tail = (tx_queue.tail + 1) % CAN_TX_QUEUE_SIZE;
        tx_queue.count--;
        ctx->stats.n_tx_can_drop_queue_full++;
    }

    tx_queue.buffer[tx_queue.head].header = *hdr;
    memcpy(tx_queue.buffer[tx_queue.head].data, data, 8);
    tx_queue.head = (tx_queue.head + 1) % CAN_TX_QUEUE_SIZE;
    tx_queue.count++;
    ctx->stats.n_tx_queued = tx_queue.count;

    __enable_irq();

    SendFromQueue(ctx->hw.can1);

    return HAL_OK;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    SendFromQueue(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    SendFromQueue(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    SendFromQueue(hcan);
}
