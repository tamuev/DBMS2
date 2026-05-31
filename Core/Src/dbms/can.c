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


// CAN1 = charger bus (500k), no queue, ELCON only
int ConfigElconCan(DbmsCtx* ctx)
{
    g_can_ctx = ctx;
    int status = 0;
    const int base_bank = 0;

    CanFilterMask charger_masks[] =
    {
        { CANID_ELCON_TX, 0x1FFFFFFF, true },
        { CANID_ELCON_RX, 0x1FFFFFFF, true }
    };

    if ((status = ConfigCanFilters(ctx->hw.elcon_can, charger_masks,
                                   sizeof(charger_masks)/sizeof(charger_masks[0]),
                                   base_bank)) != 0)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }

    if ((status = HAL_CAN_Start(ctx->hw.elcon_can)) != HAL_OK)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }

    if ((status = HAL_CAN_ActivateNotification(ctx->hw.elcon_can,
                                            CAN_IT_RX_FIFO0_MSG_PENDING)) != HAL_OK)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }

    return status;
}

// CAN2 = vehicle bus (1M), queued TX
int ConfigCan(DbmsCtx* ctx)
{
    g_can_ctx = ctx;
    int status = 0;
    const int base_bank = 14;

    CanFilterMask masks[] =
    {
        { 0x0B0, 0x7F0, false },
        { 0x500, 0x700, false },
        { 0x624, 0x7FE, false }, // Plex distance (0x624) + set distance (0x625)
        { 0x0B000000, 0x1FFF0000, true}
    };

    if ((status = ConfigCanFilters(ctx->hw.can, masks,
                                   sizeof(masks)/sizeof(masks[0]),
                                   base_bank)) != 0)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }

    if ((status = HAL_CAN_Start(ctx->hw.can)) != HAL_OK)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }

    if ((status = HAL_CAN_ActivateNotification(ctx->hw.can,
                                            CAN_IT_RX_FIFO0_MSG_PENDING |
                                            CAN_IT_RX_FIFO1_MSG_PENDING |
                                            CAN_IT_TX_MAILBOX_EMPTY |
                                            CAN_IT_ERROR |
                                            CAN_IT_BUSOFF)) != HAL_OK)
    {
        ctx->led_state = LED_FIRMWARE_FAULT;
        return status;
    }

    ctx->hw.can_tx_header.StdId = 0x500;
    ctx->hw.can_tx_header.IDE = CAN_ID_STD;
    ctx->hw.can_tx_header.RTR = CAN_RTR_DATA;
    ctx->hw.can_tx_header.DLC = 8;
    ctx->hw.can_tx_header.TransmitGlobalTime = DISABLE;

    return status;
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
    CAN_TxHeaderTypeDef* hdr = &ctx->hw.can_tx_header;

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

    if (tx_queue.count == 0 && HAL_CAN_GetTxMailboxesFreeLevel(ctx->hw.can) > 0U)
    {
        if (id == CANID_TX_DELAY)
        {
            ctx->delay.T1 = GET_US2();
        }
        int32_t result = HAL_CAN_AddTxMessage(ctx->hw.can, hdr, data, &ctx->hw.can_tx_mailbox);

        if (result != HAL_OK)
        {
            ctx->stats.n_tx_can_fail++;
            ctx->last_can_err = HAL_CAN_GetError(ctx->hw.can);
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

    SendFromQueue(ctx->hw.can);

    return HAL_OK;
}

// charger TX on CAN1, no queue
int CanChargeTransmit(DbmsCtx* ctx, uint32_t id, uint8_t data[8])
{
    CAN_TxHeaderTypeDef hdr = {0};

    if (id > CAN_STD_ID_MASK)
    {
        hdr.IDE = CAN_ID_EXT;
        hdr.ExtId = id & CAN_EXT_ID_MASK;
    }
    else
    {
        hdr.IDE = CAN_ID_STD;
        hdr.StdId = id & CAN_STD_ID_MASK;
    }
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = 8;
    hdr.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(ctx->hw.elcon_can) == 0U)
    {
        ctx->stats.n_tx_can_fail++;
        ctx->last_can_err = HAL_CAN_GetError(ctx->hw.elcon_can);
        return HAL_BUSY;
    }

    uint32_t mailbox;
    int32_t result = HAL_CAN_AddTxMessage(ctx->hw.elcon_can, &hdr, data, &mailbox);

    if (result != HAL_OK)
    {
        ctx->stats.n_tx_can_fail++;
        ctx->last_can_err = HAL_CAN_GetError(ctx->hw.elcon_can);
        return result;
    }

    ctx->stats.n_tx_can_frames++;
    ctx->stats.last_can_tx_ts = GetUs(ctx);
    return HAL_OK;
}

// queue lives on CAN2 (vehicle bus); ignore CAN1 (charger) mailbox completions
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN2) SendFromQueue(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN2) SendFromQueue(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN2) SendFromQueue(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN2) return;
    if (hcan->ErrorCode & HAL_CAN_ERROR_BOF) {
        uint64_t t = GetUs(g_can_ctx);
        if (g_can_ctx->stats.last_can_bo_ts == 0) {
            g_can_ctx->stats.last_can_bo_ts = t;
            g_can_ctx->stats.n_can_bussoffs++;
        }
        g_can_ctx->stats.last_can_bad_ts = t;
    }
}
