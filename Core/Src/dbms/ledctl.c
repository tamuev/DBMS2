/** 
 * 
 * Distributed BMS      LED Controller
 *
 * Copyright (C) 2025   Texas A&M University
 * 
 *                      Justus Languell  <justus@tamu.edu>
 *                      Cam Stone        <cameron28202@tamu.edu>
 *                      Abhinav Akavaram <abhinav.akavaram@tamu.edu>
 *                      Eli Nicksic      <eli.n@tamu.edu>
 */
#include "ledctl.h"

#define FAST_BLINK_INTERVAL 100
#define SLOW_BLINK_INTERVAL 500
#define NUM_LEDS (sizeof(leds) / sizeof(LedMapping))

/**
 *  LedMapping struct maps a logical LED to its physical hardware pins.
 *  contains the GPIO port and pin number for both the red and green components of a single LED
 */
typedef struct _LedMapping
{
    GPIO_TypeDef* rx;
    uint16_t rp;
    GPIO_TypeDef* gx;
    uint16_t gp;
} LedMapping;

/**
 *  Enum for blink types
 */
typedef enum _BlinkType
{
    BLINK_TYPE_OFF = 0,
    BLINK_TYPE_FAST,
    BLINK_TYPE_SLOW,
    BLINK_TYPE_COUNT
} BlinkType;

typedef struct _InternalLedState
{
    LedColor color;
    BlinkType blink_type;
    bool locking; // for future
} InternalLedState;

/**
 *  Array of LED mappings for each LED
 *  LED 0: Red on PC6, Green on PC7
 *  LED 1: Red on PC8, Green on PC9
 *  LED 2: Red on PA8, Green on PA9
 */
static const LedMapping leds[] = {{GPIOC, GPIO_PIN_6, GPIOC, GPIO_PIN_7},
                                  {GPIOC, GPIO_PIN_8, GPIOC, GPIO_PIN_9},
                                  {GPIOA, GPIO_PIN_8, GPIOA, GPIO_PIN_9}};

/**
 *  Table mapping each system state to per-LED states
 */
static const InternalLedState system_led_patterns[LED_STATE_COUNT][NUM_LEDS] = {
        [LED_FIRMWARE_FAULT] = {{LED_RED, BLINK_TYPE_OFF, false},
                       {LED_RED, BLINK_TYPE_OFF, false},
                       {LED_RED, BLINK_TYPE_OFF, false}},
        [LED_IDLE] = {{LED_YELLOW, BLINK_TYPE_FAST, false},
                       {LED_YELLOW, BLINK_TYPE_OFF, false},
                       {LED_YELLOW, BLINK_TYPE_OFF, false}},
        [LED_IDLE_FAULT] = {{LED_RED, BLINK_TYPE_FAST, false},
                       {LED_YELLOW, BLINK_TYPE_OFF, false},
                       {LED_YELLOW, BLINK_TYPE_OFF, false}},
        [LED_ACTIVE] = {{LED_GREEN, BLINK_TYPE_FAST, false},
                       {LED_GREEN, BLINK_TYPE_OFF, false},
                       {LED_GREEN, BLINK_TYPE_OFF, false}},
        [LED_ACTIVE_WARNING] = {{LED_YELLOW, BLINK_TYPE_FAST, false},
                       {LED_GREEN, BLINK_TYPE_OFF, false},
                       {LED_GREEN, BLINK_TYPE_OFF, false}},
        [LED_ACTIVE_FAULT] = {{LED_RED, BLINK_TYPE_FAST, false},
                       {LED_GREEN, BLINK_TYPE_OFF, false},
                       {LED_GREEN, BLINK_TYPE_OFF, false}},
        [LED_INIT] = {{LED_YELLOW, BLINK_TYPE_OFF, false},
                       {LED_GREEN, BLINK_TYPE_OFF, false},
                       {LED_GREEN, BLINK_TYPE_OFF, false}},
        [LED_CHARGING]= {{LED_GREEN, BLINK_TYPE_FAST, false},
                       {LED_GREEN, BLINK_TYPE_FAST, false},
                       {LED_GREEN, BLINK_TYPE_FAST, false}},
        [LED_BALANCING_ODDS]= {{LED_YELLOW, BLINK_TYPE_FAST, false},
                       {LED_YELLOW, BLINK_TYPE_FAST, false},
                       {LED_YELLOW, BLINK_TYPE_OFF, false}},
        [LED_BALANCING_EVENS] = {{LED_YELLOW, BLINK_TYPE_FAST, false},
                       {LED_YELLOW, BLINK_TYPE_OFF, false},
                       {LED_YELLOW, BLINK_TYPE_FAST, false}},
        [LED_CHARGING_COMPLETE]= {{LED_GREEN, BLINK_TYPE_FAST, false},
                       {LED_GREEN, BLINK_TYPE_FAST, false},
                       {LED_GREEN, BLINK_TYPE_OFF, false}},
        [LED_CHARGING_WAIT] = {{LED_YELLOW, BLINK_TYPE_FAST, false},
                {LED_YELLOW, BLINK_TYPE_FAST, false},
                {LED_YELLOW, BLINK_TYPE_FAST, false}}
};
static InternalLedState internal_led_states[NUM_LEDS];

void LedController_Init(void)
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        internal_led_states[i].color = LED_OFF;
        internal_led_states[i].blink_type = BLINK_TYPE_OFF;
        internal_led_states[i].locking = false;
    }
}

/**
 *  Function the dbms iter loop will actually call
 *  Handles blinking and non-blinking states
 */
void ProcessLedAction(DbmsCtx* ctx)
{
    static uint32_t last_blink_time = 0;
    static uint8_t blink_on = 0;
    uint32_t now = HAL_GetTick();

    uint8_t blink_needed = 0;
    uint32_t blink_interval = 0;

    // determine if any LED needs blinking, and set the interval
    for (int i = 0; i < NUM_LEDS; ++i)
    {
        InternalLedState state = system_led_patterns[ctx->led_state][i];
        if (state.blink_type != BLINK_TYPE_OFF)
        {
            blink_needed = 1;
            blink_interval = state.blink_type == BLINK_TYPE_FAST ? FAST_BLINK_INTERVAL : SLOW_BLINK_INTERVAL;
        }
    }

    // calculate if we need a blink now or not
    // if so, toggle the blink state and update the last blink time
    if (blink_needed && (now - last_blink_time >= blink_interval))
    {
        blink_on = !blink_on;
        last_blink_time = now;
    }

    // set each LED according to its state and blink logic
    for (int i = 0; i < NUM_LEDS; ++i)
    {
        InternalLedState state = system_led_patterns[ctx->led_state][i];
        LedColor color = state.color;

        // if this is a blinking state and we're in the "off" part of the cycle, turn off the LED
        if (state.blink_type != BLINK_TYPE_OFF && !blink_on)
        {
            color = LED_OFF;
        }
        LedSet((Led)i, color);
    }
}

/**
 *  Helper function to set the individual LED color using HAL
 */
void LedSet(Led led, LedColor color)
{
    HAL_GPIO_WritePin(leds[led].rx, leds[led].rp, color & 0b01);
    HAL_GPIO_WritePin(leds[led].gx, leds[led].gp, color & 0b10);
}
