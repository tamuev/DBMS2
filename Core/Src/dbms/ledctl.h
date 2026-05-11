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
#ifndef _LEDCTL_H_
#define _LEDCTL_H_

#include "utils/common.h"
#include "context.h"

/**
 *  Enum for system LED states (all 3 LEDs)
 *  these combos are mapped in implementation file's system_led_patterns
 *
 *  (fwd declared)
 */
enum _LedState
{
    LED_FIRMWARE_FAULT = 0,
    LED_IDLE,
    LED_IDLE_FAULT,
    LED_ACTIVE,
    LED_ACTIVE_WARNING,
    LED_ACTIVE_FAULT,
    LED_INIT,
    LED_CHARGING,
    LED_BALANCING_ODDS,
    LED_BALANCING_EVENS,
    LED_CHARGING_COMPLETE,
    LED_CHARGING_WAIT,
    // ...
    LED_STATE_COUNT
};

/**
 *  Enum for actual LED colors
 */
typedef enum _LedColor
{
    LED_OFF = 0,
    LED_RED,
    LED_GREEN,
    LED_YELLOW,
} LedColor;

/**
 *  Enum for individual LEDs, used to index
 */
typedef enum _Led
{
    LED6 = 0,
    LED7,
    LED8
} Led;

void ProcessLedAction(DbmsCtx* ctx);

void LedSet(Led led, LedColor color);

#endif
