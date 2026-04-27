/*
 * Ultrasonic.h
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Ultrasonic — HC-SR04 Distance Sensor Driver
 * =============================================================================
 *  Measures distance using a 10 µs trigger pulse and input-capture echo timing.
 *  Compatible with STM32 HAL (TIM input capture + DMA-free GPIO trigger).
 *
 *  Hardware Requirements
 *  ---------------------
 *    - One GPIO output pin   : trigger pulse
 *    - One timer (Base IT)   : fires the trigger at a fixed rate
 *    - One timer (Input Capture, 2 channels) : captures rise and fall edges
 *
 *  Init Sequence
 *  -------------
 *    Ultrasonic_Config_t cfg = {
 *        .Trig_Port     = GPIOC,
 *        .Trig_Pin      = GPIO_PIN_1,
 *        .Trigger_Timer = &htim6,
 *        .Echo_Timer    = &htim1,
 *        .Echo_Ch_Rise  = TIM_CHANNEL_1,
 *        .Echo_Ch_Fall  = TIM_CHANNEL_2,
 *    };
 *    Ultrasonic_Init(&Sensor, &cfg);
 *
 *  ISR Hooks
 *  ---------
 *    // In HAL_TIM_PeriodElapsedCallback:
 *    Ultrasonic_TimerElapsed_Handler(&Sensor, htim);
 *
 *    // In HAL_TIM_IC_CaptureCallback:
 *    Ultrasonic_IC_Capture_Handler(&Sensor, htim);
 *
 *  Reading the Result
 *  ------------------
 *    float distance = Sensor.Distance_mm;
 * =============================================================================
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#include "main.h"

/* =============================================================================
 *  Limits
 * =============================================================================
 */
#define ULTRASONIC_MAX_RANGE_MM   4000U   /* Readings beyond 4 m are clamped */

/* =============================================================================
 *  Ultrasonic_Config_t  —  Hardware Configuration
 *  Fill this struct, then pass it to Ultrasonic_Init().
 * =============================================================================
 */
typedef struct {
    GPIO_TypeDef      *Trig_Port;      /* Trigger GPIO port              */
    uint16_t           Trig_Pin;       /* Trigger GPIO pin               */
    TIM_HandleTypeDef *Trigger_Timer;  /* Timer that fires the trigger   */
    TIM_HandleTypeDef *Echo_Timer;     /* Timer used for input capture   */
    uint16_t           Echo_Ch_Rise;   /* IC channel — rising  edge      */
    uint16_t           Echo_Ch_Fall;   /* IC channel — falling edge      */
} Ultrasonic_Config_t;

/* =============================================================================
 *  Ultrasonic_t  —  Sensor Handle
 *  Read Distance_mm after each capture interrupt.
 * =============================================================================
 */
typedef struct {
    Ultrasonic_Config_t cfg;            /* Hardware config (copied in at Init) */
    uint32_t            Pulse_Width_us; /* Raw echo pulse width [µs]           */
    float               Distance_mm;   /* Measured distance [mm]              */
} Ultrasonic_t;

/* =============================================================================
 *  API
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 *  Ultrasonic_Init
 *  Initialise the sensor from a config struct.
 *  Drives the trigger pin low and starts all required timers.
 *
 *  Parameters
 *  ----------
 *  sensor  Pointer to Ultrasonic_t handle.
 *  config  Pointer to a fully filled Ultrasonic_Config_t.
 * ----------------------------------------------------------------------------- */
void Ultrasonic_Init(Ultrasonic_t *sensor, const Ultrasonic_Config_t *config);

/* -----------------------------------------------------------------------------
 *  Ultrasonic_Trigger
 *  Fire a 10 µs trigger pulse manually.
 *  Normally called automatically from Ultrasonic_TimerElapsed_Handler.
 *
 *  Parameters
 *  ----------
 *  sensor  Pointer to Ultrasonic_t handle.
 * ----------------------------------------------------------------------------- */
void Ultrasonic_Trigger(Ultrasonic_t *sensor);

/* -----------------------------------------------------------------------------
 *  Ultrasonic_TimerElapsed_Handler
 *  Place inside HAL_TIM_PeriodElapsedCallback().
 *  Fires a trigger pulse when the Trigger_Timer elapses.
 *
 *  Parameters
 *  ----------
 *  sensor  Pointer to Ultrasonic_t handle.
 *  htim    Timer handle from the callback — used to match the correct timer.
 * ----------------------------------------------------------------------------- */
void Ultrasonic_TimerElapsed_Handler(Ultrasonic_t *sensor, TIM_HandleTypeDef *htim);

/* -----------------------------------------------------------------------------
 *  Ultrasonic_IC_Capture_Handler
 *  Place inside HAL_TIM_IC_CaptureCallback().
 *  Computes the echo pulse width from rise/fall capture values
 *  and updates Distance_mm. Handles 16-bit timer overflow.
 *
 *  Parameters
 *  ----------
 *  sensor  Pointer to Ultrasonic_t handle.
 *  htim    Timer handle from the callback — used to match the correct timer.
 * ----------------------------------------------------------------------------- */
void Ultrasonic_IC_Capture_Handler(Ultrasonic_t *sensor, TIM_HandleTypeDef *htim);

#endif /* INC_ULTRASONIC_H_ */
