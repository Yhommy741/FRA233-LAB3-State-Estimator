/*
 * Ultrasonic.c
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Ultrasonic — HC-SR04 Distance Sensor Driver
 * =============================================================================
 *  Distance formula:
 *    Speed of sound ≈ 340 m/s → 0.034 mm/µs (one way)
 *    Round-trip → divide by 2
 *    Simplified: Distance_mm = pulse_us × 10 / 58
 * =============================================================================
 */

#include "Ultrasonic.h"

/* =============================================================================
 *  Private — _pulse_to_mm
 *  Convert raw echo pulse width (µs) to distance in millimetres.
 * =============================================================================
 */
static inline float _pulse_to_mm(uint32_t pulse_us)
{
    return (float)pulse_us * 10.0f / 58.0f;
}

/* =============================================================================
 *  Ultrasonic_Init
 * =============================================================================
 */
void Ultrasonic_Init(Ultrasonic_t *sensor, const Ultrasonic_Config_t *config)
{
    /* Copy config into handle so the caller does not need to keep it alive */
    sensor->cfg = *config;

    /* Drive trigger low before starting timers */
    HAL_GPIO_WritePin(sensor->cfg.Trig_Port, sensor->cfg.Trig_Pin, GPIO_PIN_RESET);

    /* Start trigger timer — fires periodic interrupt at the desired rate */
    HAL_TIM_Base_Start_IT(sensor->cfg.Trigger_Timer);

    /* Start echo timer base and both input-capture channels */
    HAL_TIM_Base_Start_IT(sensor->cfg.Echo_Timer);
    HAL_TIM_IC_Start    (sensor->cfg.Echo_Timer, sensor->cfg.Echo_Ch_Rise);
    HAL_TIM_IC_Start_IT (sensor->cfg.Echo_Timer, sensor->cfg.Echo_Ch_Fall);
}

/* =============================================================================
 *  Ultrasonic_Trigger
 *  Drive trigger HIGH for 10 µs then LOW using a spin-wait on the echo timer.
 * =============================================================================
 */
void Ultrasonic_Trigger(Ultrasonic_t *sensor)
{
    /* Set HIGH */
    HAL_GPIO_WritePin(sensor->cfg.Trig_Port, sensor->cfg.Trig_Pin, GPIO_PIN_SET);

    /* Spin-wait 10 µs using the echo timer counter */
    uint32_t t0 = __HAL_TIM_GET_COUNTER(sensor->cfg.Echo_Timer);
    while ((__HAL_TIM_GET_COUNTER(sensor->cfg.Echo_Timer) - t0) < 10U);

    /* Set LOW */
    HAL_GPIO_WritePin(sensor->cfg.Trig_Port, sensor->cfg.Trig_Pin, GPIO_PIN_RESET);
}

/* =============================================================================
 *  Ultrasonic_TimerElapsed_Handler
 *  Call from HAL_TIM_PeriodElapsedCallback().
 * =============================================================================
 */
void Ultrasonic_TimerElapsed_Handler(Ultrasonic_t *sensor, TIM_HandleTypeDef *htim)
{
    if (htim == sensor->cfg.Trigger_Timer) {
        Ultrasonic_Trigger(sensor);
    }
}

/* =============================================================================
 *  Ultrasonic_IC_Capture_Handler
 *  Call from HAL_TIM_IC_CaptureCallback().
 *  Reads both captured edges, computes pulse width (with overflow handling),
 *  converts to mm and clamps to ULTRASONIC_MAX_RANGE_MM.
 * =============================================================================
 */
void Ultrasonic_IC_Capture_Handler(Ultrasonic_t *sensor, TIM_HandleTypeDef *htim)
{
    if (htim != sensor->cfg.Echo_Timer) return;

    uint16_t rise = (uint16_t)HAL_TIM_ReadCapturedValue(htim, sensor->cfg.Echo_Ch_Rise);
    uint16_t fall = (uint16_t)HAL_TIM_ReadCapturedValue(htim, sensor->cfg.Echo_Ch_Fall);

    /* Handle 16-bit timer overflow between rise and fall captures */
    sensor->Pulse_Width_us = (fall >= rise)
        ? (uint32_t)(fall - rise)
        : (uint32_t)(65535U - rise + fall + 1U);

    float dist = _pulse_to_mm(sensor->Pulse_Width_us);

    /* Clamp to maximum rated range */
    sensor->Distance_mm = (dist > (float)ULTRASONIC_MAX_RANGE_MM)
        ? (float)ULTRASONIC_MAX_RANGE_MM
        : dist;
}
