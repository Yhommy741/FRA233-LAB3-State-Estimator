/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* =============================================================================
 *  Mode Summary
 * =============================================================================
 *
 *  Mode 1  CD   — vary q (scalar),  fixed R = R_nom
 *  Mode 2  CVD  — vary σ_a,         fixed R = R_nom
 *  Mode 3  CAD  — vary σ_a,         fixed R = R_nom
 *
 *  Mode 4  MSD  — vary Q, fixed R = R_nom
 *  Mode 5  MSD  — vary R, fixed Q = Q_fix
 *
 *  ─── Q/R Ratio design ────────────────────────────────────────────────────
 *
 *  R_nom = 7.5e-7 m²   (from HC-SR04 resolution: R = Δ²/12, Δ = 3 mm)
 *  Q_fix = 1.0e-9 m²   (anchor Q for Mode 5)
 *
 *  Target Q/R ratios: { 1e-1, 1e-3, 1e-5 }
 *
 *  Mode 4 — fixed R = R_nom, vary Q to hit each ratio:
 *    KF1: Q = R_nom × 1e-1 = 7.5e-8   → Q/R = 1e-1
 *    KF2: Q = R_nom × 1e-3 = 7.5e-10  → Q/R = 1e-3
 *    KF3: Q = R_nom × 1e-5 = 7.5e-12  → Q/R = 1e-5
 *
 *  Mode 5 — fixed Q = Q_fix, vary R to hit each ratio:
 *    KF1: R = Q_fix / 1e-1 = 1.0e-8   → Q/R = 1e-1
 *    KF2: R = Q_fix / 1e-3 = 1.0e-6   → Q/R = 1e-3
 *    KF3: R = Q_fix / 1e-5 = 1.0e-4   → Q/R = 1e-5
 *
 *  => Mode 4 KF_n and Mode 5 KF_n share the same Q/R → output should be
 *     theoretically identical. Any difference reveals floating-point or
 *     numerical effects on the STM32.
 *
 *  Outputs (Modes 4 & 5):
 *    KF_Out_1/2/3 — filtered position  [m]
 *    KF_Vel_1/2/3 — estimated velocity [m/s]
 * =============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "Ultrasonic.h"
#include "SerialFrame.h"
#include "KalmanFilterCD.h"
#include "KalmanFilterCVD.h"
#include "KalmanFilterCAD.h"
#include "KalmanFilterMSD.h"
#include "Config.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define SAMPLE_DT   0.01f   /* 100 Hz            */
#define Gravity     9.81f   /* m/s²              */
/* USER CODE END PD */

/* USER CODE BEGIN PV */

/* ── Filter instances ────────────────────────────────────────────────── */
static KalmanFilterCD_t  KF_CD[3];
static KalmanFilterCVD_t KF_CVD[3];
static KalmanFilterCAD_t KF_CAD[3];
static KalmanFilterMSD_t KF_MSD_VQ[3];   /* Mode 4: vary Q, fixed R */
static KalmanFilterMSD_t KF_MSD_VR[3];   /* Mode 5: fixed Q, vary R */

/* ── Sensor & comms ──────────────────────────────────────────────────── */
static Ultrasonic_t  Sensor;
static SerialFrame_t Frame;

/* ── Application state ───────────────────────────────────────────────── */
static int16_t Mode         = 0;
static float   Calib_Height = 0.0f;
static float   Mass         = 0.3043f;    /* kg    */
static float   Damping      = 0.04093f;   /* Ns/m  */
static float   Spring_K     = 30.387f;    /* N/m   */

/* TX — position (all modes) */
static float Raw_Pos  = 0.0f;
static float KF_Out_1 = 0.0f;
static float KF_Out_2 = 0.0f;
static float KF_Out_3 = 0.0f;

/* TX — velocity (modes 4 & 5 only, zero otherwise) */
static float KF_Vel_1 = 0.0f;
static float KF_Vel_2 = 0.0f;
static float KF_Vel_3 = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Init(void);
static void Update(void);
/* USER CODE END PFP */

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM6_Init();
    MX_TIM3_Init();
    MX_LPUART1_UART_Init();
    /* USER CODE BEGIN 2 */
    Init();
    /* USER CODE END 2 */
    while (1) {}
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN            = 85;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

/* USER CODE BEGIN 4 */

/* ============================================================
 *  Init
 * ============================================================ */
static void Init(void)
{
    /* ── Ultrasonic ─────────────────────────────────────────── */
    Ultrasonic_Config_t s_cfg = {
        .Trig_Port     = ULTRASONIC_TRIG_PORT,
        .Trig_Pin      = ULTRASONIC_TRIG_PIN,
        .Trigger_Timer = ULTRASONIC_TRIGGER_HTIM,
        .Echo_Timer    = ULTRASONIC_ECHO_HTIM,
        .Echo_Ch_Rise  = ULTRASONIC_TIM_CHANNEL_1,
        .Echo_Ch_Fall  = ULTRASONIC_TIM_CHANNEL_2,
    };
    Ultrasonic_Init(&Sensor, &s_cfg);
    HAL_TIM_Base_Start_IT(OBSERVER_HTIM);

    /* ── Serial frame ───────────────────────────────────────── */
    SerialFrame_Init(&Frame, &hlpuart1, 0x44, 'Y');
    SerialFrame_Add_TX(&Frame, "Raw",   &Raw_Pos,  SF_FLOAT);
    SerialFrame_Add_TX(&Frame, "KF_1",  &KF_Out_1, SF_FLOAT);
    SerialFrame_Add_TX(&Frame, "KF_2",  &KF_Out_2, SF_FLOAT);
    SerialFrame_Add_TX(&Frame, "KF_3",  &KF_Out_3, SF_FLOAT);
    SerialFrame_Add_TX(&Frame, "Vel_1", &KF_Vel_1, SF_FLOAT);
    SerialFrame_Add_TX(&Frame, "Vel_2", &KF_Vel_2, SF_FLOAT);
    SerialFrame_Add_TX(&Frame, "Vel_3", &KF_Vel_3, SF_FLOAT);
    SerialFrame_Add_RX(&Frame, "Mass",     &Mass,         SF_FLOAT);
    SerialFrame_Add_RX(&Frame, "Damping",  &Damping,      SF_FLOAT);
    SerialFrame_Add_RX(&Frame, "Spring_K", &Spring_K,     SF_FLOAT);
    SerialFrame_Add_RX(&Frame, "H_calib",  &Calib_Height, SF_FLOAT);
    SerialFrame_Add_RX(&Frame, "Mode",     &Mode,         SF_INT16);
    SerialFrame_Receive(&Frame, &hlpuart1);

    /* ── Noise anchor ───────────────────────────────────────────────────
     *  R_nom = Δ²/12 = (3e-3)²/12 = 7.5e-7 m²  (HC-SR04, Δ = 3 mm)
     *  Q_fix = 1.0e-9 m²  (fixed Q used in Mode 5)
     * ──────────────────────────────────────────────────────────────── */
    const float R_nom = 7.5e-7f;
    const float Q_fix = 1.0e-9f;

    /* ── Target Q/R ratios (shared by Mode 4 & Mode 5) ─────────────────
     *  ratio[0] = 1e-1  (high  Q/R → responsive)
     *  ratio[1] = 1e-3  (mid   Q/R → balanced)
     *  ratio[2] = 1e-5  (low   Q/R → smooth)
     * ──────────────────────────────────────────────────────────────── */
    const float ratio[3] = { 1.0e-1f, 1.0e-3f, 1.0e-5f };

    /* ── Mode 1: KalmanFilterCD  (vary q, fixed R) ──────────── */
    const float cd_q[3] = { 7.5e-8f, 7.5e-9f, 7.5e-10f };
    for (int i = 0; i < 3; i++) {
        KalmanFilterCD_Init(&KF_CD[i]);
        KalmanFilterCD_Set_ProcessNoise(&KF_CD[i], cd_q[i]);
        KalmanFilterCD_Set_MeasurementNoise(&KF_CD[i], R_nom);
        KalmanFilterCD_Set_Covariance(&KF_CD[i], 1.0f);
        KalmanFilterCD_Start(&KF_CD[i], 0.0f);
    }

    /* ── Mode 2: KalmanFilterCVD  (vary σ_a, fixed R) ──────── */
    const float Var_acc[3] = { 1.0e-1f, 1.0e-3f, 1.0e-5f };
    for (int i = 0; i < 3; i++) {
        KalmanFilterCVD_Init(&KF_CVD[i]);
        KalmanFilterCVD_Set_ObserverPeriod(&KF_CVD[i], SAMPLE_DT);
        KalmanFilterCVD_Set_ProcessNoise(&KF_CVD[i], Var_acc[i]);
        KalmanFilterCVD_Set_MeasurementNoise(&KF_CVD[i], R_nom);
        KalmanFilterCVD_Set_Covariance(&KF_CVD[i], 1.0f, 1.0f);
        KalmanFilterCVD_Start(&KF_CVD[i], 0.0f, 0.0f);
    }

    /* ── Mode 3: KalmanFilterCAD  (vary σ_a, fixed R) ──────── */
    for (int i = 0; i < 3; i++) {
        KalmanFilterCAD_Init(&KF_CAD[i]);
        KalmanFilterCAD_Set_ObserverPeriod(&KF_CAD[i], SAMPLE_DT);
        KalmanFilterCAD_Set_ProcessNoise(&KF_CAD[i], Var_acc[i]);
        KalmanFilterCAD_Set_MeasurementNoise(&KF_CAD[i], R_nom);
        KalmanFilterCAD_Set_Covariance(&KF_CAD[i], 1.0f, 1.0f, 1.0f);
        KalmanFilterCAD_Start(&KF_CAD[i], 0.0f, 0.0f, 0.0f);
    }

    /* ── Mode 4: MSD — fixed R = R_nom, vary Q ──────────────────────────
     *  Q[i] = R_nom × ratio[i]  → same Q/R as Mode 5
     *
     *  KF1: Q = 7.5e-8,  R = 7.5e-7  → Q/R = 1e-1
     *  KF2: Q = 7.5e-10, R = 7.5e-7  → Q/R = 1e-3
     *  KF3: Q = 7.5e-12, R = 7.5e-7  → Q/R = 1e-5
     * ──────────────────────────────────────────────────────────────── */
    for (int i = 0; i < 3; i++) {
        float q = R_nom * ratio[i];   /* Q = R_nom × target ratio */
        KalmanFilterMSD_Init(&KF_MSD_VQ[i], Mass, Damping, Spring_K);
        KalmanFilterMSD_Set_ObserverPeriod(&KF_MSD_VQ[i], SAMPLE_DT);
        KalmanFilterMSD_Set_ProcessNoise(&KF_MSD_VQ[i], q, q);
        KalmanFilterMSD_Set_MeasurementNoise(&KF_MSD_VQ[i], R_nom);
        KalmanFilterMSD_Set_Covariance(&KF_MSD_VQ[i], 1.0f, 1.0f);
        KalmanFilterMSD_Start(&KF_MSD_VQ[i], 0.0f, 0.0f);
    }

    /* ── Mode 5: MSD — fixed Q = Q_fix, vary R ──────────────────────────
     *  R[i] = Q_fix / ratio[i]  → same Q/R as Mode 4
     *
     *  KF1: Q = 1e-9, R = 1e-8   → Q/R = 1e-1
     *  KF2: Q = 1e-9, R = 1e-6   → Q/R = 1e-3
     *  KF3: Q = 1e-9, R = 1e-4   → Q/R = 1e-5
     * ──────────────────────────────────────────────────────────────── */
    for (int i = 0; i < 3; i++) {
        float r = Q_fix / ratio[i];   /* R = Q_fix / target ratio */
        KalmanFilterMSD_Init(&KF_MSD_VR[i], Mass, Damping, Spring_K);
        KalmanFilterMSD_Set_ObserverPeriod(&KF_MSD_VR[i], SAMPLE_DT);
        KalmanFilterMSD_Set_ProcessNoise(&KF_MSD_VR[i], Q_fix, Q_fix);
        KalmanFilterMSD_Set_MeasurementNoise(&KF_MSD_VR[i], r);
        KalmanFilterMSD_Set_Covariance(&KF_MSD_VR[i], 1.0f, 1.0f);
        KalmanFilterMSD_Start(&KF_MSD_VR[i], 0.0f, 0.0f);
    }
}

/* ============================================================
 *  Update — 100 Hz
 * ============================================================ */
static void Update(void)
{
    Raw_Pos  = Calib_Height - (Sensor.Distance_mm * 0.001f);
    KF_Vel_1 = 0.0f;
    KF_Vel_2 = 0.0f;
    KF_Vel_3 = 0.0f;

    switch (Mode) {

        /* ── Mode 1: CD ───────────────────────────────────────── */
        case 1:
            KalmanFilterCD_Update(&KF_CD[0], Raw_Pos);
            KalmanFilterCD_Update(&KF_CD[1], Raw_Pos);
            KalmanFilterCD_Update(&KF_CD[2], Raw_Pos);
            KF_Out_1 = KalmanFilterCD_Get_State(&KF_CD[0]);
            KF_Out_2 = KalmanFilterCD_Get_State(&KF_CD[1]);
            KF_Out_3 = KalmanFilterCD_Get_State(&KF_CD[2]);
            break;

        /* ── Mode 2: CVD ──────────────────────────────────────── */
        case 2:
            KalmanFilterCVD_Update(&KF_CVD[0], Raw_Pos);
            KalmanFilterCVD_Update(&KF_CVD[1], Raw_Pos);
            KalmanFilterCVD_Update(&KF_CVD[2], Raw_Pos);
            KF_Out_1 = KalmanFilterCVD_Get_Position(&KF_CVD[0]);
            KF_Out_2 = KalmanFilterCVD_Get_Position(&KF_CVD[1]);
            KF_Out_3 = KalmanFilterCVD_Get_Position(&KF_CVD[2]);
            break;

        /* ── Mode 3: CAD ──────────────────────────────────────── */
        case 3:
            KalmanFilterCAD_Update(&KF_CAD[0], Raw_Pos);
            KalmanFilterCAD_Update(&KF_CAD[1], Raw_Pos);
            KalmanFilterCAD_Update(&KF_CAD[2], Raw_Pos);
            KF_Out_1 = KalmanFilterCAD_Get_Position(&KF_CAD[0]);
            KF_Out_2 = KalmanFilterCAD_Get_Position(&KF_CAD[1]);
            KF_Out_3 = KalmanFilterCAD_Get_Position(&KF_CAD[2]);
            break;

        /* ── Mode 4: MSD — fixed R, vary Q ───────────────────────
         *  KF1: Q/R = 1e-1  (responsive)
         *  KF2: Q/R = 1e-3  (balanced)
         *  KF3: Q/R = 1e-5  (smooth)
         * ───────────────────────────────────────────────────────── */
        case 4:
            KalmanFilterMSD_Update(&KF_MSD_VQ[0], Raw_Pos);
            KalmanFilterMSD_Update(&KF_MSD_VQ[1], Raw_Pos);
            KalmanFilterMSD_Update(&KF_MSD_VQ[2], Raw_Pos);
            KF_Out_1 = KalmanFilterMSD_Get_Position(&KF_MSD_VQ[0]);
            KF_Out_2 = KalmanFilterMSD_Get_Position(&KF_MSD_VQ[1]);
            KF_Out_3 = KalmanFilterMSD_Get_Position(&KF_MSD_VQ[2]);
            KF_Vel_1 = KalmanFilterMSD_Get_Velocity(&KF_MSD_VQ[0]);
            KF_Vel_2 = KalmanFilterMSD_Get_Velocity(&KF_MSD_VQ[1]);
            KF_Vel_3 = KalmanFilterMSD_Get_Velocity(&KF_MSD_VQ[2]);
            break;

        /* ── Mode 5: MSD — fixed Q, vary R ───────────────────────
         *  KF1: Q/R = 1e-1  (same ratio as Mode 4 KF1)
         *  KF2: Q/R = 1e-3  (same ratio as Mode 4 KF2)
         *  KF3: Q/R = 1e-5  (same ratio as Mode 4 KF3)
         * ───────────────────────────────────────────────────────── */
        case 5:
            KalmanFilterMSD_Update(&KF_MSD_VR[0], Raw_Pos);
            KalmanFilterMSD_Update(&KF_MSD_VR[1], Raw_Pos);
            KalmanFilterMSD_Update(&KF_MSD_VR[2], Raw_Pos);
            KF_Out_1 = KalmanFilterMSD_Get_Position(&KF_MSD_VR[0]);
            KF_Out_2 = KalmanFilterMSD_Get_Position(&KF_MSD_VR[1]);
            KF_Out_3 = KalmanFilterMSD_Get_Position(&KF_MSD_VR[2]);
            KF_Vel_1 = KalmanFilterMSD_Get_Velocity(&KF_MSD_VR[0]);
            KF_Vel_2 = KalmanFilterMSD_Get_Velocity(&KF_MSD_VR[1]);
            KF_Vel_3 = KalmanFilterMSD_Get_Velocity(&KF_MSD_VR[2]);
            break;

        default: break;
    }

    SerialFrame_Transmit(&Frame);
}

/* ============================================================
 *  Interrupt handlers
 * ============================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    Ultrasonic_TimerElapsed_Handler(&Sensor, htim);
    if (htim == OBSERVER_HTIM) Update();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    SerialFrame_Receive(&Frame, huart);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    Ultrasonic_IC_Capture_Handler(&Sensor, htim);
}

/* USER CODE END 4 */

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
