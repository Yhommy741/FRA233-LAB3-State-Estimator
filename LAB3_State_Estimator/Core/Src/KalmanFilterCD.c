/*
 * KalmanFilterCD.c
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Kalman Filter — Constant Dynamics (CD)
 *  N = 1, M = 1
 * =============================================================================
 *  F = [1]   H = [1]   Q = [q]   R = [r]
 *  Initial P = stored _p  (must be set via Set_Covariance before Start)
 * =============================================================================
 */

#include "KalmanFilterCD.h"

/* =============================================================================
 *  KalmanFilterCD_Init
 * =============================================================================
 */
void KalmanFilterCD_Init(KalmanFilterCD_t *kf)
{
    memset(kf, 0, sizeof(*kf));

    /* Wire backing arrays into the engine handle */
    kf->_kf.x_data  = kf->_x;
    kf->_kf.P_data  = kf->_P;
    kf->_kf.F_data  = kf->_F;
    kf->_kf.Q_data  = kf->_Q;
    kf->_kf.H_data  = kf->_H;
    kf->_kf.R_data  = kf->_R;
    kf->_kf.scratch = kf->_sc;

    KalmanFilter_Init(&kf->_kf, 1, 1);

    /* Fixed matrices for the CD model */
    kf->_F[0] = 1.0f;   /* F = [1] */
    kf->_H[0] = 1.0f;   /* H = [1] */
}

/* =============================================================================
 *  KalmanFilterCD_Set_ProcessNoise
 * =============================================================================
 */
void KalmanFilterCD_Set_ProcessNoise(KalmanFilterCD_t *kf, float Q)
{
    kf->_q    = Q;
    kf->_Q[0] = Q;
}

/* =============================================================================
 *  KalmanFilterCD_Set_MeasurementNoise
 * =============================================================================
 */
void KalmanFilterCD_Set_MeasurementNoise(KalmanFilterCD_t *kf, float R)
{
    kf->_r    = R;
    kf->_R[0] = R;
}

/* =============================================================================
 *  KalmanFilterCD_Set_Covariance
 *  CD state: [ value ]  — one scalar P value.
 * =============================================================================
 */
void KalmanFilterCD_Set_Covariance(KalmanFilterCD_t *kf, float P_val)
{
    kf->_p = P_val;
}

/* =============================================================================
 *  KalmanFilterCD_Start
 * =============================================================================
 */
void KalmanFilterCD_Start(KalmanFilterCD_t *kf, float initial_state)
{
    kf->_x[0] = initial_state;
    kf->_P[0] = kf->_p;

    /* Attach CMSIS matrix instances to backing arrays */
    arm_mat_init_f32(&kf->_kf.x, 1, 1, kf->_x);
    arm_mat_init_f32(&kf->_kf.P, 1, 1, kf->_P);
    arm_mat_init_f32(&kf->_kf.F, 1, 1, kf->_F);
    arm_mat_init_f32(&kf->_kf.Q, 1, 1, kf->_Q);
    arm_mat_init_f32(&kf->_kf.H, 1, 1, kf->_H);
    arm_mat_init_f32(&kf->_kf.R, 1, 1, kf->_R);
}

/* =============================================================================
 *  KalmanFilterCD_Update / KalmanFilterCD_Get_State
 * =============================================================================
 */
void KalmanFilterCD_Update(KalmanFilterCD_t *kf, float z)
{
    KalmanFilter_Update(&kf->_kf, &z);
}

float KalmanFilterCD_Get_State(KalmanFilterCD_t *kf)
{
    return kf->_kf.x_data[0];
}
