/*
 * KalmanFilterMSD.c
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Kalman Filter — Mass-Spring-Damper (MSD)
 *  N = 2, M = 1
 * =============================================================================
 *  Only F depends on dt for MSD. Q is diagonal and does not contain dt terms,
 *  so only _build_F is needed when the sample period changes.
 * =============================================================================
 */

#include "KalmanFilterMSD.h"

/* =============================================================================
 *  Private — _build_F
 *  Rebuild state transition matrix F from stored physical parameters and dt.
 *
 *  F = [  1              dt      ]
 *      [ -(k/m)·dt   1-(c/m)·dt ]
 * =============================================================================
 */
static void _build_F(KalmanFilterMSD_t *kf)
{
    float m = kf->_m, c = kf->_c, k = kf->_k, dt = kf->_dt;
    kf->_F[0] =  1.0f;
    kf->_F[1] =  dt;
    kf->_F[2] = -(k / m) * dt;
    kf->_F[3] =  1.0f - (c / m) * dt;
}

/* =============================================================================
 *  KalmanFilterMSD_Init
 * =============================================================================
 */
void KalmanFilterMSD_Init(KalmanFilterMSD_t *kf,
                           float m, float c, float k)
{
    memset(kf, 0, sizeof(*kf));

    kf->_kf.x_data  = kf->_x;
    kf->_kf.P_data  = kf->_P;
    kf->_kf.F_data  = kf->_F;
    kf->_kf.Q_data  = kf->_Q;
    kf->_kf.H_data  = kf->_H;
    kf->_kf.R_data  = kf->_R;
    kf->_kf.scratch = kf->_sc;

    kf->_m = m;
    kf->_c = c;
    kf->_k = k;

    KalmanFilter_Init(&kf->_kf, 2, 1);

    /* H is fixed: measure position only */
    kf->_H[0] = 1.0f;  kf->_H[1] = 0.0f;
}

/* =============================================================================
 *  KalmanFilterMSD_Set_ObserverPeriod
 *  F is the only matrix that depends on dt for MSD.
 *  If Start() was already called, also refreshes the live CMSIS instance.
 * =============================================================================
 */
void KalmanFilterMSD_Set_ObserverPeriod(KalmanFilterMSD_t *kf, float dt)
{
    kf->_dt = dt;
    _build_F(kf);

    if (kf->_started) {
        arm_mat_init_f32(&kf->_kf.F, 2, 2, kf->_F);
    }
}

/* =============================================================================
 *  KalmanFilterMSD_Set_ProcessNoise
 * =============================================================================
 */
void KalmanFilterMSD_Set_ProcessNoise(KalmanFilterMSD_t *kf,
                                       float Q_Pos, float Q_Vel)
{
    kf->_Q[0] = Q_Pos;  kf->_Q[1] = 0.0f;
    kf->_Q[2] = 0.0f;   kf->_Q[3] = Q_Vel;

    if (kf->_started) {
        arm_mat_init_f32(&kf->_kf.Q, 2, 2, kf->_Q);
    }
}

/* =============================================================================
 *  KalmanFilterMSD_Set_MeasurementNoise
 * =============================================================================
 */
void KalmanFilterMSD_Set_MeasurementNoise(KalmanFilterMSD_t *kf, float R)
{
    kf->_R[0] = R;

    if (kf->_started) {
        arm_mat_init_f32(&kf->_kf.R, 1, 1, kf->_R);
    }
}

/* =============================================================================
 *  KalmanFilterMSD_Set_Covariance
 *  MSD state: [ position, velocity ]
 * =============================================================================
 */
void KalmanFilterMSD_Set_Covariance(KalmanFilterMSD_t *kf,
                                     float P_pos, float P_vel)
{
    kf->_p_pos = P_pos;
    kf->_p_vel = P_vel;
}

/* =============================================================================
 *  KalmanFilterMSD_Start
 * =============================================================================
 */
void KalmanFilterMSD_Start(KalmanFilterMSD_t *kf,
                            float initial_pos,
                            float initial_vel)
{
    kf->_x[0] = initial_pos;
    kf->_x[1] = initial_vel;

    float pp = kf->_p_pos;
    float pv = kf->_p_vel;
    kf->_P[0] = pp;    kf->_P[1] = 0.0f;
    kf->_P[2] = 0.0f;  kf->_P[3] = pv;

    arm_mat_init_f32(&kf->_kf.x, 2, 1, kf->_x);
    arm_mat_init_f32(&kf->_kf.P, 2, 2, kf->_P);
    arm_mat_init_f32(&kf->_kf.F, 2, 2, kf->_F);
    arm_mat_init_f32(&kf->_kf.Q, 2, 2, kf->_Q);
    arm_mat_init_f32(&kf->_kf.H, 1, 2, kf->_H);
    arm_mat_init_f32(&kf->_kf.R, 1, 1, kf->_R);

    kf->_started = 1;
}

/* =============================================================================
 *  KalmanFilterMSD_Update / Get_Position / Get_Velocity
 * =============================================================================
 */
void KalmanFilterMSD_Update(KalmanFilterMSD_t *kf, float z)
{
    KalmanFilter_Update(&kf->_kf, &z);
}

float KalmanFilterMSD_Get_Position(KalmanFilterMSD_t *kf) { return kf->_kf.x_data[0]; }
float KalmanFilterMSD_Get_Velocity(KalmanFilterMSD_t *kf) { return kf->_kf.x_data[1]; }
