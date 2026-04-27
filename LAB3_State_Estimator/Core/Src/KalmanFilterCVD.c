/*
 * KalmanFilterCVD.c
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Kalman Filter — Constant Velocity Dynamics (CVD)
 *  N = 2, M = 1
 * =============================================================================
 */

#include "KalmanFilterCVD.h"

/* =============================================================================
 *  Private — _build_F
 *  Rebuild state transition matrix F from stored dt.
 *
 *  F = [ 1   dt ]
 *      [ 0    1 ]
 * =============================================================================
 */
static void _build_F(KalmanFilterCVD_t *kf)
{
    float dt = kf->_dt;
    kf->_F[0] = 1.0f;  kf->_F[1] = dt;
    kf->_F[2] = 0.0f;  kf->_F[3] = 1.0f;
}

/* =============================================================================
 *  Private — _build_Q
 *  Rebuild process noise matrix Q from stored dt and var_accel.
 *
 *  Q = [ dt⁴/4  dt³/2 ] × σ_a²
 *      [ dt³/2   dt²  ]
 * =============================================================================
 */
static void _build_Q(KalmanFilterCVD_t *kf)
{
    float dt  = kf->_dt;
    float sa  = kf->_var_accel;
    float dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt;
    kf->_Q[0] = (dt4 / 4.0f) * sa;
    kf->_Q[1] = (dt3 / 2.0f) * sa;
    kf->_Q[2] = (dt3 / 2.0f) * sa;
    kf->_Q[3] =  dt2          * sa;
}

/* =============================================================================
 *  Private — _rewire_cmsis
 *  Re-attach CMSIS matrix instances to the backing arrays after a data change.
 *  Called when Set_ObserverPeriod() is used after Start() has already run.
 * =============================================================================
 */
static void _rewire_cmsis(KalmanFilterCVD_t *kf)
{
    arm_mat_init_f32(&kf->_kf.F, 2, 2, kf->_F);
    arm_mat_init_f32(&kf->_kf.Q, 2, 2, kf->_Q);
}

/* =============================================================================
 *  KalmanFilterCVD_Init
 * =============================================================================
 */
void KalmanFilterCVD_Init(KalmanFilterCVD_t *kf)
{
    memset(kf, 0, sizeof(*kf));

    kf->_kf.x_data  = kf->_x;
    kf->_kf.P_data  = kf->_P;
    kf->_kf.F_data  = kf->_F;
    kf->_kf.Q_data  = kf->_Q;
    kf->_kf.H_data  = kf->_H;
    kf->_kf.R_data  = kf->_R;
    kf->_kf.scratch = kf->_sc;

    KalmanFilter_Init(&kf->_kf, 2, 1);

    /* H is fixed and never depends on dt */
    kf->_H[0] = 1.0f;  kf->_H[1] = 0.0f;
}

/* =============================================================================
 *  KalmanFilterCVD_Set_ObserverPeriod
 *  Rebuilds F and Q. If Start() was already called, also refreshes the live
 *  CMSIS matrix instances so the engine sees the new values immediately.
 * =============================================================================
 */
void KalmanFilterCVD_Set_ObserverPeriod(KalmanFilterCVD_t *kf, float dt)
{
    kf->_dt = dt;
    _build_F(kf);
    _build_Q(kf);   /* Q also uses dt — must rebuild both */

    if (kf->_started) {
        _rewire_cmsis(kf);
    }
}

/* =============================================================================
 *  KalmanFilterCVD_Set_ProcessNoise
 * =============================================================================
 */
void KalmanFilterCVD_Set_ProcessNoise(KalmanFilterCVD_t *kf, float VarAccel)
{
    kf->_var_accel = VarAccel;
    _build_Q(kf);

    if (kf->_started) {
        arm_mat_init_f32(&kf->_kf.Q, 2, 2, kf->_Q);
    }
}

/* =============================================================================
 *  KalmanFilterCVD_Set_MeasurementNoise
 * =============================================================================
 */
void KalmanFilterCVD_Set_MeasurementNoise(KalmanFilterCVD_t *kf, float R)
{
    kf->_R[0] = R;

    if (kf->_started) {
        arm_mat_init_f32(&kf->_kf.R, 1, 1, kf->_R);
    }
}

/* =============================================================================
 *  KalmanFilterCVD_Set_Covariance
 *  CVD state: [ position, velocity ] — two diagonal P elements.
 * =============================================================================
 */
void KalmanFilterCVD_Set_Covariance(KalmanFilterCVD_t *kf,
                                     float P_pos, float P_vel)
{
    kf->_p_pos = P_pos;
    kf->_p_vel = P_vel;
}

/* =============================================================================
 *  KalmanFilterCVD_Start
 * =============================================================================
 */
void KalmanFilterCVD_Start(KalmanFilterCVD_t *kf,
                            float initial_pos,
                            float initial_vel)
{
    kf->_x[0] = initial_pos;
    kf->_x[1] = initial_vel;

    float pp = kf->_p_pos;   /* set by Set_Covariance */
    float pv = kf->_p_vel;   /* set by Set_Covariance */
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
 *  KalmanFilterCVD_Update / KalmanFilterCVD_Get_Position / Get_Velocity
 * =============================================================================
 */
void KalmanFilterCVD_Update(KalmanFilterCVD_t *kf, float z)
{
    KalmanFilter_Update(&kf->_kf, &z);
}

float KalmanFilterCVD_Get_Position(KalmanFilterCVD_t *kf) { return kf->_kf.x_data[0]; }
float KalmanFilterCVD_Get_Velocity(KalmanFilterCVD_t *kf) { return kf->_kf.x_data[1]; }
