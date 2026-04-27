/*
 * KalmanFilterCAD.c
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Kalman Filter — Constant Acceleration Dynamics (CAD)
 *  N = 3, M = 1
 * =============================================================================
 *  F = [ 1   dt   dt²/2 ]
 *      [ 0    1    dt   ]
 *      [ 0    0     1   ]
 *
 *  Q = [ dt⁴/4   dt³/2   dt²/2 ]
 *      [ dt³/2   dt²     dt    ] × σ_a²
 *      [ dt²/2   dt      1     ]
 * =============================================================================
 */

#include "KalmanFilterCAD.h"

/* =============================================================================
 *  Private — _build_F
 *  Rebuild state transition matrix F from stored dt.
 * =============================================================================
 */
static void _build_F(KalmanFilterCAD_t *kf)
{
    float dt  = kf->_dt;
    float dt2 = dt * dt;
    kf->_F[0] = 1.0f;  kf->_F[1] = dt;    kf->_F[2] = dt2 / 2.0f;
    kf->_F[3] = 0.0f;  kf->_F[4] = 1.0f;  kf->_F[5] = dt;
    kf->_F[6] = 0.0f;  kf->_F[7] = 0.0f;  kf->_F[8] = 1.0f;
}

/* =============================================================================
 *  Private — _build_Q
 *  Rebuild process noise matrix Q from stored dt and var_accel.
 *
 *  Q = [ dt⁴/4   dt³/2   dt²/2 ]
 *      [ dt³/2   dt²     dt    ] × σ_a²
 *      [ dt²/2   dt      1     ]
 * =============================================================================
 */
static void _build_Q(KalmanFilterCAD_t *kf)
{
    float dt  = kf->_dt;
    float sa  = kf->_var_accel;
    float dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt;
    kf->_Q[0] = (dt4 / 4.0f) * sa;  kf->_Q[1] = (dt3 / 2.0f) * sa;  kf->_Q[2] = (dt2 / 2.0f) * sa;
    kf->_Q[3] = (dt3 / 2.0f) * sa;  kf->_Q[4] =  dt2          * sa;  kf->_Q[5] =  dt           * sa;
    kf->_Q[6] = (dt2 / 2.0f) * sa;  kf->_Q[7] =  dt           * sa;  kf->_Q[8] =  1.0f         * sa;
}

/* =============================================================================
 *  Private — _rewire_cmsis
 *  Re-attach CMSIS matrix instances after a data change post-Start().
 * =============================================================================
 */
static void _rewire_cmsis(KalmanFilterCAD_t *kf)
{
    arm_mat_init_f32(&kf->_kf.F, 3, 3, kf->_F);
    arm_mat_init_f32(&kf->_kf.Q, 3, 3, kf->_Q);
}

/* =============================================================================
 *  KalmanFilterCAD_Init
 * =============================================================================
 */
void KalmanFilterCAD_Init(KalmanFilterCAD_t *kf)
{
    memset(kf, 0, sizeof(*kf));

    kf->_kf.x_data  = kf->_x;
    kf->_kf.P_data  = kf->_P;
    kf->_kf.F_data  = kf->_F;
    kf->_kf.Q_data  = kf->_Q;
    kf->_kf.H_data  = kf->_H;
    kf->_kf.R_data  = kf->_R;
    kf->_kf.scratch = kf->_sc;

    KalmanFilter_Init(&kf->_kf, 3, 1);

    /* H is fixed: measure position only */
    kf->_H[0] = 1.0f;  kf->_H[1] = 0.0f;  kf->_H[2] = 0.0f;
}

/* =============================================================================
 *  KalmanFilterCAD_Set_ObserverPeriod
 *  Rebuilds F and Q. If Start() was already called, also refreshes the live
 *  CMSIS matrix instances so the engine sees the new values immediately.
 * =============================================================================
 */
void KalmanFilterCAD_Set_ObserverPeriod(KalmanFilterCAD_t *kf, float dt)
{
    kf->_dt = dt;
    _build_F(kf);
    _build_Q(kf);

    if (kf->_started) {
        _rewire_cmsis(kf);
    }
}

/* =============================================================================
 *  KalmanFilterCAD_Set_ProcessNoise
 * =============================================================================
 */
void KalmanFilterCAD_Set_ProcessNoise(KalmanFilterCAD_t *kf, float VarAccel)
{
    kf->_var_accel = VarAccel;
    _build_Q(kf);

    if (kf->_started) {
        arm_mat_init_f32(&kf->_kf.Q, 3, 3, kf->_Q);
    }
}

/* =============================================================================
 *  KalmanFilterCAD_Set_MeasurementNoise
 * =============================================================================
 */
void KalmanFilterCAD_Set_MeasurementNoise(KalmanFilterCAD_t *kf, float R)
{
    kf->_R[0] = R;

    if (kf->_started) {
        arm_mat_init_f32(&kf->_kf.R, 1, 1, kf->_R);
    }
}

/* =============================================================================
 *  KalmanFilterCAD_Set_Covariance
 *  CAD state: [ position, velocity, acceleration ]
 * =============================================================================
 */
void KalmanFilterCAD_Set_Covariance(KalmanFilterCAD_t *kf,
                                     float P_pos, float P_vel, float P_accel)
{
    kf->_p_pos   = P_pos;
    kf->_p_vel   = P_vel;
    kf->_p_accel = P_accel;
}

/* =============================================================================
 *  KalmanFilterCAD_Start
 * =============================================================================
 */
void KalmanFilterCAD_Start(KalmanFilterCAD_t *kf,
                            float initial_pos,
                            float initial_vel,
                            float initial_accel)
{
    kf->_x[0] = initial_pos;
    kf->_x[1] = initial_vel;
    kf->_x[2] = initial_accel;

    float pp = kf->_p_pos;
    float pv = kf->_p_vel;
    float pa = kf->_p_accel;
    kf->_P[0] = pp;    kf->_P[1] = 0.0f;  kf->_P[2] = 0.0f;
    kf->_P[3] = 0.0f;  kf->_P[4] = pv;    kf->_P[5] = 0.0f;
    kf->_P[6] = 0.0f;  kf->_P[7] = 0.0f;  kf->_P[8] = pa;

    arm_mat_init_f32(&kf->_kf.x, 3, 1, kf->_x);
    arm_mat_init_f32(&kf->_kf.P, 3, 3, kf->_P);
    arm_mat_init_f32(&kf->_kf.F, 3, 3, kf->_F);
    arm_mat_init_f32(&kf->_kf.Q, 3, 3, kf->_Q);
    arm_mat_init_f32(&kf->_kf.H, 1, 3, kf->_H);
    arm_mat_init_f32(&kf->_kf.R, 1, 1, kf->_R);

    kf->_started = 1;
}

/* =============================================================================
 *  KalmanFilterCAD_Update / Get_Position / Get_Velocity / Get_Accel
 * =============================================================================
 */
void KalmanFilterCAD_Update(KalmanFilterCAD_t *kf, float z)
{
    KalmanFilter_Update(&kf->_kf, &z);
}

float KalmanFilterCAD_Get_Position(KalmanFilterCAD_t *kf) { return kf->_kf.x_data[0]; }
float KalmanFilterCAD_Get_Velocity(KalmanFilterCAD_t *kf) { return kf->_kf.x_data[1]; }
float KalmanFilterCAD_Get_Accel   (KalmanFilterCAD_t *kf) { return kf->_kf.x_data[2]; }
