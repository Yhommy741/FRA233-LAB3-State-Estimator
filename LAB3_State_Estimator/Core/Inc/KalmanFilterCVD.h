/*
 * KalmanFilterCVD.h
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Kalman Filter — Constant Velocity Dynamics (CVD)
 * =============================================================================
 *
 *  State vector
 *  ------------
 *    x = [ position ]    (N = 2)
 *        [ velocity ]
 *    z = [ position ]    (M = 1)
 *
 *  Matrices
 *  --------
 *    F = [ 1   dt ]     H = [ 1  0 ]
 *        [ 0    1 ]
 *
 *    Q is built automatically from VarAccel (σ_a²):
 *      Q = [ dt⁴/4  dt³/2 ] × σ_a²
 *          [ dt³/2   dt²  ]
 *
 *    VarAccel — process noise : raise if the system accelerates rapidly
 *    R        — sensor noise  : raise for smoother position (more lag)
 *    P_pos    — initial position covariance (must be set via Set_Covariance)
 *    P_vel    — initial velocity covariance (must be set via Set_Covariance)
 *
 *  Init Sequence
 *  -------------
 *    KalmanFilterCVD_Init(&kf);
 *    KalmanFilterCVD_Set_ObserverPeriod(&kf, dt);
 *    KalmanFilterCVD_Set_ProcessNoise(&kf, VarAccel);
 *    KalmanFilterCVD_Set_MeasurementNoise(&kf, R);
 *    KalmanFilterCVD_Set_Covariance(&kf, P_pos, P_vel);
 *    KalmanFilterCVD_Start(&kf, initial_pos, initial_vel);
 *
 *  Interrupt Sequence
 *  -------------
 *    KalmanFilterCVD_Update(&kf, raw_z);
 *    float pos = KalmanFilterCVD_Get_Position(&kf);
 *    float vel = KalmanFilterCVD_Get_Velocity(&kf);
 * =============================================================================
 */

#ifndef INC_KALMANFILTERCVD_H_
#define INC_KALMANFILTERCVD_H_

#include "KalmanFilter.h"

/* =============================================================================
 *  KalmanFilterCVD_t
 * =============================================================================
 */
typedef struct {
    KalmanFilter_t  _kf;

    /* Backing arrays (N=2, M=1) */
    float _x[2];
    float _P[4];
    float _F[4];
    float _Q[4];
    float _H[2];
    float _R[1];
    float _sc[KF_SCRATCH_SIZE];
    float   _dt;
    float   _var_accel;
    float   _p_pos;
    float   _p_vel;
    uint8_t _started;

} KalmanFilterCVD_t;

/* =============================================================================
 *  API
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 *  Step 1 — KalmanFilterCVD_Init
 *  Zero handle and wire internal memory pointers.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCVD_Init(KalmanFilterCVD_t *kf);

/* -----------------------------------------------------------------------------
 *  Step 2 — KalmanFilterCVD_Set_ObserverPeriod
 *  Set the observer (sample) period.
 *
 *  Parameters
 *  ----------
 *  dt  Sample period in seconds.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCVD_Set_ObserverPeriod(KalmanFilterCVD_t *kf, float dt);

/* -----------------------------------------------------------------------------
 *  Step 3 — KalmanFilterCVD_Set_ProcessNoise
 *  Set process noise from acceleration variance. Rebuilds Q automatically.
 *
 *  Parameters
 *  ----------
 *  VarAccel  Expected acceleration noise variance (m/s²)².
 * ----------------------------------------------------------------------------- */
void KalmanFilterCVD_Set_ProcessNoise(KalmanFilterCVD_t *kf, float VarAccel);

/* -----------------------------------------------------------------------------
 *  Step 4 — KalmanFilterCVD_Set_MeasurementNoise
 *  Set sensor noise variance R.
 *
 *  Parameters
 *  ----------
 *  R  Sensor noise variance.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCVD_Set_MeasurementNoise(KalmanFilterCVD_t *kf, float R);

/* -----------------------------------------------------------------------------
 *  Step 5 — KalmanFilterCVD_Set_Covariance
 *  Set initial estimate covariance P (diagonal).
 *
 *  CVD state: [ position, velocity ]
 *
 *  Parameters
 *  ----------
 *  P_pos  Initial position variance P[0,0].
 *  P_vel  Initial velocity variance P[1,1].
 * ----------------------------------------------------------------------------- */
void KalmanFilterCVD_Set_Covariance(KalmanFilterCVD_t *kf,
                                     float P_pos, float P_vel);

/* -----------------------------------------------------------------------------
 *  Step 6 — KalmanFilterCVD_Start
 *  Commit all matrices and set the initial state.
 *
 *  Parameters
 *  ----------
 *  initial_pos  Initial position estimate.
 *  initial_vel  Initial velocity estimate (0 if unknown).
 * ----------------------------------------------------------------------------- */
void KalmanFilterCVD_Start(KalmanFilterCVD_t *kf,
                            float initial_pos,
                            float initial_vel);

/* -----------------------------------------------------------------------------
 *  KalmanFilterCVD_Update
 *
 *  Parameters
 *  ----------
 *  z  Raw position measurement.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCVD_Update(KalmanFilterCVD_t *kf, float z);

/* -----------------------------------------------------------------------------
 *  KalmanFilterCVD_Get_Position
 *  Returns the filtered position estimate.
 * ----------------------------------------------------------------------------- */
float KalmanFilterCVD_Get_Position(KalmanFilterCVD_t *kf);

/* -----------------------------------------------------------------------------
 *  KalmanFilterCVD_Get_Velocity
 *  Returns the estimated velocity (derived, not directly measured).
 * ----------------------------------------------------------------------------- */
float KalmanFilterCVD_Get_Velocity(KalmanFilterCVD_t *kf);

#endif /* INC_KALMANFILTERCVD_H_ */
