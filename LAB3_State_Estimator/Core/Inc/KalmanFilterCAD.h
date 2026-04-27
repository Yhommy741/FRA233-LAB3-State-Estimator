/*
 * KalmanFilterCAD.h
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Kalman Filter — Constant Acceleration Dynamics (CAD)
 * =============================================================================
 *
 *  State vector
 *  ------------
 *    x = [ position     ]    (N = 3)
 *        [ velocity     ]
 *        [ acceleration ]
 *    z = [ position     ]    (M = 1)
 *
 *  Matrices
 *  --------
 *    F = [ 1   dt   dt²/2 ]     H = [ 1  0  0 ]
 *        [ 0    1    dt   ]
 *        [ 0    0     1   ]
 *
 *    Q is built automatically from VarAccel (σ_a²):
 *      Q = [ dt⁴/4   dt³/2   dt²/2 ]
 *          [ dt³/2   dt²     dt    ] × σ_a²
 *          [ dt²/2   dt      1     ]
 *
 *    VarAccel — process noise
 *    R        — sensor noise
 *    P_pos    — initial position
 *    P_vel    — initial velocity
 *    P_accel  — initial acceleration
 *
 *  Init Sequence
 *  -------------
 *    KalmanFilterCAD_Init(&kf);
 *    KalmanFilterCAD_Set_ObserverPeriod(&kf, dt);
 *    KalmanFilterCAD_Set_ProcessNoise(&kf, VarAccel);
 *    KalmanFilterCAD_Set_MeasurementNoise(&kf, R);
 *    KalmanFilterCAD_Set_Covariance(&kf, P_pos, P_vel, P_accel);
 *    KalmanFilterCAD_Start(&kf, initial_pos, initial_vel, initial_accel);
 *
 *  Interrupt Sequence
 *  -------------
 *    KalmanFilterCAD_Update(&kf, raw_z);
 *    float pos   = KalmanFilterCAD_Get_Position(&kf);
 *    float vel   = KalmanFilterCAD_Get_Velocity(&kf);
 *    float accel = KalmanFilterCAD_Get_Accel(&kf);
 * =============================================================================
 */

#ifndef INC_KALMANFILTERCAD_H_
#define INC_KALMANFILTERCAD_H_

#include "KalmanFilter.h"

/* =============================================================================
 *  KalmanFilterCAD_t  —  Handle
 * =============================================================================
 */
typedef struct {
    KalmanFilter_t  _kf;

    /* Backing arrays (N=3, M=1) */
    float _x[3];
    float _P[9];
    float _F[9];
    float _Q[9];
    float _H[3];
    float _R[1];
    float _sc[KF_SCRATCH_SIZE];
    float   _dt;
    float   _var_accel;
    float   _p_pos;
    float   _p_vel;
    float   _p_accel;
    uint8_t _started;
} KalmanFilterCAD_t;

/* =============================================================================
 *  API
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 *  Step 1 — KalmanFilterCAD_Init
 *  Zero handle and wire internal memory pointers.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCAD_Init(KalmanFilterCAD_t *kf);

/* -----------------------------------------------------------------------------
 *  Step 2 — KalmanFilterCAD_Set_ObserverPeriod
 *  Set the observer (sample) period. Rebuilds F and Q to match the new dt.
 *
 *  Parameters
 *  ----------
 *  dt  Sample period in seconds.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCAD_Set_ObserverPeriod(KalmanFilterCAD_t *kf, float dt);

/* -----------------------------------------------------------------------------
 *  Step 3 — KalmanFilterCAD_Set_ProcessNoise
 *  Set process noise from acceleration variance. Rebuilds Q automatically.
 *
 *  Parameters
 *  ----------
 *  VarAccel  Expected acceleration noise variance (m/s²)².
 * ----------------------------------------------------------------------------- */
void KalmanFilterCAD_Set_ProcessNoise(KalmanFilterCAD_t *kf, float VarAccel);

/* -----------------------------------------------------------------------------
 *  Step 4 — KalmanFilterCAD_Set_MeasurementNoise
 *  Set sensor noise variance R.
 *
 *  Parameters
 *  ----------
 *  R  Sensor noise variance.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCAD_Set_MeasurementNoise(KalmanFilterCAD_t *kf, float R);

/* -----------------------------------------------------------------------------
 *  Step 5 — KalmanFilterCAD_Set_Covariance
 *  Set initial estimate covariance P (diagonal).
 *
 *  CAD state: [ position, velocity, acceleration ]
 *
 *  Parameters
 *  ----------
 *  P_pos    Initial position     variance P[0,0].
 *  P_vel    Initial velocity     variance P[1,1].
 *  P_accel  Initial acceleration variance P[2,2].
 * ----------------------------------------------------------------------------- */
void KalmanFilterCAD_Set_Covariance(KalmanFilterCAD_t *kf,
                                     float P_pos, float P_vel, float P_accel);

/* -----------------------------------------------------------------------------
 *  Step 6 — KalmanFilterCAD_Start
 *  Commit all matrices and set the initial state.
 *
 *  Parameters
 *  ----------
 *  initial_pos    Initial position estimate.
 *  initial_vel    Initial velocity estimate     (0 if unknown).
 *  initial_accel  Initial acceleration estimate (0 if unknown).
 * ----------------------------------------------------------------------------- */
void KalmanFilterCAD_Start(KalmanFilterCAD_t *kf,
                            float initial_pos,
                            float initial_vel,
                            float initial_accel);

/* -----------------------------------------------------------------------------
 *  KalmanFilterCAD_Update
 *
 *  Parameters
 *  ----------
 *  z  Raw position measurement.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCAD_Update(KalmanFilterCAD_t *kf, float z);

/* -----------------------------------------------------------------------------
 *  KalmanFilterCAD_Get_Position  —  Returns filtered position estimate.
 *  KalmanFilterCAD_Get_Velocity  —  Returns estimated velocity.
 *  KalmanFilterCAD_Get_Accel     —  Returns estimated acceleration.
 * ----------------------------------------------------------------------------- */
float KalmanFilterCAD_Get_Position(KalmanFilterCAD_t *kf);
float KalmanFilterCAD_Get_Velocity(KalmanFilterCAD_t *kf);
float KalmanFilterCAD_Get_Accel   (KalmanFilterCAD_t *kf);

#endif /* INC_KALMANFILTERCAD_H_ */
