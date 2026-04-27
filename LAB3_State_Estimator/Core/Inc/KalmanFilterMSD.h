/*
 * KalmanFilterMSD.h
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Kalman Filter — Mass-Spring-Damper (MSD)
 * =============================================================================
 *  Model: the system obeys second-order physical dynamics:
 *
 *    m·ẍ + c·ẋ + k·x = 0
 *
 *  Discretised with first-order Euler.
 *
 *  State vector
 *  ------------
 *    x = [ position ]    (N = 2)
 *        [ velocity ]
 *    z = [ position ]    (M = 1)
 *
 *  Matrices
 *  --------
 *    F = [  1              dt      ]     H = [ 1  0 ]
 *        [ -(k/m)·dt   1-(c/m)·dt ]
 *
 *    Q diagonal = [ q_pos,  0    ]
 *                 [  0,   q_vel  ]
 *
 *  Tuning
 *  ------
 *    Q_Pos  — position process noise variance Q[0,0]
 *    Q_Vel  — velocity process noise variance Q[1,1]
 *    R      — sensor noise variance
 *    P_pos  — initial position covariance
 *    P_vel  — initial velocity covariance
 *
 *  Init Sequence
 *  -------------
 *    KalmanFilterMSD_Init(&kf, m, c, k);
 *    KalmanFilterMSD_Set_ObserverPeriod(&kf, dt);
 *    KalmanFilterMSD_Set_ProcessNoise(&kf, Q_Pos, Q_Vel);
 *    KalmanFilterMSD_Set_MeasurementNoise(&kf, R);
 *    KalmanFilterMSD_Set_Covariance(&kf, P_pos, P_vel);
 *    KalmanFilterMSD_Start(&kf, initial_pos, initial_vel);
 *
 *    // Every sample period:
 *    KalmanFilterMSD_Update(&kf, raw_z);
 *    float pos = KalmanFilterMSD_Get_Position(&kf);
 *    float vel = KalmanFilterMSD_Get_Velocity(&kf);
 * =============================================================================
 */

#ifndef INC_KALMANFILTERMSD_H_
#define INC_KALMANFILTERMSD_H_

#include "KalmanFilter.h"

/* =============================================================================
 *  KalmanFilterMSD_t  —  Handle
 * =============================================================================
 */
typedef struct {
    KalmanFilter_t  _kf;

    float _x[2];
    float _P[4];
    float _F[4];
    float _Q[4];
    float _H[2];
    float _R[1];
    float _sc[KF_SCRATCH_SIZE];
    float   _m, _c, _k;
    float   _dt;
    float   _p_pos;
    float   _p_vel;
    uint8_t _started;
} KalmanFilterMSD_t;

/* =============================================================================
 *  API
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 *  Step 1 — KalmanFilterMSD_Init
 *  Zero handle, wire pointers, store physical system parameters.
 *
 *  Parameters
 *  ----------
 *  m  Mass            [kg]
 *  c  Damping coeff   [N·s/m]
 *  k  Spring constant [N/m]
 * ----------------------------------------------------------------------------- */
void KalmanFilterMSD_Init(KalmanFilterMSD_t *kf,
                           float m, float c, float k);

/* -----------------------------------------------------------------------------
 *  Step 2 — KalmanFilterMSD_Set_ObserverPeriod
 *  Set the observer (sample) period. Rebuilds F to match the new dt.
 *
 *  Parameters
 *  ----------
 *  dt  Sample period in seconds.
 * ----------------------------------------------------------------------------- */
void KalmanFilterMSD_Set_ObserverPeriod(KalmanFilterMSD_t *kf, float dt);

/* -----------------------------------------------------------------------------
 *  Step 3 — KalmanFilterMSD_Set_ProcessNoise
 *  Set process noise (diagonal Q).
 *
 *  Parameters
 *  ----------
 *  Q_Pos  Position process noise variance Q[0,0].
 *  Q_Vel  Velocity process noise variance Q[1,1].
 * ----------------------------------------------------------------------------- */
void KalmanFilterMSD_Set_ProcessNoise(KalmanFilterMSD_t *kf,
                                       float Q_Pos, float Q_Vel);

/* -----------------------------------------------------------------------------
 *  Step 4 — KalmanFilterMSD_Set_MeasurementNoise
 *  Set sensor noise variance R.
 *
 *  Parameters
 *  ----------
 *  R  Sensor noise variance.
 * ----------------------------------------------------------------------------- */
void KalmanFilterMSD_Set_MeasurementNoise(KalmanFilterMSD_t *kf, float R);

/* -----------------------------------------------------------------------------
 *  Step 5 — KalmanFilterMSD_Set_Covariance
 *  Set initial estimate covariance P (diagonal).
 *
 *  MSD state: [ position, velocity ]
 *
 *  Parameters
 *  ----------
 *  P_pos  Initial position variance P[0,0].
 *  P_vel  Initial velocity variance P[1,1].
 * ----------------------------------------------------------------------------- */
void KalmanFilterMSD_Set_Covariance(KalmanFilterMSD_t *kf,
                                     float P_pos, float P_vel);

/* -----------------------------------------------------------------------------
 *  Step 6 — KalmanFilterMSD_Start
 *  Commit all matrices and set the initial state.
 *
 *  Parameters
 *  ----------
 *  initial_pos  Initial position estimate.
 *  initial_vel  Initial velocity estimate.
 * ----------------------------------------------------------------------------- */
void KalmanFilterMSD_Start(KalmanFilterMSD_t *kf,
                            float initial_pos,
                            float initial_vel);

/* -----------------------------------------------------------------------------
 *  KalmanFilterMSD_Update
 *
 *  Parameters
 *  ----------
 *  z  Raw position measurement.
 * ----------------------------------------------------------------------------- */
void KalmanFilterMSD_Update(KalmanFilterMSD_t *kf, float z);

/* -----------------------------------------------------------------------------
 *  KalmanFilterMSD_Get_Position  —  Returns filtered position estimate.
 *  KalmanFilterMSD_Get_Velocity  —  Returns estimated velocity.
 * ----------------------------------------------------------------------------- */
float KalmanFilterMSD_Get_Position(KalmanFilterMSD_t *kf);
float KalmanFilterMSD_Get_Velocity(KalmanFilterMSD_t *kf);

#endif /* INC_KALMANFILTERMSD_H_ */
