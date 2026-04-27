/*
 * KalmanFilterCD.h
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Kalman Filter — Constant Dynamics (CD)
 * =============================================================================
 *  Model: the true state is constant (or nearly constant) between samples.
 *
 *  State vector
 *  ------------
 *    x = [ value ]     (N = 1)
 *    z = [ value ]     (M = 1)
 *
 *  Matrices
 *  --------
 *    F = [ 1 ]     Q = [ q ]     H = [ 1 ]     R = [ r ]
 *
 *    Q  — process noise variance
 *    R  — measurement noise variance
 *    P  — initial covariance
 *
 *  Init Sequence
 *  -------------
 *    KalmanFilterCD_Init(&kf);
 *    KalmanFilterCD_Set_ProcessNoise(&kf, Q);
 *    KalmanFilterCD_Set_MeasurementNoise(&kf, R);
 *    KalmanFilterCD_Set_Covariance(&kf, P);
 *    KalmanFilterCD_Start(&kf, initial_value);
 *
 *  Interrupt Sequence
 *  -------------
 *    KalmanFilterCD_Update(&kf, raw_z);
 *    float x = KalmanFilterCD_Get_State(&kf);
 * =============================================================================
 */

#ifndef INC_KALMANFILTERCD_H_
#define INC_KALMANFILTERCD_H_

#include "KalmanFilter.h"

/* =============================================================================
 *  KalmanFilterCD_t  —  Handle
 * =============================================================================
 */
typedef struct {
    KalmanFilter_t  _kf;

    /* Backing arrays (N=1, M=1) */
    float _x[1];
    float _P[1];
    float _F[1];
    float _Q[1];
    float _H[1];
    float _R[1];
    float _sc[KF_SCRATCH_SIZE];
    float _q;   /* Process noise variance    */
    float _r;   /* Measurement noise variance */
    float _p;   /* Initial covariance — must be set by Set_Covariance before Start */
} KalmanFilterCD_t;

/* =============================================================================
 *  API
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 *  Step 1 — KalmanFilterCD_Init
 * ----------------------------------------------------------------------------- */
void KalmanFilterCD_Init(KalmanFilterCD_t *kf);

/* -----------------------------------------------------------------------------
 *  Step 2 — KalmanFilterCD_Set_ProcessNoise
 *  Set process noise variance Q.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCD_Set_ProcessNoise(KalmanFilterCD_t *kf, float Q);

/* -----------------------------------------------------------------------------
 *  Step 3 — KalmanFilterCD_Set_MeasurementNoise
 *  Set sensor noise variance R.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCD_Set_MeasurementNoise(KalmanFilterCD_t *kf, float R);

/* -----------------------------------------------------------------------------
 *  Step 4 — KalmanFilterCD_Set_Covariance
 *  Set initial estimate covariance
 *
 *  CD state: [ value ]
 *
 *  Parameters
 *  ----------
 *  P_val  Initial variance of the state estimate.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCD_Set_Covariance(KalmanFilterCD_t *kf, float P_val);

/* -----------------------------------------------------------------------------
 *  Step 5 — KalmanFilterCD_Start
 *
 *  Parameters
 *  ----------
 *  initial_state  Best guess of the state value at t=0.
 * ----------------------------------------------------------------------------- */
void KalmanFilterCD_Start(KalmanFilterCD_t *kf, float initial_state);

/* -----------------------------------------------------------------------------
 *  KalmanFilterCD_Update
 *  Parameters
 *  ----------
 *  z  Raw sensor measurement
 * ----------------------------------------------------------------------------- */
void KalmanFilterCD_Update(KalmanFilterCD_t *kf, float z);

/* -----------------------------------------------------------------------------
 *  KalmanFilterCD_Get_State
 *  Returns the current filtered state estimate.
 * ----------------------------------------------------------------------------- */
float KalmanFilterCD_Get_State(KalmanFilterCD_t *kf);

#endif /* INC_KALMANFILTERCD_H_ */
