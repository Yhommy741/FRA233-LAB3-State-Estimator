/*
 * KalmanFilter.h
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Kalman Filter — N-Dimensional Engine
 * =============================================================================
 *
 *  Equations
 *  ---------
 *  PREDICT:  x_pred = F * x
 *            P_pred = F * P * F^T + Q
 *
 *  UPDATE:   y  = z - H * x_pred          (innovation)
 *            S  = H * P_pred * H^T + R    (innovation covariance)
 *            K  = P_pred * H^T * S^-1     (Kalman gain)
 *            x  = x_pred + K * y
 *            P  = (I - K*H) * P_pred
 *
 *  Dimension Limits
 *  ----------------
 *  KF_MAX_N  max state dimensions      (default 4 — covers all built-in models)
 *  KF_MAX_M  max measurement dims      (default 1 — all built-in models use M=1)
 * =============================================================================
 */

#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_

#include "main.h"
#include "arm_math.h"
#include <string.h>
#include <stdint.h>

/* =============================================================================
 *  Dimension Caps
 *  Covers all built-in models: CD=1, CVD=2, CAD=3, MSD=2
 * =============================================================================
 */
#ifndef KF_MAX_N
#define KF_MAX_N  4
#endif
#ifndef KF_MAX_M
#define KF_MAX_M  1
#endif

/* =============================================================================
 *  Scratch Buffer Size
 *  Sum of all intermediate matrix regions needed by KalmanFilter_Update()
 * =============================================================================
 */
#define KF_SCRATCH_SIZE  (KF_MAX_N + KF_MAX_N*KF_MAX_N*7 + KF_MAX_M*KF_MAX_N*2 \
                          + KF_MAX_M*KF_MAX_M*3 + KF_MAX_N*KF_MAX_M*2 + KF_MAX_M + KF_MAX_N)

/* =============================================================================
 *  KalmanFilter_t  —  Core Engine Handle
 *  Embedded inside every model handle. Do not declare or access directly.
 * =============================================================================
 */
typedef struct {
    uint8_t N;    /* State dimension       */
    uint8_t M;    /* Measurement dimension */

    arm_matrix_instance_f32 x;   /* State vector      (N×1) */
    arm_matrix_instance_f32 P;   /* Error covariance  (N×N) */
    arm_matrix_instance_f32 F;   /* State transition  (N×N) */
    arm_matrix_instance_f32 Q;   /* Process noise     (N×N) */
    arm_matrix_instance_f32 H;   /* Observation model (M×N) */
    arm_matrix_instance_f32 R;   /* Measurement noise (M×M) */

    float *x_data;
    float *P_data;
    float *F_data;
    float *Q_data;
    float *H_data;
    float *R_data;
    float *scratch;
} KalmanFilter_t;

/* =============================================================================
 *  Engine Functions
 * =============================================================================
 */
void KalmanFilter_Init   (KalmanFilter_t *kf, uint8_t N, uint8_t M);
void KalmanFilter_Update (KalmanFilter_t *kf, const float *z);

#endif /* INC_KALMANFILTER_H_ */
