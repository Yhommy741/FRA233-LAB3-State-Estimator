/*
 * KalmanFilter.c
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  Kalman Filter — N-Dimensional Engine Implementation
 * =============================================================================
 */

#include "KalmanFilter.h"

/* =============================================================================
 *  Private Macros
 * =============================================================================
 *  _SLICE  Cut a named float pointer from the scratch pool and advance offset.
 *  _MAT    Declare a CMSIS arm_matrix_instance_f32 wrapping a float pointer.
 * =============================================================================
 */
#define _SLICE(ptr, pool, off, n)  float *(ptr) = (pool) + (off); (off) += (n)
#define _MAT(m, r, c, p)           arm_matrix_instance_f32 (m) = {(r), (c), (p)}

/* =============================================================================
 *  KalmanFilter_Init
 *  Wire N and M into the handle.
 *  Called once by every model's _Init function.
 * =============================================================================
 */
void KalmanFilter_Init(KalmanFilter_t *kf, uint8_t N, uint8_t M)
{
    kf->N = N;
    kf->M = M;
}

/* =============================================================================
 *  KalmanFilter_Update
 *  Full predict + update cycle.
 *  Called by every model's _Update function.
 *
 *  Parameters
 *  ----------
 *  kf   Pointer to engine handle (with all matrices already wired).
 *  z    Pointer to measurement vector (length M).
 * =============================================================================
 */
void KalmanFilter_Update(KalmanFilter_t *kf, const float *z)
{
    uint8_t  N  = kf->N;
    uint8_t  M  = kf->M;
    float   *sc = kf->scratch;
    uint32_t o  = 0;

    /* -----------------------------------------------------------------
     *  Slice scratch buffer into named working regions
     * ----------------------------------------------------------------- */
    _SLICE(x_new_d, sc, o, N);
    _SLICE(P_new_d, sc, o, N*N);
    _SLICE(F_T_d,   sc, o, N*N);
    _SLICE(FP_d,    sc, o, N*N);
    _SLICE(FPFt_d,  sc, o, N*N);
    _SLICE(HP_d,    sc, o, M*N);
    _SLICE(Ht_d,    sc, o, N*M);
    _SLICE(S_d,     sc, o, M*M);
    _SLICE(Si_d,    sc, o, M*M);
    _SLICE(PHt_d,   sc, o, N*M);
    _SLICE(K_d,     sc, o, N*M);
    _SLICE(y_d,     sc, o, M);
    _SLICE(Ky_d,    sc, o, N);
    _SLICE(KH_d,    sc, o, N*N);
    _SLICE(IKH_d,   sc, o, N*N);
    _SLICE(I_d,     sc, o, N*N);

    /* -----------------------------------------------------------------
     *  Wrap each region in a CMSIS matrix instance
     * ----------------------------------------------------------------- */
    _MAT(x_new, N, 1, x_new_d);
    _MAT(P_new, N, N, P_new_d);
    _MAT(F_T,   N, N, F_T_d);
    _MAT(FP,    N, N, FP_d);
    _MAT(FPFt,  N, N, FPFt_d);
    _MAT(HP,    M, N, HP_d);
    _MAT(Ht,    N, M, Ht_d);
    _MAT(S,     M, M, S_d);
    _MAT(Si,    M, M, Si_d);
    _MAT(PHt,   N, M, PHt_d);
    _MAT(K,     N, M, K_d);
    _MAT(y,     M, 1, y_d);
    _MAT(Ky,    N, 1, Ky_d);
    _MAT(KH,    N, N, KH_d);
    _MAT(IKH,   N, N, IKH_d);

    /* =================================================================
     *  PREDICT
     * =================================================================
     *  x_pred = F * x
     *  P_pred = F * P * F^T + Q
     * ================================================================= */
    arm_mat_mult_f32(&kf->F, &kf->x, &x_new);   /* x_new = F * x        */
    arm_mat_mult_f32(&kf->F, &kf->P, &FP);       /* FP    = F * P        */
    arm_mat_trans_f32(&kf->F, &F_T);              /* F_T   = F^T          */
    arm_mat_mult_f32(&FP, &F_T, &FPFt);           /* FPFt  = FP * F^T     */
    arm_mat_add_f32(&FPFt, &kf->Q, &P_new);       /* P_new = FPFt + Q     */

    /* Commit predicted state */
    memcpy(kf->x_data, x_new_d, N   * sizeof(float));
    memcpy(kf->P_data, P_new_d, N*N * sizeof(float));

    /* =================================================================
     *  UPDATE
     * =================================================================
     *  y = z - H * x_pred                        (innovation)
     *  S = H * P_pred * H^T + R                  (innovation covariance)
     *  K = P_pred * H^T * S^-1                   (Kalman gain)
     *  x = x_pred + K * y                        (state update)
     *  P = (I - K*H) * P_pred                    (covariance update)
     * ================================================================= */

    /* y = z - H*x */
    _MAT(Hx, M, 1, y_d);
    arm_mat_mult_f32(&kf->H, &kf->x, &Hx);
    for (uint8_t i = 0; i < M; i++) y_d[i] = z[i] - y_d[i];

    /* S = H*P*H^T + R */
    arm_mat_mult_f32(&kf->H, &kf->P, &HP);
    arm_mat_trans_f32(&kf->H, &Ht);
    arm_mat_mult_f32(&HP, &Ht, &S);
    arm_mat_add_f32(&S, &kf->R, &S);

    /* S^-1  (scalar fast-path when M==1, general inverse otherwise) */
    if (M == 1) {
        Si_d[0] = (S_d[0] > 1e-9f) ? (1.0f / S_d[0]) : 0.0f;
    } else {
        arm_mat_inverse_f32(&S, &Si);
    }

    /* K = P * H^T * S^-1 */
    arm_mat_mult_f32(&kf->P, &Ht, &PHt);
    arm_mat_mult_f32(&PHt, &Si, &K);

    /* x = x + K*y */
    arm_mat_mult_f32(&K, &y, &Ky);
    arm_mat_add_f32(&kf->x, &Ky, &x_new);

    /* P = (I - K*H) * P */
    arm_mat_mult_f32(&K, &kf->H, &KH);
    memset(I_d, 0, N*N * sizeof(float));
    for (uint8_t i = 0; i < N; i++) I_d[i*N + i] = 1.0f;
    _MAT(I, N, N, I_d);
    arm_mat_sub_f32(&I, &KH, &IKH);
    arm_mat_mult_f32(&IKH, &kf->P, &P_new);

    /* Commit updated state */
    memcpy(kf->x_data, x_new_d, N   * sizeof(float));
    memcpy(kf->P_data, P_new_d, N*N * sizeof(float));
}
