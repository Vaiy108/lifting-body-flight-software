/* ---------------------------------------------------------------------
 * VENDORED, UNMODIFIED, from lifting-body-gnc (Phase 3/4a).
 * Source of truth: https://github.com/Vaiy108/lifting-body-gnc
 *   (see that repo's c/ directory and README for the physics models,
 *   navigation/control design, SIL cross-validation methodology, and
 *   host-simulated PIL verification this firmware is built on).
 * Never edit this file here -- make changes upstream in
 * lifting-body-gnc and re-vendor, so the two repos never silently
 * diverge.
 * --------------------------------------------------------------------- */

/* eskf.c -- see eskf.h and python/hlgnc/navigation.py for derivation.
 *
 * Embedded stack-budget note: the large N_ERR-scale (15x15/16x16)
 * scratch matrices inside eskf_predict() and joseph_update() are
 * declared `static` rather than on the stack. With stack-allocated
 * temporaries, eskf_predict() alone required ~17 KB of stack and the
 * predict+update call chain required ~37 KB (measured with
 * `gcc -fstack-usage`) -- more than the default stack budget on an
 * STM32F401RE (96 KB total SRAM shared across stack, heap, and all
 * static data). Moving these temporaries to static storage (.bss)
 * eliminates that risk entirely, at the cost of eskf_predict() and
 * the update functions no longer being reentrant or thread-safe. This
 * is an accepted tradeoff for a single-threaded, single-instance
 * flight-control loop (the target architecture here); it would need
 * revisiting if multiple ESKF instances or a multi-threaded/RTOS
 * context were ever required.
 */
#include "eskf.h"
#include "quat_math.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define S (MATLIB_MAXN)

void eskf_default_noise(ImuNoiseParams *np) {
    np->accel_noise_std = 0.02;
    np->gyro_noise_std = 0.01 * (M_PI / 180.0);
    np->gyro_bias_walk_std = 1e-6;
    np->accel_bias_walk_std = 1e-5;
}

void eskf_init(ESKF *kf, const double p0[3], const double v0[3],
              const double q0[4], double pos_std, double vel_std,
              double att_std, double bg_std, double ba_std) {
    memcpy(kf->state.p, p0, 3 * sizeof(double));
    memcpy(kf->state.v, v0, 3 * sizeof(double));
    memcpy(kf->state.q, q0, 4 * sizeof(double));
    kf->state.bg[0] = kf->state.bg[1] = kf->state.bg[2] = 0.0;
    kf->state.ba[0] = kf->state.ba[1] = kf->state.ba[2] = 0.0;

    mat_zero(&kf->P[0][0], N_ERR, N_ERR, S);
    double vars[N_ERR];
    for (int i = 0; i < 3; i++) vars[i] = pos_std * pos_std;
    for (int i = 3; i < 6; i++) vars[i] = vel_std * vel_std;
    for (int i = 6; i < 9; i++) vars[i] = att_std * att_std;
    for (int i = 9; i < 12; i++) vars[i] = bg_std * bg_std;
    for (int i = 12; i < 15; i++) vars[i] = ba_std * ba_std;
    for (int i = 0; i < N_ERR; i++) kf->P[i][i] = vars[i];

    eskf_default_noise(&kf->noise);
}

void eskf_predict(ESKF *kf, const double f_meas[3], const double w_meas[3],
                  double dt) {
    NavState *st = &kf->state;
    double f_b[3], w_b[3];
    vec3_sub(f_meas, st->ba, f_b);
    vec3_sub(w_meas, st->bg, w_b);

    double C_nb[3][3];
    quat_to_dcm(st->q, C_nb);

    double a_ned[3], g_ned[3] = {0.0, 0.0, GRAV};
    mat3_vec3_mult(C_nb, f_b, a_ned);
    vec3_add(a_ned, g_ned, a_ned);

    double half_a_dt2[3], v_dt[3];
    vec3_scale(a_ned, 0.5 * dt * dt, half_a_dt2);
    vec3_scale(st->v, dt, v_dt);
    vec3_add(st->p, v_dt, st->p);
    vec3_add(st->p, half_a_dt2, st->p);

    double a_dt[3];
    vec3_scale(a_ned, dt, a_dt);
    vec3_add(st->v, a_dt, st->v);

    double w_dt[3], dq[4], q_new[4];
    vec3_scale(w_b, dt, w_dt);
    quat_from_rotvec(w_dt, dq);
    quat_mult(st->q, dq, q_new);
    quat_normalize(q_new);
    memcpy(st->q, q_new, 4 * sizeof(double));

    /* -- error-state transition matrix F = I + Fc*dt (first order) -- */
    static double F[MATLIB_MAXN][MATLIB_MAXN];
    mat_eye(&F[0][0], N_ERR, S);
    for (int i = 0; i < 3; i++) F[i][3 + i] = dt;   /* dp/dv */

    double skf[3][3], Cskf[3][3];
    skew3(f_b, skf);
    mat3_mult(C_nb, skf, Cskf);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            F[3 + i][6 + j] = -Cskf[i][j] * dt;      /* dv/dtheta */
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            F[3 + i][12 + j] = -C_nb[i][j] * dt;     /* dv/dba */

    double skw[3][3];
    skew3(w_b, skw);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            F[6 + i][6 + j] = (i == j ? 1.0 : 0.0) - skw[i][j] * dt; /* dtheta/dtheta */
    for (int i = 0; i < 3; i++) F[6 + i][9 + i] = -dt; /* dtheta/dbg */

    /* -- process noise: Qd = G Qc G^T * dt, G maps [n_v,n_theta,n_bg,n_ba] -- */
    static double G[MATLIB_MAXN][MATLIB_MAXN];
    mat_zero(&G[0][0], N_ERR, 12, S);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            G[3 + i][0 + j] = C_nb[i][j];
    for (int i = 0; i < 3; i++) G[6 + i][3 + i] = -1.0;
    for (int i = 0; i < 3; i++) G[9 + i][6 + i] = 1.0;
    for (int i = 0; i < 3; i++) G[12 + i][9 + i] = 1.0;

    double Qc[12];
    for (int i = 0; i < 3; i++) Qc[i] = kf->noise.accel_noise_std * kf->noise.accel_noise_std;
    for (int i = 3; i < 6; i++) Qc[i] = kf->noise.gyro_noise_std * kf->noise.gyro_noise_std;
    for (int i = 6; i < 9; i++) Qc[i] = kf->noise.gyro_bias_walk_std * kf->noise.gyro_bias_walk_std;
    for (int i = 9; i < 12; i++) Qc[i] = kf->noise.accel_bias_walk_std * kf->noise.accel_bias_walk_std;

    /* GQc (15x12): scale columns of G by Qc */
    static double GQc[MATLIB_MAXN][MATLIB_MAXN];
    for (int i = 0; i < N_ERR; i++)
        for (int j = 0; j < 12; j++)
            GQc[i][j] = G[i][j] * Qc[j];

    static double Gt[MATLIB_MAXN][MATLIB_MAXN];
    mat_transpose(&G[0][0], N_ERR, 12, S, &Gt[0][0], S);

    static double Qd[MATLIB_MAXN][MATLIB_MAXN];
    mat_mult(&GQc[0][0], N_ERR, 12, S, &Gt[0][0], 12, N_ERR, S, &Qd[0][0], S);
    for (int i = 0; i < N_ERR; i++)
        for (int j = 0; j < N_ERR; j++)
            Qd[i][j] *= dt;

    /* P = F P F^T + Qd */
    static double FP[MATLIB_MAXN][MATLIB_MAXN], Ft[MATLIB_MAXN][MATLIB_MAXN];
    static double FPFt[MATLIB_MAXN][MATLIB_MAXN];
    mat_mult(&F[0][0], N_ERR, N_ERR, S, &kf->P[0][0], N_ERR, N_ERR, S, &FP[0][0], S);
    mat_transpose(&F[0][0], N_ERR, N_ERR, S, &Ft[0][0], S);
    mat_mult(&FP[0][0], N_ERR, N_ERR, S, &Ft[0][0], N_ERR, N_ERR, S, &FPFt[0][0], S);
    mat_add(&FPFt[0][0], &Qd[0][0], &kf->P[0][0], N_ERR, N_ERR, S, S, S);
}

/* Shared Joseph-form update, H is (m x 15), R is (m x m), innov is (m). */
static int joseph_update(ESKF *kf, double H[MATLIB_MAXN][MATLIB_MAXN], int m,
                         double R[MATLIB_MAXN][MATLIB_MAXN],
                         const double *innov) {
    static double HP[MATLIB_MAXN][MATLIB_MAXN], Ht[MATLIB_MAXN][MATLIB_MAXN];
    mat_mult(&H[0][0], m, N_ERR, S, &kf->P[0][0], N_ERR, N_ERR, S, &HP[0][0], S);
    mat_transpose(&H[0][0], m, N_ERR, S, &Ht[0][0], S);

    static double Sm[MATLIB_MAXN][MATLIB_MAXN];
    mat_mult(&HP[0][0], m, N_ERR, S, &Ht[0][0], N_ERR, m, S, &Sm[0][0], S);
    mat_add(&Sm[0][0], &R[0][0], &Sm[0][0], m, m, S, S, S);

    static double PHt[MATLIB_MAXN][MATLIB_MAXN];
    mat_mult(&kf->P[0][0], N_ERR, N_ERR, S, &Ht[0][0], N_ERR, m, S, &PHt[0][0], S);

    static double B[MATLIB_MAXN][MATLIB_MAXN]; /* B = PHt^T (m x 15), becomes K^T after solve */
    mat_transpose(&PHt[0][0], N_ERR, m, S, &B[0][0], S);

    if (mat_solve(Sm, m, B, N_ERR) != 0) return -1;

    static double K[MATLIB_MAXN][MATLIB_MAXN]; /* K = B^T (15 x m) */
    mat_transpose(&B[0][0], m, N_ERR, S, &K[0][0], S);

    double dx[N_ERR];
    for (int i = 0; i < N_ERR; i++) {
        double s = 0.0;
        for (int j = 0; j < m; j++) s += K[i][j] * innov[j];
        dx[i] = s;
    }

    static double KH[MATLIB_MAXN][MATLIB_MAXN], IKH[MATLIB_MAXN][MATLIB_MAXN];
    mat_mult(&K[0][0], N_ERR, m, S, &H[0][0], m, N_ERR, S, &KH[0][0], S);
    static double I15[MATLIB_MAXN][MATLIB_MAXN];
    mat_eye(&I15[0][0], N_ERR, S);
    mat_sub(&I15[0][0], &KH[0][0], &IKH[0][0], N_ERR, N_ERR, S, S, S);

    static double IKHt[MATLIB_MAXN][MATLIB_MAXN], tmp1[MATLIB_MAXN][MATLIB_MAXN], term1[MATLIB_MAXN][MATLIB_MAXN];
    mat_transpose(&IKH[0][0], N_ERR, N_ERR, S, &IKHt[0][0], S);
    mat_mult(&IKH[0][0], N_ERR, N_ERR, S, &kf->P[0][0], N_ERR, N_ERR, S, &tmp1[0][0], S);
    mat_mult(&tmp1[0][0], N_ERR, N_ERR, S, &IKHt[0][0], N_ERR, N_ERR, S, &term1[0][0], S);

    static double KR[MATLIB_MAXN][MATLIB_MAXN], Kt[MATLIB_MAXN][MATLIB_MAXN], term2[MATLIB_MAXN][MATLIB_MAXN];
    mat_mult(&K[0][0], N_ERR, m, S, &R[0][0], m, m, S, &KR[0][0], S);
    mat_transpose(&K[0][0], N_ERR, m, S, &Kt[0][0], S);
    mat_mult(&KR[0][0], N_ERR, m, S, &Kt[0][0], m, N_ERR, S, &term2[0][0], S);

    static double P_new[MATLIB_MAXN][MATLIB_MAXN];
    mat_add(&term1[0][0], &term2[0][0], &P_new[0][0], N_ERR, N_ERR, S, S, S);

    /* inject and reset */
    NavState *st = &kf->state;
    for (int i = 0; i < 3; i++) st->p[i] += dx[i];
    for (int i = 0; i < 3; i++) st->v[i] += dx[3 + i];
    double dtheta[3] = {dx[6], dx[7], dx[8]};
    double dq[4], q_new[4];
    quat_from_rotvec(dtheta, dq);
    quat_mult(st->q, dq, q_new);
    quat_normalize(q_new);
    memcpy(st->q, q_new, 4 * sizeof(double));
    for (int i = 0; i < 3; i++) st->bg[i] += dx[9 + i];
    for (int i = 0; i < 3; i++) st->ba[i] += dx[12 + i];

    for (int i = 0; i < N_ERR; i++)
        for (int j = 0; j < N_ERR; j++)
            kf->P[i][j] = P_new[i][j];
    return 0;
}

int eskf_update_gnss(ESKF *kf, const double pos_meas[3],
                     const double vel_meas[3], const double pos_std[3],
                     double vel_std) {
    static double H[MATLIB_MAXN][MATLIB_MAXN];
    mat_zero(&H[0][0], 6, N_ERR, S);
    for (int i = 0; i < 3; i++) H[i][i] = 1.0;
    for (int i = 0; i < 3; i++) H[3 + i][3 + i] = 1.0;

    static double R[MATLIB_MAXN][MATLIB_MAXN];
    mat_zero(&R[0][0], 6, 6, S);
    for (int i = 0; i < 3; i++) R[i][i] = pos_std[i] * pos_std[i];
    for (int i = 0; i < 3; i++) R[3 + i][3 + i] = vel_std * vel_std;

    double innov[6];
    for (int i = 0; i < 3; i++) innov[i] = pos_meas[i] - kf->state.p[i];
    for (int i = 0; i < 3; i++) innov[3 + i] = vel_meas[i] - kf->state.v[i];

    return joseph_update(kf, H, 6, R, innov);
}

int eskf_update_baro(ESKF *kf, double alt_meas, double alt_std) {
    static double H[MATLIB_MAXN][MATLIB_MAXN];
    mat_zero(&H[0][0], 1, N_ERR, S);
    H[0][2] = -1.0;

    static double R[MATLIB_MAXN][MATLIB_MAXN];
    mat_zero(&R[0][0], 1, 1, S);
    R[0][0] = alt_std * alt_std;

    double innov[1];
    innov[0] = alt_meas - (-kf->state.p[2]);

    return joseph_update(kf, H, 1, R, innov);
}
