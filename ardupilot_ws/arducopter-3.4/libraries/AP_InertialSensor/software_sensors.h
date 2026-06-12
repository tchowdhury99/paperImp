// software_sensors.h
// Sensor conversion equations — Section 3.2 of Choi et al. RAID 2020
#pragma once
#include <cmath>

// ── Smooth noise-robust differentiator (Holoborodko, cited Section 3.3) ──────
// Causal 5-point form (derivative evaluated at n-2, i.e. 2-sample delay):
//   f'[n-2] ≈ (2*(f[n-1]-f[n-3]) + (f[n]-f[n-4])) / (8*h)
// Exact on ramps; zero gain at Nyquist. buf[0]=f[n] ... buf[4]=f[n-4].
// (The earlier 4-point variant had a 25% gain error on ramps.)
static inline float holoborodko_deriv(const float* buf, float h) {
    return (2.0f*(buf[1]-buf[3]) + (buf[0]-buf[4])) / (8.0f * h);
}

// ── Accelerometer (Eq. 4) ─────────────────────────────────────────────────────
// a(t) = c_k * (v(t) - v(t-k)) / (k*dt) realized as the noise-robust
// differentiator over the model velocity prediction history (§3.3).
static inline float software_accel(const float* v_buf, float dt) {
    return holoborodko_deriv(v_buf, dt);
}

// wrap angle difference to [-pi, pi] (heading residual canonicalization)
static inline float wrap_pi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

// ── Barometer (Eq. 5) ─────────────────────────────────────────────────────────
// Ph = P0 * exp(-g0*M*(z-h0) / (R*T0))
static inline float software_baro(float z_m,
                                   float h0=0.0f,
                                   float P0=101325.0f,
                                   float T0=288.15f) {
    const float g0 = 9.87f;
    const float M  = 0.02896f;
    const float R  = 8.3143f;
    return P0 * expf((-g0 * M * (z_m - h0)) / (R * T0));
}

// ── Magnetometer heading (Eq. 6) ──────────────────────────────────────────────
static inline float software_mag_heading(float mx, float my, float mz,
                                          float phi, float theta) {
    return atan2f(-my*cosf(phi) + mz*sinf(phi),
                   mx*cosf(theta)
                 + my*sinf(theta)*sinf(phi)
                 + mz*sinf(theta)*cosf(phi));
}

// ── Frame: body → inertial rotation matrix (Appendix A, Eq. 8) ───────────────
static inline void body_to_inertial_R(float phi, float theta, float psi,
                                       float R[3][3]) {
    float Cp=cosf(phi),  Sp=sinf(phi);
    float Ct=cosf(theta),St=sinf(theta);
    float Cy=cosf(psi),  Sy=sinf(psi);
    R[0][0]= Cy*Ct;  R[0][1]= Cy*St*Sp - Sy*Cp;  R[0][2]= Cy*St*Cp + Sy*Sp;
    R[1][0]= Sy*Ct;  R[1][1]= Sy*St*Sp + Cy*Cp;  R[1][2]= Sy*St*Cp - Cy*Sp;
    R[2][0]=-St;     R[2][1]= Ct*Sp;              R[2][2]= Ct*Cp;
}

// ── Body rates → Euler rates (Appendix A, Eq. 10) ────────────────────────────
static inline void body_to_euler_rates(float phi, float theta,
                                        float p, float q, float r,
                                        float* phi_d, float* theta_d, float* psi_d) {
    float Cp=cosf(phi), Sp=sinf(phi);
    float Ct=cosf(theta), Tt=tanf(theta);
    *phi_d   = p + (q*Sp + r*Cp)*Tt;
    *theta_d = q*Cp - r*Sp;
    *psi_d   = (q*Sp + r*Cp)/Ct;
}

// ── Supplementary compensation (Appendix B, Eq. 11) ──────────────────────────
// Used when ALL gyros are compromised (Table 3 C3/C5/C6).
static inline void supplementary_compensation(
    float xa, float ya, float za,
    float xm, float ym, float zm,
    float* phi_acc, float* theta_acc, float* psi_mag) {
    *phi_acc   = atan2f(ya, sqrtf(xa*xa + za*za));
    *theta_acc = atan2f(xa, sqrtf(ya*ya + za*za));
    *psi_mag   = atan2f(-ym*cosf(*phi_acc) + zm*sinf(*phi_acc),
                         xm*cosf(*theta_acc)
                       + ym*sinf(*theta_acc)*sinf(*phi_acc)
                       + zm*sinf(*theta_acc)*cosf(*phi_acc));
}
