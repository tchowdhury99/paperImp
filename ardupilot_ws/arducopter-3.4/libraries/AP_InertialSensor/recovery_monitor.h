// recovery_monitor.h — Algorithm 1, Choi et al. RAID 2020, implemented line-for-line.
//
//  1:  u  control input of the real vehicle (target states)
//  2:  m  sensor measurement
//  3:  x  control states of the real vehicle (model state, Eq. 3)
//  5:  procedure RECOVERYMONITOR(u, m)
//  6:    y  <- C*x + D*u                       (model output, BEFORE state advance)
//  7:    x  <- A*x + B*u                       (open loop — NO Kalman/observer term)
//  8:    m  <- filter(m)                       (low-pass filter, §3.3 Fig. 8)
//  9:    ms <- convert(y)                      (software-sensor conversion, §3.2)
// 10:    t++
// 11:    if !recovery_mode && t > window:      (checkpoint ONLY when not in recovery)
// 12:        t <- 0
// 13:        r <- 0
// 14:        e <- error_estimation(...)        (avg of (ms_raw - m) over prev window)
// 15:        ms <- m                           (sync; synchronized readings are fed
//                                              into the system model -> state re-seed,
//                                              §3.3 "Model Errors")
// 16:    ms <- ms - e                          (external/model error compensation)
// 17:    r  <- r + |m - ms|
// 18:    if r > T_on:  recovery_mode <- true; safe_count <- 0
// 19:    if recovery_mode:
// 20:        m <- ms                           (replace compromised sensor)
// 21:        if r < T_off: safe_count++        (no reset on the else branch)
// 22:        if safe_count > K: recovery_mode <- false
// 23:        recovery_action()                 (optional)
// 24:    return m
//
// Healthy-path semantics follow Fig. 3 (motivating code): the control loop receives
// the RAW reading unless recovery is active; the filtered m is used only for
// residual/error estimation. (Algorithm 1's return of the filtered m would alter
// normal-operation control, contradicting §4.2.1 "the observed overhead does not
// impact normal operations".)
//
// One shared RecoveryModel (the Eq. 1-3 vehicle model) + one RecoveryChannel per
// monitored physical sensor channel (per-sensor t, r, e, window, thresholds —
// Fig. 3 monitors each physical sensor individually).
//
// Sample rate: 50 Hz (Ts=0.02) — the one sanctioned deviation from the paper's 400 Hz.

#pragma once
#include <cmath>
#include <cstring>

#ifdef RECOVERY_TEST_MODEL
// Tiny known model for conformance tests: NX=NY=2, NU=1, stable, C=I, D=0.
static const int   NX = 2;
static const int   NU = 1;
static const int   NY = 2;
static const float TS = 0.02f;
static const float A_MAT[2][2] = {{0.90f, 0.10f}, {0.00f, 0.80f}};
static const float B_MAT[2][1] = {{0.00f}, {0.10f}};
static const float C_MAT[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
static const float D_MAT[2][1] = {{0.0f}, {0.0f}};
#define REC_WINDOW_CAP 512
#define REC_KSAFE_DEFAULT 10
static const int   REC_N_DEFAULT[2]     = {50, 50};
static const float REC_TON_DEFAULT[2]   = {1e9f, 1e9f};
static const float REC_TOFF_DEFAULT[2]  = {1e9f, 1e9f};
#else
#include "model_matrices.h"     // NX, NU, NY, TS, A_MAT, B_MAT, C_MAT, D_MAT
#include "recovery_params.h"    // REC_WINDOW_CAP, REC_KSAFE_DEFAULT, per-channel N/T
#endif

// ── filter(m): 2nd-order Butterworth low-pass (Alg.1 l.8, §3.3) ──────────────
// The paper applies "the low-pass filter [40] which is a standard filter to
// attenuate high frequencies with a pre-selected cutoff". Cutoff (documented
// choice, paper does not give one): 5 Hz at fs=50 Hz. Coefficients from
// butter(2, 5/(50/2)). The SAME filter must be used by select_parameters.py.
struct LPFilter {
    float w1, w2;       // direct form II transposed delay line
};
static const float LPF_B[3] = {0.06745527f, 0.13491055f, 0.06745527f};
static const float LPF_A[3] = {1.0f, -1.14298050f, 0.41280160f};

static inline void lpf_init(LPFilter* f) { f->w1 = f->w2 = 0.0f; }
static inline float lpf_step(LPFilter* f, float x) {
    float y = LPF_B[0] * x + f->w1;
    f->w1 = LPF_B[1] * x - LPF_A[1] * y + f->w2;
    f->w2 = LPF_B[2] * x - LPF_A[2] * y;
    return y;
}

// ── The Eq. (1)-(3) vehicle model, shared by all software sensors ────────────
struct RecoveryModel {
    float x[NX];        // model state (Eq. 3)
    float y[NY];        // model output of the current tick (line 6)
    bool  initialized;
};

static inline void recovery_model_init(RecoveryModel* m) {
    memset(m, 0, sizeof(*m));
}

// Single shared vehicle-model instance for all sensor wiring sites
// (ArduCopter.cpp gyro/accel, AP_Baro, AP_GPS, AP_Compass). Non-static inline
// function => one instance across translation units.
inline RecoveryModel& recovery_shared_model() {
    static RecoveryModel m;
    return m;
}

// Lines 6-7: output BEFORE state advance, then open-loop state update.
static inline void recovery_model_update(RecoveryModel* mdl, const float u[NU]) {
    for (int k = 0; k < NY; k++) {                    // line 6: y = C*x + D*u
        float y = 0.0f;
        for (int j = 0; j < NX; j++) y += C_MAT[k][j] * mdl->x[j];
        for (int j = 0; j < NU; j++) y += D_MAT[k][j] * u[j];
        mdl->y[k] = y;
    }
    float xn[NX] = {};
    for (int i = 0; i < NX; i++) {                    // line 7: x = A*x + B*u
        for (int j = 0; j < NX; j++) xn[i] += A_MAT[i][j] * mdl->x[j];
        for (int j = 0; j < NU; j++) xn[i] += B_MAT[i][j] * u[j];
    }
    memcpy(mdl->x, xn, sizeof(float) * NX);
}

// ── Per-sensor monitor state ─────────────────────────────────────────────────
struct RecoveryChannel {
    int      ch;          // model output index this sensor predicts (C = I)
    int      state_idx;   // model state re-seeded at sync (-1 = none); = ch for C=I
    // inverse of convert(): maps the real measurement back to the model state for
    // the line-15 sync ("synchronized readings are then fed into the system model",
    // §3.3). NULL = identity (state channels). E.g. baro: pressure -> altitude.
    float  (*inv_convert)(float m);
    LPFilter lpf;
    float    e;           // error compensation term (§3.3)
    float    r;           // accumulated residual (per window)
    int      t;           // tick counter within window
    bool     recovery_mode;
    int      safe_count;
    int      window;      // N (DTW-selected, §3.3)
    float    T_on, T_off; // thresholds (T = e_max + margin, §3.3)
    int      K_safe;      // switch-back count (Alg.1 l.22)
    float    err_hist[REC_WINDOW_CAP];   // rolling (ms_raw - m) samples
};

static inline void recovery_channel_init(RecoveryChannel* c, int ch) {
    memset(c, 0, sizeof(*c));
    c->ch = ch;
    c->state_idx = ch;                   // C = I: output k is state k
    lpf_init(&c->lpf);
    c->window = REC_N_DEFAULT[ch];
    c->T_on   = REC_TON_DEFAULT[ch];
    c->T_off  = REC_TOFF_DEFAULT[ch];
    c->K_safe = REC_KSAFE_DEFAULT;
}

// Optional recovery action (Alg.1 line 23) — e.g. GCS alert / safe landing.
typedef void (*recovery_action_fn)(int ch);
static recovery_action_fn g_recovery_action = 0;   // default: no-op

// ── Algorithm 1 lines 8-24 for one sensor channel ───────────────────────────
// ms_raw: convert(y) — the software-sensor prediction for this sensor (line 9).
// m_raw : raw physical sensor reading (possibly under attack).
// Returns the measurement the control loop should use (raw if healthy, ms if
// recovering — Fig. 3).
static inline float recovery_check_ms(RecoveryModel* mdl, RecoveryChannel* c,
                                      float ms_raw, float m_raw) {
    float m = lpf_step(&c->lpf, m_raw);                     // line 8

    // err_hist holds at most REC_WINDOW_CAP samples
    const int w = (c->window <= REC_WINDOW_CAP) ? c->window : REC_WINDOW_CAP;

    c->t++;                                                  // line 10

    if (!c->recovery_mode && c->t > c->window) {             // line 11 (checkpoint)
        c->t = 0;                                            // line 12
        c->r = 0.0f;                                         // line 13
        float sum = 0.0f;                                    // line 14:
        for (int i = 0; i < w; i++) sum += c->err_hist[i];
        c->e = sum / (float)w;                               //   e = avg(ms_raw - m)
        ms_raw = m;                                          // line 15: sync
        if (c->state_idx >= 0)                               //   feed synchronized
            mdl->x[c->state_idx] =                           //   reading into model
                c->inv_convert ? c->inv_convert(m) : m;
    }

    float ms = ms_raw - c->e;                                // line 16

    // store the raw-prediction error for the NEXT window's error_estimation
    c->err_hist[c->t % w] = ms_raw - m;

    c->r += fabsf(m - ms);                                   // line 17

    if (c->r > c->T_on) {                                    // line 18
        c->recovery_mode = true;
        c->safe_count    = 0;
    }

    if (c->recovery_mode) {                                  // line 19
        if (c->r < c->T_off)                                 // line 21
            c->safe_count++;
        if (c->safe_count > c->K_safe)                       // line 22
            c->recovery_mode = false;
        if (g_recovery_action)                               // line 23
            g_recovery_action(c->ch);
        return ms;                                           // line 20: m <- ms
    }
    return m_raw;                                            // healthy (Fig. 3)
}

// Identity conversion (state channels: gyro p/q/r, attitude, position, velocity —
// §3.2: gyro and GPS measurements are acquired directly from the model state).
static inline float recovery_check(RecoveryModel* mdl, RecoveryChannel* c,
                                   float m_raw) {
    return recovery_check_ms(mdl, c, mdl->y[c->ch], m_raw);
}

// ── Model output channel indices (Eq. 3 order, C = I) ────────────────────────
#define CH_PN    0
#define CH_PE    1
#define CH_ALT   2
#define CH_PHI   3
#define CH_THETA 4
#define CH_PSI   5
#define CH_VN    6
#define CH_VE    7
#define CH_VUP   8
#define CH_P     9
#define CH_Q     10
#define CH_R     11
