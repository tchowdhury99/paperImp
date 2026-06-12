// test_recovery.cpp — Algorithm 1 conformance tests (Choi et al. RAID 2020)
//
// Each test pins ONE paper-mandated behavior of recovery_monitor.h.
// Build:  g++ -O2 -std=c++14 -DRECOVERY_TEST_MODEL -o test_recovery test_recovery.cpp -lm
//
// RECOVERY_TEST_MODEL makes recovery_monitor.h use a tiny known 2-state test model
// instead of model_matrices.h, so tests are independent of the identified model.

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

#define RECOVERY_TEST_MODEL
#include "recovery_monitor.h"
#include "software_sensors.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; printf("  PASS %s\n", msg); } \
    else      { g_fail++; printf("  FAIL %s\n", msg); } } while (0)

// Test model (set by recovery_monitor.h under RECOVERY_TEST_MODEL):
//   NX=NY=2, NU=1, stable 2nd order, C=I, D=0.
// Channel 0 is used as "the sensor" in most tests.

static void reset_channel_params(RecoveryChannel* c, int window,
                                 float ton, float toff, int ksafe) {
    c->window = window; c->T_on = ton; c->T_off = toff; c->K_safe = ksafe;
}

// ── T1: filter(m) exists — Algorithm 1 line 8 ───────────────────────────────
static void T1_lowpass_filter() {
    printf("== T1: low-pass filter on real measurement (Alg.1 l.8) ==\n");
    LPFilter f; lpf_init(&f);
    // DC convergence
    float y = 0;
    for (int i = 0; i < 500; i++) y = lpf_step(&f, 1.0f);
    CHECK(fabsf(y - 1.0f) < 0.02f, "LPF converges to DC");
    // attenuation of alternating-sign (Nyquist) noise
    LPFilter f2; lpf_init(&f2);
    float maxout = 0;
    for (int i = 0; i < 500; i++) {
        float v = lpf_step(&f2, (i % 2) ? 1.0f : -1.0f);
        if (i > 100 && fabsf(v) > maxout) maxout = fabsf(v);
    }
    CHECK(maxout < 0.1f, "LPF attenuates Nyquist noise by >10x");
}

// ── T2: error compensation sign — Alg.1 l.14+16, §3.3 wind example ─────────
// If the model prediction is consistently HIGHER than the real measurement by a
// constant offset (the paper's wind case), then after one checkpoint the
// compensated prediction ms must move TOWARD the measurement, shrinking the
// per-window residual.
static void T2_error_compensation_sign() {
    printf("== T2: e = avg(ms - m), ms <- ms - e (paper wind example) ==\n");
    RecoveryModel mdl; RecoveryChannel ch;
    recovery_model_init(&mdl); recovery_channel_init(&ch, 0);
    reset_channel_params(&ch, 50, 1e9f, 1e9f, 10);   // huge T_on: never trigger

    float u[NU] = {0};
    // measurement = model output - 0.5 (model consistently over-predicts).
    // With a constant offset, the error-compensation term e must track it (correct
    // sign) so the per-window residual stays SMALL. A wrong sign would double the
    // offset and make the residual grow unboundedly.
    float r_peak = 0;
    for (int i = 0; i < 200; i++) {
        recovery_model_update(&mdl, u);
        float m = mdl.y[0] - 0.5f;
        recovery_check(&mdl, &ch, m);
        if (i > 60 && ch.r > r_peak) r_peak = ch.r;   // after first checkpoint
    }
    printf("  peak per-window residual after compensation = %.3f\n", r_peak);
    // window = 50; a wrong-sign e would DOUBLE the 0.5 offset -> r ~ 50/window.
    // Correct compensation keeps r near the post-resync LPF-transient floor (~2).
    CHECK(r_peak < 6.0f, "constant offset compensated (correct sign keeps r small)");
}

// ── T3: checkpoint gated on !recovery_mode — Alg.1 l.11 ────────────────────
static void T3_no_checkpoint_during_recovery() {
    printf("== T3: no r reset / no checkpoint while in recovery (Alg.1 l.11) ==\n");
    RecoveryModel mdl; RecoveryChannel ch;
    recovery_model_init(&mdl); recovery_channel_init(&ch, 0);
    reset_channel_params(&ch, 50, 5.0f, 3.0f, 10);

    float u[NU] = {0};
    bool exited_during_attack = false;
    bool entered = false;
    // attack: constant +2.0 offset for 6 windows
    for (int i = 0; i < 300; i++) {
        recovery_model_update(&mdl, u);
        float m = mdl.y[0] + 2.0f;
        recovery_check(&mdl, &ch, m);
        if (ch.recovery_mode) entered = true;
        if (entered && !ch.recovery_mode) exited_during_attack = true;
    }
    CHECK(entered, "attack detected");
    CHECK(!exited_during_attack, "recovery persists across window boundaries under attack");
    CHECK(ch.t > ch.window, "t keeps counting during recovery (checkpoint deferred)");
}

// ── T4: checkpoint actions — Alg.1 l.12-15 + §3.3 model-state resync ────────
static void T4_checkpoint_sync() {
    printf("== T4: checkpoint: t<-0, r<-0, e update, ms<-m, state resync ==\n");
    RecoveryModel mdl; RecoveryChannel ch;
    recovery_model_init(&mdl); recovery_channel_init(&ch, 0);
    reset_channel_params(&ch, 50, 1e9f, 1e9f, 10);

    float u[NU] = {0};
    mdl.x[0] = 5.0f;          // model state far from measurement (m ~ 0)
    float r_before = 0;
    for (int i = 0; i <= 50; i++) {
        if (i == 50) {
            mdl.x[0] = 5.0f;  // re-poison the state right before the checkpoint
            r_before = ch.r;  // accumulated residual of window 1
        }
        recovery_model_update(&mdl, u);
        recovery_check(&mdl, &ch, 0.0f);  // real measurement constant 0
    }
    // call i=50 made t reach 51 > window=50 -> checkpoint executed inside it
    CHECK(ch.t == 0, "t reset at checkpoint (l.12)");
    CHECK(r_before > 10.0f && ch.r < 0.1f * r_before,
          "r restarted from 0 at checkpoint (l.13)");
    CHECK(fabsf(ch.e) > 0.1f, "error compensation estimated from prev window (l.14)");
    CHECK(fabsf(mdl.x[0]) < 0.5f,
          "model state re-seeded from real reading (l.15 / s3.3)");
}

// ── T5: safe_count semantics — Alg.1 l.21-22 (no extra resets) ──────────────
static void T5_safe_count() {
    printf("== T5: safe_count only ++ under T_off; reset only on trigger (l.18) ==\n");
    RecoveryChannel ch; recovery_channel_init(&ch, 0);
    reset_channel_params(&ch, 1000, 10.0f, 8.0f, 3);
    RecoveryModel mdl; recovery_model_init(&mdl);
    float u[NU] = {0};

    // clean warmup so priming calibrates e to the unattacked baseline
    for (int i = 0; i < 5; i++) { recovery_model_update(&mdl, u);
                                  recovery_check(&mdl, &ch, mdl.y[0]); }
    // force into recovery with a burst
    for (int i = 0; i < 10; i++) { recovery_model_update(&mdl, u);
                                   recovery_check(&mdl, &ch, mdl.y[0] + 5.0f); }
    CHECK(ch.recovery_mode, "entered recovery");
    // drain the LPF with clean readings (keep r above T_off so nothing changes)
    for (int i = 0; i < 40; i++) {
        ch.r = 20.0f;
        recovery_model_update(&mdl, u);
        recovery_check(&mdl, &ch, mdl.y[0]);
    }
    CHECK(ch.recovery_mode && ch.safe_count == 0, "still in recovery, no safe counts");
    // paper does NOT reset safe_count when r >= T_off: alternate r around T_off;
    // dips must accumulate to K (=3) and clear recovery even though every other
    // tick is above T_off.
    int cleared = 0;
    for (int i = 0; i < 10; i++) {
        ch.r = (i % 2 == 0) ? 7.0f : 9.0f;   // below/above T_off alternating
        recovery_model_update(&mdl, u);
        recovery_check(&mdl, &ch, mdl.y[0]);
        if (!ch.recovery_mode) { cleared = 1; break; }
    }
    CHECK(cleared == 1, "intermittent below-T_off ticks accumulate to K and clear");
}

// ── T6: detection + substitution — Alg.1 l.18-20, Fig. 3 ───────────────────
static void T6_detect_and_substitute() {
    printf("== T6: bias attack detected; output switches to software sensor ==\n");
    RecoveryModel mdl; RecoveryChannel ch;
    recovery_model_init(&mdl); recovery_channel_init(&ch, 0);
    reset_channel_params(&ch, 50, 5.0f, 3.0f, 10);

    float u[NU] = {0};
    // healthy phase
    for (int i = 0; i < 100; i++) { recovery_model_update(&mdl, u);
                                    recovery_check(&mdl, &ch, mdl.y[0]); }
    CHECK(!ch.recovery_mode, "no false positive on clean data");
    // attack
    int detect_at = -1;
    float out = 0, m_attacked = 0;
    for (int i = 0; i < 200; i++) {
        recovery_model_update(&mdl, u);
        m_attacked = mdl.y[0] + 2.0f;
        out = recovery_check(&mdl, &ch, m_attacked);
        if (ch.recovery_mode && detect_at < 0) detect_at = i;
    }
    CHECK(detect_at >= 0, "attack detected");
    printf("  detected after %d ticks\n", detect_at);
    CHECK(fabsf(out - m_attacked) > 1.0f, "output is software sensor, not attacked reading");
}

// ── T7: healthy passthrough — Fig. 3 (raw reading reaches control loop) ─────
static void T7_healthy_passthrough() {
    printf("== T7: healthy channel returns the RAW measurement (Fig. 3) ==\n");
    RecoveryModel mdl; RecoveryChannel ch;
    recovery_model_init(&mdl); recovery_channel_init(&ch, 0);
    reset_channel_params(&ch, 50, 1e9f, 1e9f, 10);
    float u[NU] = {0};
    bool raw_passthrough = true;
    for (int i = 0; i < 80; i++) {
        recovery_model_update(&mdl, u);
        float m_raw = mdl.y[0] + ((i % 2) ? 0.3f : -0.3f);   // noisy raw reading
        float out = recovery_check(&mdl, &ch, m_raw);
        if (out != m_raw) raw_passthrough = false;
    }
    CHECK(raw_passthrough, "healthy: control loop receives raw (unfiltered) reading");
}

// ── T8: software-sensor conversion equations (Eq. 4, 5, 6, 11) ──────────────
static void T8_conversions() {
    printf("== T8: conversion equations ==\n");
    // Eq. 5 barometer: at z = h0, Ph = P0; pressure decreases with altitude
    float p0 = software_baro(0.0f, 0.0f, 101325.0f, 288.15f);
    float p100 = software_baro(100.0f, 0.0f, 101325.0f, 288.15f);
    CHECK(fabsf(p0 - 101325.0f) < 1.0f, "Eq.5: P(h0) = P0");
    CHECK(p100 < p0 && p100 > 0.98f * p0, "Eq.5: ~1.2 kPa drop over 100 m");

    // Eq. 4 / Holoborodko (causal 5-point, derivative at n-2): exact on a ramp.
    // The previous 4-point form had a 25% gain error (0.75*slope on a ramp).
    float dt = 0.02f;
    float buf[5] = {3*4*dt, 3*3*dt, 3*2*dt, 3*1*dt, 0.0f};  // f[n] ... f[n-4]
    float d = holoborodko_deriv(buf, dt);
    CHECK(fabsf(d - 3.0f) < 1e-4f, "Eq.4: differentiator exact on ramp");
    // and it must strongly attenuate Nyquist noise (the reason the paper uses it)
    float bufn[5] = {1.0f, -1.0f, 1.0f, -1.0f, 1.0f};
    float dn = holoborodko_deriv(bufn, dt);
    float bufd[5] = {2*4*dt, 2*3*dt, 2*2*dt, 2*1*dt, 0.0f};
    CHECK(fabsf(dn) < fabsf(holoborodko_deriv(bufd, dt)),
          "Eq.4: noise-robust (Nyquist gain below ramp gain)");

    // Eq. 6 heading: level vehicle, field pointing north -> H = 0
    float H = software_mag_heading(1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    CHECK(fabsf(H) < 1e-6f, "Eq.6: north field, level -> heading 0");
    H = software_mag_heading(0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    CHECK(fabsf(H + (float)M_PI/2) < 1e-6f, "Eq.6: east field, level -> -pi/2");

    // Eq. 11 supplementary: level, 1 g (ArduPilot body z: AccZ ~ -9.8 level)
    float phi_a, th_a, psi_m;
    supplementary_compensation(0.0f, 0.0f, -9.81f, 1.0f, 0.0f, 0.0f,
                               &phi_a, &th_a, &psi_m);
    CHECK(fabsf(phi_a) < 1e-6f && fabsf(th_a) < 1e-6f, "Eq.11: level -> phi=theta=0");
    CHECK(fabsf(psi_m) < 1e-6f, "Eq.11: north mag field -> psi=0");
}

int main() {
    T1_lowpass_filter();
    T2_error_compensation_sign();
    T3_no_checkpoint_during_recovery();
    T4_checkpoint_sync();
    T5_safe_count();
    T6_detect_and_substitute();
    T7_healthy_passthrough();
    T8_conversions();
    printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
