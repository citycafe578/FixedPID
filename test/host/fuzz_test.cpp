// Randomized tests for FixedPID, meant to be built with ASan + UBSan
// (see the Makefile). Four checks:
//   1. exact:     first call of every update path vs. a 128-bit reference model
//   2. robust:    random configs and call sequences; outputs stay inside the
//                 limits and the sanitizers see no overflow / UB
//   3. identical: variable-dt and fixed-rate paths agree at divisible rates
//   4. reset:     reset() makes an instance behave like a fresh one
//   5. exactDoM:  derivative-on-measurement: first call has no D, second call
//                 vs. the 128-bit reference model (all four paths)
//   Checks 2-4 also run with derivative-on-measurement switched on at random.
//
// usage: fuzz_test [scale] [seed]     (scale 1 ~ a few seconds)
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "FixedPID.h"

typedef __int128 i128;

static uint64_t rngState = 88172645463325252ULL;
static uint64_t rnd() {
    rngState ^= rngState << 13;
    rngState ^= rngState >> 7;
    rngState ^= rngState << 17;
    return rngState;
}

static int32_t pick32() {
    switch (rnd() % 8) {
        case 0: return INT32_MIN;
        case 1: return INT32_MAX;
        case 2: return 0;
        case 3: return 1;
        case 4: return -1;
        case 5: return (int32_t)(rnd() % 2001) - 1000;
        default: return (int32_t)rnd();
    }
}

static int32_t pickGain() {          // non-negative
    switch (rnd() % 6) {
        case 0: return INT32_MAX;
        case 1: return 0;
        case 2: return 1;
        case 3: return 1000;
        case 4: return (int32_t)(rnd() % 100000);
        default: return (int32_t)(rnd() & 0x7fffffff);
    }
}

static uint32_t pickDt() {
    uint32_t v;
    switch (rnd() % 7) {
        case 0: v = 1; break;
        case 1: v = 1000; break;
        case 2: v = 1000000; break;
        case 3: v = UINT32_MAX; break;
        case 4: v = 1 + (uint32_t)(rnd() % 100); break;
        default: v = (uint32_t)rnd(); break;
    }
    return v == 0 ? 1 : v;
}

static uint32_t pickHz() {
    static const uint32_t common[] = {1, 3, 500, 1000, 3000, 4000, 300000, 1000000};
    if (rnd() % 2) return common[rnd() % 8];
    return 1 + (uint32_t)(rnd() % 1000000);
}

static long failures = 0;
#define CHECK(cond, ...)                                              \
    do {                                                              \
        if (!(cond)) {                                                \
            if (failures < 20) { printf("FAIL: " __VA_ARGS__); puts(""); } \
            failures++;                                               \
        }                                                             \
    } while (0)

static i128 clampI(i128 v, i128 lo, i128 hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Cfg {
    int32_t kp, ki, kd, outMin, outMax, intMin, intMax;
};

static void randomCfg(Cfg& c, bool nonNegative) {
    c.kp = nonNegative ? pickGain() : pick32();
    c.ki = nonNegative ? pickGain() : pick32();
    c.kd = nonNegative ? pickGain() : pick32();

    c.outMin = INT32_MIN; c.outMax = INT32_MAX;
    c.intMin = INT32_MIN; c.intMax = INT32_MAX;
    if (rnd() % 2) {
        int32_t a = pick32(), b = pick32();
        c.outMin = a < b ? a : b;
        c.outMax = a < b ? b : a;
    }
    if (rnd() % 2) {
        int32_t a = pick32(), b = pick32();
        c.intMin = a < b ? a : b;
        c.intMax = a < b ? b : a;
    }
}

static void apply(FixedPID& p, const Cfg& c) {
    p.setTunings(c.kp, c.ki, c.kd);
    p.setOutputLimits(c.outMin, c.outMax);
    p.setIntegralLimits(c.intMin, c.intMax);
}

// ---- 128-bit reference models of the FIRST call after construction ----------

static bool withinThreshold(i128 e) { return e <= INT32_MAX && e >= -(i128)INT32_MAX; }

static int32_t refPosition(const Cfg& c, int32_t t, int32_t in, uint32_t dt, bool fixed, uint32_t hz) {
    i128 e = (i128)t - in;
    i128 P = (i128)c.kp * e / 1000;

    i128 inc;
    if (!fixed) {
        inc = e * (i128)dt;
    } else {
        i128 dtu = 1000000u / hz, rem = 1000000u % hz;
        inc = e * dtu + (rem ? (e * rem) / hz : 0);
    }
    i128 raw = (c.ki != 0 && withinThreshold(e)) ? (i128)c.ki * inc / 1000000000LL : 0;
    i128 I = c.ki != 0 ? clampI(raw, c.intMin, c.intMax) : 0;

    i128 D = 0;
    if (c.kd != 0) D = fixed ? e * ((i128)c.kd * hz) / 1000 : (i128)c.kd * e * 1000 / dt;

    return (int32_t)clampI(P + I + D, c.outMin, c.outMax);
}

static int32_t refVelocity(const Cfg& c, int32_t t, int32_t in, uint32_t dt, bool fixed, uint32_t hz) {
    i128 e = (i128)t - in;
    i128 dP = (i128)c.kp * e / 1000;

    i128 term;
    if (!fixed) {
        term = (i128)c.ki * e * (i128)dt;
    } else {
        i128 dtu = 1000000u / hz, rem = 1000000u % hz;
        term = (i128)c.ki * e * dtu + (rem ? ((i128)c.ki * e * rem) / hz : 0);
    }
    i128 dI = withinThreshold(e) ? term / 1000000000LL : 0;

    i128 dD = 0;
    if (c.kd != 0) dD = fixed ? e * ((i128)c.kd * hz) / 1000 : (i128)c.kd * e * 1000 / dt;

    return (int32_t)clampI(dP + dI + dD, c.outMin, c.outMax);
}

static void testExact(long n) {
    for (long i = 0; i < n; i++) {
        Cfg c;
        randomCfg(c, true);
        int32_t t = pick32(), in = pick32();
        uint32_t dt = pickDt(), hz = pickHz();

        { FixedPID p; apply(p, c);
          int32_t got = p.update(t, in, dt), want = refPosition(c, t, in, dt, false, 0);
          CHECK(got == want, "update kp=%d ki=%d kd=%d t=%d in=%d dt=%u got=%d want=%d",
                c.kp, c.ki, c.kd, t, in, dt, got, want); }

        { FixedPID p; apply(p, c); p.setFrequency(hz);
          int32_t got = p.updateFixedRate(t, in), want = refPosition(c, t, in, 0, true, hz);
          CHECK(got == want, "updateFixedRate kp=%d ki=%d kd=%d t=%d in=%d hz=%u got=%d want=%d",
                c.kp, c.ki, c.kd, t, in, hz, got, want); }

        { FixedPID p; apply(p, c);
          int32_t got = p.updateVelocity(t, in, dt), want = refVelocity(c, t, in, dt, false, 0);
          CHECK(got == want, "updateVelocity kp=%d ki=%d kd=%d t=%d in=%d dt=%u got=%d want=%d",
                c.kp, c.ki, c.kd, t, in, dt, got, want); }

        { FixedPID p; apply(p, c); p.setFrequency(hz);
          int32_t got = p.updateVelocityFixedRate(t, in), want = refVelocity(c, t, in, 0, true, hz);
          CHECK(got == want, "updateVelocityFixedRate kp=%d ki=%d kd=%d t=%d in=%d hz=%u got=%d want=%d",
                c.kp, c.ki, c.kd, t, in, hz, got, want); }
    }
}

// ---- derivative on measurement: exact second call, no D on the first ----------

static void testExactDoM(long n) {
    for (long i = 0; i < n; i++) {
        Cfg c;
        randomCfg(c, false);
        c.kp = 0; c.ki = 0;                       // only the derivative term
        int32_t t1 = pick32(), in1 = pick32(), t2 = pick32(), in2 = pick32();
        uint32_t dt = pickDt(), hz = pickHz();
        Cfg none = c; none.kd = 0;

        for (int path = 0; path < 4; path++) {
            FixedPID p; apply(p, c); p.setFrequency(hz);
            p.setDerivativeOnMeasurement(true);
            bool fixed = path == 1 || path == 3, vel = path >= 2;
            int32_t first  = path == 0 ? p.update(t1, in1, dt) : path == 1 ? p.updateFixedRate(t1, in1)
                           : path == 2 ? p.updateVelocity(t1, in1, dt) : p.updateVelocityFixedRate(t1, in1);
            int32_t second = path == 0 ? p.update(t2, in2, dt) : path == 1 ? p.updateFixedRate(t2, in2)
                           : path == 2 ? p.updateVelocity(t2, in2, dt) : p.updateVelocityFixedRate(t2, in2);
            // first call: no derivative at all. second call: D of the input change, e = in1 - in2
            int32_t want1 = vel ? refVelocity(none, t1, in1, dt, fixed, hz) : refPosition(none, t1, in1, dt, fixed, hz);
            int32_t want2;
            if (vel) {   // velocity form accumulates onto the (clamped) output of the first call
                i128 e = (i128)in1 - in2;
                i128 dD = c.kd == 0 ? 0 : (fixed ? e * ((i128)c.kd * hz) / 1000 : (i128)c.kd * e * 1000 / dt);
                want2 = (int32_t)clampI((i128)first + dD, c.outMin, c.outMax);
            } else {
                want2 = refPosition(c, in1, in2, dt, fixed, hz);   // e = in1 - in2 gives the same D
            }
            (void)t2;
            CHECK(first == clampI(0, c.outMin, c.outMax) && want1 == first,
                  "DoM first call path=%d kd=%d got=%d", path, c.kd, first);
            CHECK(second == want2, "DoM second path=%d kd=%d in1=%d in2=%d dt=%u hz=%u got=%d want=%d",
                  path, c.kd, in1, in2, dt, hz, second, want2);
        }
    }
}

// ---- robustness: anything goes, outputs stay in the limits, no UB ----------

static void testRobust(long sequences) {
    for (long s = 0; s < sequences; s++) {
        Cfg c;
        randomCfg(c, false);

        FixedPID a, b, v, w;
        FixedPID* all[4] = {&a, &b, &v, &w};
        uint32_t hz = pickHz();
        for (int k = 0; k < 4; k++) {
            apply(*all[k], c);
            all[k]->setFrequency(hz);
            all[k]->setErrorDeadband((uint32_t)pick32());
            all[k]->setErrorIntegralThreshold(pick32());
            all[k]->setDerivativeFilter((uint32_t)(rnd() % 1100));
        }
        bool dom = rnd() % 2;
        for (int k = 0; k < 4; k++) all[k]->setDerivativeOnMeasurement(dom);

        for (int step = 0; step < 40; step++) {
            int32_t t = pick32(), in = pick32();
            uint32_t dt = pickDt();
            int32_t out[4] = {
                a.update(t, in, dt), b.updateFixedRate(t, in),
                v.updateVelocity(t, in, dt), w.updateVelocityFixedRate(t, in)};
            for (int k = 0; k < 4; k++) {
                CHECK(out[k] >= c.outMin && out[k] <= c.outMax,
                      "robust path=%d out=%d limits=[%d,%d]", k, out[k], c.outMin, c.outMax);
            }
            CHECK(a.update(t, in, 0) == 0, "dt=0 must return 0");
            if (rnd() % 17 == 0) { for (int k = 0; k < 4; k++) all[k]->reset(); }
            if (rnd() % 13 == 0) { dom = !dom; for (int k = 0; k < 4; k++) all[k]->setDerivativeOnMeasurement(dom); }
        }
    }
}

// ---- variable-dt vs fixed-rate must agree at rates that divide 1 MHz --------

static void testIdentical(long sequences) {
    static const uint32_t rates[] = {500, 1000, 2000, 4000, 10000};
    for (long s = 0; s < sequences; s++) {
        uint32_t hz = rates[rnd() % 5];
        uint32_t dt = 1000000u / hz;

        int32_t kp = (int32_t)(rnd() % 10000000), ki = (int32_t)(rnd() % 10000000), kd = (int32_t)(rnd() % 10000000);
        int32_t lim = 1 + (int32_t)(rnd() % 1000000);
        uint32_t db = (uint32_t)(rnd() % 20), alpha = (uint32_t)(rnd() % 1001);
        int32_t thr = 1 + (int32_t)(rnd() % 1000000);

        FixedPID a, b, v, w;
        FixedPID* all[4] = {&a, &b, &v, &w};
        for (int k = 0; k < 4; k++) {
            all[k]->setTunings(kp, ki, kd);
            all[k]->setOutputLimits(-lim * 100, lim * 100);
            all[k]->setIntegralLimits(-lim, lim);
            all[k]->setFrequency(hz);
            all[k]->setErrorDeadband(db);
            all[k]->setErrorIntegralThreshold(thr);
            all[k]->setDerivativeFilter(alpha);
        }
        bool dom = rnd() % 2;
        for (int k = 0; k < 4; k++) all[k]->setDerivativeOnMeasurement(dom);
        for (int step = 0; step < 100; step++) {
            int32_t t = (int32_t)(rnd() % 2000001) - 1000000;
            int32_t in = (int32_t)(rnd() % 2000001) - 1000000;
            int32_t x = a.update(t, in, dt), y = b.updateFixedRate(t, in);
            int32_t p = v.updateVelocity(t, in, dt), q = w.updateVelocityFixedRate(t, in);
            CHECK(x == y, "identical position hz=%u step=%d %d vs %d", hz, step, x, y);
            CHECK(p == q, "identical velocity hz=%u step=%d %d vs %d", hz, step, p, q);
        }
    }
}

// ---- reset() restores a pristine state ------------------------------------

static void testReset(long sequences) {
    for (long s = 0; s < sequences; s++) {
        Cfg c;
        randomCfg(c, false);
        uint32_t hz = pickHz(), alpha = (uint32_t)(rnd() % 1100);

        FixedPID used, fresh;
        FixedPID* both[2] = {&used, &fresh};
        bool dom = rnd() % 2;
        for (int k = 0; k < 2; k++) {
            both[k]->setDerivativeOnMeasurement(dom);
            apply(*both[k], c);
            both[k]->setFrequency(hz);
            both[k]->setDerivativeFilter(alpha);
        }
        for (int i = 0; i < 30; i++) {
            used.update(pick32(), pick32(), pickDt());
            used.updateFixedRate(pick32(), pick32());
            used.updateVelocity(pick32(), pick32(), pickDt());
            used.updateVelocityFixedRate(pick32(), pick32());
        }
        used.reset();

        for (int i = 0; i < 20; i++) {
            int32_t t = pick32(), in = pick32();
            uint32_t dt = pickDt();
            int sel = (int)(rnd() % 4);
            int32_t x, y;
            if (sel == 0)      { x = used.update(t, in, dt);              y = fresh.update(t, in, dt); }
            else if (sel == 1) { x = used.updateFixedRate(t, in);         y = fresh.updateFixedRate(t, in); }
            else if (sel == 2) { x = used.updateVelocity(t, in, dt);      y = fresh.updateVelocity(t, in, dt); }
            else               { x = used.updateVelocityFixedRate(t, in); y = fresh.updateVelocityFixedRate(t, in); }
            CHECK(x == y, "reset path=%d step=%d %d vs %d", sel, i, x, y);
        }
    }
}

int main(int argc, char** argv) {
    long scale = argc > 1 ? atol(argv[1]) : 1;
    if (scale < 1) scale = 1;
    if (argc > 2) rngState ^= (uint64_t)atoll(argv[2]) * 0x9E3779B97F4A7C15ULL;

    testExact(300000 * scale);
    testExactDoM(100000 * scale);
    testRobust(30000 * scale);
    testIdentical(4000 * scale);
    testReset(20000 * scale);

    if (failures) {
        printf("fuzz: %ld FAILURES\n", failures);
        return 1;
    }
    printf("fuzz: OK (scale %ld)\n", scale);
    return 0;
}
