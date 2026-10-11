# FixedPID Usage Guide

[English](USAGE.md) | [繁體中文](USAGE.zh-TW.md)

This guide explains how to use FixedPID in a real project: the units, which update function to call, what every setting does, how the numbers behave at the edges, and some copy-and-paste recipes. For a feature overview and benchmarks, see the [README](../README.md).

## Contents

1. [Install](#1-install)
2. [Quick start](#2-quick-start)
3. [Units and scaling](#3-units-and-scaling)
4. [Choosing an update function](#4-choosing-an-update-function)
5. [Settings reference](#5-settings-reference)
6. [Limits and anti-windup](#6-limits-and-anti-windup)
7. [Derivative filter](#7-derivative-filter)
8. [Velocity form](#8-velocity-form)
9. [Input ranges and numeric behavior](#9-input-ranges-and-numeric-behavior)
10. [Recipes](#10-recipes)
11. [Tuning workflow](#11-tuning-workflow)
12. [Troubleshooting](#12-troubleshooting)
13. [Using it from interrupts and tasks](#13-using-it-from-interrupts-and-tasks)
14. [Running the tests](#14-running-the-tests)

---

## 1. Install

* **Arduino IDE / PlatformIO (manual):** copy `src/FixedPID.h` and `src/FixedPID.cpp` into your project, or put the repository folder into your `libraries/` directory.
* **PlatformIO (from GitHub):**

  ```ini
  lib_deps = https://github.com/citycafe578/FixedPID.git
  ```

Then `#include "FixedPID.h"`. The library only needs `<stdint.h>` and a GCC or Clang compatible compiler (it uses `__builtin_add_overflow` / `__builtin_mul_overflow`). The update functions use no floating point, heap allocation or Arduino API (only the optional `setTuningsFloat()` setup helper converts floats).

FixedPID targets 32-bit MCUs. It has been tested on the ESP32. 8-bit AVR boards are not a target platform.

## 2. Quick start

```cpp
#include "FixedPID.h"

FixedPID pid;

void setup() {
  pid.setTunings(1500, 200, 50);        // Kp = 1.5, Ki = 0.2 /s, Kd = 0.05 s
  pid.setOutputLimits(-10000, 10000);
  pid.setIntegralLimits(-5000, 5000);
}

void loop() {
  static uint32_t last = micros();
  uint32_t now = micros();
  uint32_t dt_us = now - last;          // unsigned subtraction survives micros() wrap-around
  last = now;

  int32_t output = pid.update(targetValue, measuredValue, dt_us);
  // ... drive the actuator with `output`
}
```

If your loop runs at a fixed rate (hardware timer, RTOS tick), use the fixed-rate form instead and skip `dt_us`:

```cpp
pid.setFrequency(1000);                 // 1 kHz
int32_t output = pid.updateFixedRate(targetValue, measuredValue);
```

## 3. Units and scaling

FixedPID works on integers only. Everything is expressed in these units:

| Quantity | Type | Meaning |
| --- | --- | --- |
| `target`, `input` | `int32_t` | Any unit you like (RPM, encoder ticks, 0.01 °C, ...). Use the same unit for both. |
| error | internal `int64_t` | `target - input`. The full range is 33 bits (`INT32_MIN - INT32_MAX` fits). |
| `kp`, `ki`, `kd` | `int32_t` | The real gain **multiplied by `PID_SCALE` = 1000**. |
| `dt_us` | `uint32_t` | Time step in **microseconds**. |
| output | `int32_t` | In the same unit as the actuator command you want. |

The controller computes

```
output = Kp * e  +  Ki * integral(e dt)  +  Kd * de/dt
```

with `e = target - input`, where `Kp`, `Ki`, `Kd` are the real (floating-point) gains and the stored values are `K * 1000`:

| Term | Real gain unit | Stored value | Example |
| --- | --- | --- | --- |
| P | output per error unit | `Kp * 1000` | `Kp = 1.5` → `1500` |
| I | output per (error unit · second) | `Ki * 1000` | `Ki = 0.2 /s` → `200` |
| D | output per (error unit / second) | `Kd * 1000` | `Kd = 0.05 s` → `50` |

Worked checks:

* P: error `100`, `kp = 1500` → `1500 * 100 / 1000 = 150`.
* I: error `100` held for 1 second, `ki = 200` → `0.2 * 100 * 1 = 20`.
* D: the error jumps by `10` within 1 ms, `kd = 50` → `0.05 * 10 / 0.001 = 500`.

Gains below `0.001` cannot be represented. If you need a very small gain, scale your error or output units instead.

In `setup()` you can also write the gains as plain numbers: `setTuningsFloat(1.5f, 0.2f, 0.05f)` (see [Settings reference](#5-settings-reference)).

## 4. Choosing an update function

| Function | Use it when |
| --- | --- |
| `update(target, input, dt_us)` | The loop period varies (typical for loops driven by `loop()` or sensor interrupts). You pass the measured `dt_us` every call. |
| `updateFixedRate(target, input)` | The loop is driven by a timer or RTOS tick at a fixed frequency. Call `setFrequency(hz)` once first. |
| `updateVelocity(target, input, dt_us)` | The actuator expects *changes* (incremental actuators). Variable `dt`. See [Velocity form](#8-velocity-form). |
| `updateVelocityFixedRate(target, input)` | Same, at a fixed frequency. |

All four return the new controller output, limited by `setOutputLimits()`. At rates that divide 1,000,000 evenly (500, 1000, 2000, 4000, ... Hz) `update(.., 1000000 / hz)` and `updateFixedRate(..)` return exactly the same values for inputs in the normal working range (the fuzz test checks this).

Do not mix position-form and velocity-form calls on one instance: they keep different internal state. Use one `FixedPID` per control loop.

## 5. Settings reference

Defaults are what a freshly constructed `FixedPID` uses.

| Function | Default | What it does |
| --- | --- | --- |
| `setTunings(kp, ki, kd)` | `0, 0, 0` | Gains, each `real * 1000`. With all zeros the output is always `0`. |
| `setOutputLimits(min, max)` | `INT32_MIN, INT32_MAX` | Clamps the returned output. If `min > max` the two are swapped. |
| `setIntegralLimits(min, max)` | `INT32_MIN, INT32_MAX` | Clamps the **integral term's contribution to the output** (output units, not error units). Swapped if `min > max`. Position form only. |
| `setTuningsFloat(kp, ki, kd)` | – | Same as `setTunings()` but takes plain numbers (`1.5f`), rounded to 0.001. Uses floating point only inside this call, so use it in `setup()`. Returns `false` if a gain is NaN (set to 0) or outside ±2,147,483.647 (saturated); the other gains are still applied. |
| `setFrequency(hz)` | not set | Needed by the `*FixedRate` functions. Valid range 1 – 1,000,000 Hz. Returns `true` on success. |
| `setErrorDeadband(edb)` | `0` | If `|error| <= edb` the error is treated as `0` (P, I and D all see zero). |
| `setErrorIntegralThreshold(eit)` | `INT32_MAX` | The integral only accumulates while `|error| <= eit`. Outside it the integral is frozen (not reset). |
| `setDerivativeFilter(alpha)` | `1000` (off) | Low-pass filter on the derivative term, `0 – 1000`. See [Derivative filter](#7-derivative-filter). |
| `setDerivativeOnMeasurement(enable)` | `false` | `true`: the D term acts on `-d(input)/dt` instead of the error. See [Derivative filter](#7-derivative-filter). |
| `reset()` | – | Clears every piece of internal state (integral, previous errors, velocity output, filter, remainders). Settings are kept. |

Things to know:

* `setFrequency()` **returns `false`** for `0` or anything above 1 MHz, and then disables the fixed-rate functions: they return `0` until a valid frequency is set. Check the return value.
* `setFrequency()` and `setTunings()` can be called in either order.
* `dt_us == 0` makes `update()` / `updateVelocity()` return `0` without touching any state.
* Changing settings while running is allowed. Call `reset()` when you restart the loop after a pause, a mode change or when the actuator was disabled.

## 6. Limits and anti-windup

**Output limits** clamp what you get back. Set them to the range your actuator really accepts (for example `0 … 255` for 8-bit PWM).

**Integral limits** bound how much the I term alone may add to the output. The internal integral state is clamped together with it, so after the limit is reached, reversing the error starts to unwind the integral immediately instead of first "paying back" an invisible excess.

**Anti-windup.** In the position form, a new integral value is only kept if the unclamped output is not already beyond a limit in the direction of the error. In other words: while the output is saturated and the error would push it further, the integral stops growing.

Recommended setup:

```cpp
pid.setOutputLimits(0, 255);       // what the actuator can do
pid.setIntegralLimits(-100, 100);  // the I term may add at most +-100 to the output
```

**Deadband** (`setErrorDeadband`) removes small errors completely, which silences actuator chatter near the target. Note that the output jumps when the error leaves the band, because the deadband does not subtract its width.

**Integral threshold** (`setErrorIntegralThreshold`) turns the integral on only close to the target. It does *not* reset the integral outside the band; it just stops it from changing. It is useful for large step changes in the target.

## 7. Derivative filter

```
filteredD = (alpha * rawD + (1000 - alpha) * previousFilteredD) / 1000
```

| `alpha` | Effect |
| ---: | --- |
| `1000` | No filtering (default). |
| `500` | Each new value counts 50 %. |
| `100` | Strong smoothing. |
| `0` | The filtered value is frozen at its last value (the D term stops responding). |

**Lower `alpha` = more smoothing** (and more lag). Values above 1000 are treated as 1000. The filter only runs when `kd != 0`. In the velocity form it filters the change of the derivative term.

### Derivative on measurement

By default the derivative acts on the *error*, so a sudden change of `target` produces a one-step spike in the D term (derivative kick). If your target changes in steps, switch it to the measurement:

```cpp
pid.setDerivativeOnMeasurement(true);   // D = -Kd * d(input)/dt
```

A setpoint step then causes no kick, and with a constant `target` the output is identical to the default. Details:

* Works with all four update functions. In the velocity form only D moves to the measurement; P and I stay on the error.
* The first update after switching it on (or after `reset()`) only records the input, so the initial input value does not cause a kick.
* The error deadband does not apply to D in this mode, because D sees the raw measurement. Use the derivative filter against sensor noise.
* It can be switched at any time. You can also still keep `kd` small, use the filter, or ramp the target yourself.

## 8. Velocity form

`updateVelocity()` / `updateVelocityFixedRate()` compute the **change** of the output each step and add it to an internal accumulator. What they return is that accumulator, clamped by `setOutputLimits()`.

Things that differ from the position form:

* `setIntegralLimits()` is **not used**. The accumulator is limited only by `setOutputLimits()`, which also acts as the anti-windup.
* After `reset()` (and after construction) the accumulator starts at `0`. There is no way to preload it with the actuator's current value yet.
* P, I and D contributions are fractions of an output unit per step. The remainder is carried to the next call, so slow, small changes are not lost to rounding.
* The D term works on the second difference of the error, so a step in `target` produces a spike. `setDerivativeOnMeasurement(true)` moves D to the measurement and avoids it.

Use it for actuators that integrate your command (valves moved by increments, stepper-style positioners). For everything else the position form is simpler.

## 9. Input ranges and numeric behavior

### Guarantees

* Any `int32_t` target, input and gain, any `uint32_t` `dt_us`, any valid frequency: the library never overflows `int64_t` and has no undefined behavior. Intermediate products that would overflow **saturate** instead, so the output simply ends up at the corresponding output limit.
* The returned value is always inside `[outputMin, outputMax]`.
* Results are exact whenever the intermediate values fit in `int64_t`; extreme combinations (huge gains × huge errors × huge `dt`) may saturate earlier than the mathematically exact value would.
* The behavior is checked by unit tests, a 128-bit reference model and a sanitizer-instrumented fuzz test (see [Running the tests](#14-running-the-tests)).

### Rounding

* All divisions truncate toward zero.
* Position form: P and D are truncated each step; the integral keeps full resolution.
* Velocity form: the remainder of P, I and D is carried to the next step.
* In fixed-rate mode `1,000,000 / hz` is not an integer for most frequencies; the fraction is carried in the integral, so the time base stays exact.

### Details worth knowing

* The default integral threshold is `INT32_MAX`, so an error larger than that (possible only when `target` and `input` have opposite signs and huge magnitudes) does not integrate. Set the threshold explicitly if you work near the full `int32_t` range.
* Negative gains are accepted (reverse-acting loops). With extreme values, opposing huge terms can saturate before they cancel; if you need a reverse-acting loop, negating the error or the output is more predictable.
* Functions are not thread-safe. See [Using it from interrupts and tasks](#13-using-it-from-interrupts-and-tasks).

## 10. Recipes

### 10.1 Motor speed (variable loop time)

```cpp
#include "FixedPID.h"

FixedPID speedPid;
uint32_t lastUs;

void setup() {
  speedPid.setTunings(800, 3000, 0);      // Kp = 0.8, Ki = 3.0 /s
  speedPid.setOutputLimits(0, 255);       // PWM range
  speedPid.setIntegralLimits(0, 255);
  lastUs = micros();
}

void loop() {
  uint32_t now = micros();
  uint32_t dt = now - lastUs;
  lastUs = now;

  int32_t pwm = speedPid.update(targetRpm, readRpm(), dt);
  writePwm(pwm);
}
```

### 10.2 Temperature (slow loop, noisy sensor)

Work in hundredths of a degree so that integers have enough resolution.

```cpp
FixedPID heater;

void setup() {
  heater.setTunings(2000, 50, 0);                // Kp = 2.0, Ki = 0.05 /s
  heater.setOutputLimits(0, 255);
  heater.setIntegralLimits(0, 255);
  heater.setErrorDeadband(10);                   // ignore +-0.10 degC of sensor noise
  heater.setErrorIntegralThreshold(500);         // integrate only within 5.00 degC of the target
}

void loop() {                                    // runs once per second
  static uint32_t last = millis();
  uint32_t now = millis();
  int32_t out = heater.update(targetCentiDeg, readCentiDeg(), (now - last) * 1000UL);
  last = now;
  writeHeater(out);
  delay(1000);
}
```

### 10.3 Fixed-rate loop (timer driven, e.g. a flight controller)

```cpp
FixedPID rollRate;

void setup() {
  rollRate.setTunings(1200, 400, 30);
  rollRate.setOutputLimits(-1000, 1000);
  rollRate.setIntegralLimits(-300, 300);
  rollRate.setDerivativeFilter(250);             // smooth gyro noise in D
  if (!rollRate.setFrequency(1000)) {
    // invalid frequency: handle the error, the fixed-rate call would return 0
  }
}

// called from your 1 kHz timer task
void controlTick() {
  int32_t cmd = rollRate.updateFixedRate(rollSetpoint, gyroRoll);
  mixAndOutput(cmd);
}
```

### 10.4 Re-arming after a pause

```cpp
void onEnable() {
  pid.reset();          // forget the old integral and previous error
}
```

## 11. Tuning workflow

1. Set `ki = 0`, `kd = 0`, choose output limits that match the actuator.
2. Raise `kp` until the response is quick but starts to overshoot or oscillate, then back off to about half.
3. Add `ki` until the steady-state error disappears in an acceptable time. Set integral limits so that I alone cannot drive the actuator to its end stops.
4. Add `kd` only if you need it to damp overshoot. If the D term is noisy, lower the filter `alpha` (for example `500`, then `250`).
5. If the output chatters near the target, add a small deadband. If large target changes overshoot because of the integral, add an integral threshold.

Remember the stored gains are `real * 1000`: a gain change of `1` in the stored value is `0.001`.

## 12. Troubleshooting

| Symptom | Likely cause and fix |
| --- | --- |
| Output is always `0` | All gains are zero; `dt_us` is `0`; or you call a `*FixedRate` function without a valid `setFrequency()` (check its return value); or the error is inside the deadband. |
| Output sticks at a limit | The output limit is reached and the error keeps the same sign; check units and gain scale (remember the `* 1000`). |
| Slow creep or overshoot after a long saturation | Set `setIntegralLimits()` (position form) and make sure output limits match the actuator. |
| Jump when the loop is re-enabled | Call `reset()` when you restart the loop. |
| Output spikes when the target changes | Derivative kick. Call `setDerivativeOnMeasurement(true)`, or use the filter / a ramped target. |
| Noisy output | Lower the derivative filter `alpha`, add a deadband, or lower `kd`. |
| Result is much smaller than expected | The stored gains are `real * 1000`; `ki` is per second and `dt_us` is in microseconds. |
| Fixed-rate output differs slightly from `update()` | Only at rates that do not divide 1,000,000 evenly; the fixed-rate version is the more exact one. |

## 13. Using it from interrupts and tasks

* The library uses no heap, no floating point (in the update functions) and no blocking calls; each call takes a few microseconds on a 32-bit MCU (the exact time depends on the chip).
* An instance is not reentrant and not thread-safe. Call it from one context only, or protect it with a lock/critical section. Never share one instance between an ISR and a task without protection.
* `reset()` and the setters change the same state as the update functions; call them from the same context as `update`.

## 14. Running the tests

The repository contains tests that run on a PC and tests that run on an ESP32.

On a PC (needs `make` and GCC or Clang):

```
make test          # unit tests + fuzz tests, built with AddressSanitizer and UBSan
make fuzz SCALE=20 # a longer fuzz run
```

* `test/test.cpp` is the unit test suite. It is the same file you can run on an ESP32 (Arduino framework) and prints a pass/fail summary and a benchmark on the serial port.
* `test/host/fuzz_test.cpp` compares the first call of every update path with a 128-bit reference model, throws random extreme configurations at all four update functions, checks the derivative-on-measurement mode against the reference model, that variable-dt and fixed-rate agree, and that `reset()` restores a fresh state.

The same checks run in GitHub Actions on every push (`.github/workflows/ci.yml`).
