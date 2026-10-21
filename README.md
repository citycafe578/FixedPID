# FixedPID

[English](README.md) | [繁體中文](README.zh-TW.md)

Documentation: [Usage guide](docs/USAGE.md) · [使用指南（繁體中文）](docs/USAGE.zh-TW.md)

A small fixed-point PID controller designed for embedded systems.

FixedPID uses integer arithmetic with `int64_t` intermediate calculations instead of floating-point math. It is designed for applications that need predictable integer-based computation, low overhead, and explicit control over sample timing.

## Features

* P / PI / PID control
* Fixed-point gain representation (`PID_SCALE = 1000`)
* `int32_t` input and output
* `int64_t` intermediate calculations
* Variable timestep (`update`)
* Fixed-rate path with precomputed time scaling (`updateFixedRate`)
* Velocity-form PID (`updateVelocity` / `updateVelocityFixedRate`)
* Output limiting
* Integral limiting
* Anti-windup
* Error deadband
* Integral threshold control
* Derivative low-pass filtering
* Optional derivative on measurement (no derivative kick on setpoint steps)
* `setTuningsFloat()` setup helper: gains as plain numbers (`1.5f`) instead of `x 1000`
* Controller state reset
* `dt = 0` protection
* Range protection: overflow-safe saturating arithmetic for any `int32_t` input, gain and timestep
* Designed for embedded systems

## Status

**Version: v1.1.0**

Adds optional derivative on measurement and a float tuning helper on top of v1.0.0. Fully backward compatible: existing code behaves exactly as before. See [Changelog](#changelog).

---

## Basic Usage

```cpp
#include "FixedPID.h"

FixedPID pid;

void setup() {
    pid.setTunings(
        1500,  // Kp = 1.5
        200,   // Ki = 0.2
        50     // Kd = 0.05
    );

    pid.setOutputLimits(-10000, 10000);
    pid.setIntegralLimits(-5000, 5000);
}

void loop() {
    int32_t target = 1000;
    int32_t input  = 950;
    uint32_t dt_us = 1000;  // 1 ms

    int32_t output = pid.update(target, input, dt_us);
}
```

### Fixed-rate mode

When the control loop runs at a known fixed frequency:

```cpp
pid.setFrequency(1000);  // 1000 Hz

int32_t output = pid.updateFixedRate(target, input);
```

The fixed-rate path precomputes time-dependent scaling and avoids repeating some calculations on every update.

---

## Fixed-Point Gains

```cpp
#define PID_SCALE 1000
```

FixedPID stores gains using a scale factor of 1000:

```text
Stored value = Gain × 1000
```

| Desired gain | Value passed to `setTunings()` |
| -----------: | -----------------------------: |
|         0.01 |                             10 |
|         0.05 |                             50 |
|          0.1 |                            100 |
|          0.5 |                            500 |
|          1.0 |                           1000 |
|          1.5 |                           1500 |
|          2.0 |                           2000 |

For example:

```cpp
pid.setTunings(
    1500,  // Kp = 1.5
    200,   // Ki = 0.2
    50     // Kd = 0.05
);
```

---

## API Overview

### Core

| Function                         | Description                                                               |
| -------------------------------- | ------------------------------------------------------------------------- |
| `setTunings(kp, ki, kd)`         | Set P/I/D gains using ×1000 fixed-point scaling                           |
| `setOutputLimits(min, max)`      | Limit the final controller output                                         |
| `setIntegralLimits(min, max)`    | Limit the calculated integral contribution                                |
| `reset()`                        | Reset the integral, previous-error, velocity, and derivative-filter state |
| `update(target, input, dt_us)`   | Perform a PID update using the supplied timestep                          |
| `updateFixedRate(target, input)` | Perform a PID update using the configured fixed frequency                 |

### New in v0.2

| Function                                 | Purpose                                                                                                                                                    |
| ---------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `setFrequency(hz)`                       | Configure the fixed loop frequency used by `updateFixedRate()` and `updateVelocityFixedRate()`. Enables precomputed time scaling for the fixed-rate path.  |
| `setErrorDeadband(edb)`                  | Treat errors within ±deadband as zero. Useful for reducing small control movements caused by sensor noise.                                                 |
| `setErrorIntegralThreshold(eit)`         | Limit integral accumulation to cases where the absolute value of `error` is within the configured threshold. Useful when integral action should only operate near the target. |
| `setDerivativeFilter(alpha)`             | Apply a first-order low-pass filter to the derivative term. Lower alpha values produce stronger smoothing.                                                 |
| `updateVelocity(target, input, dt_us)`   | Perform velocity-form PID using a variable timestep.                                                                                                       |
| `updateVelocityFixedRate(target, input)` | Perform velocity-form PID using the configured fixed frequency.                                                                                            |

### New in v1.1

| Function                                 | Purpose                                                                                                   |
| ---------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `setDerivativeOnMeasurement(enable)`     | Make the D term act on the measurement instead of the error, so a setpoint step causes no derivative kick. Default off. |
| `setTuningsFloat(kp, ki, kd)`            | Set the gains as plain numbers (`1.5f`) in `setup()`. Rounded to 0.001; returns `false` for NaN or out-of-range gains.  |

---

## API Details

### `setFrequency(uint32_t hz)`

Configures the controller's fixed update frequency.

Valid range is 1 to 1,000,000 Hz. Out-of-range values return `false` and disable the fixed-rate paths (`updateFixedRate()` and `updateVelocityFixedRate()` then always return 0), so check the return value. When the frequency does not divide 1,000,000 evenly (for example 3000 Hz), the integral time base is remainder-compensated and does not drift.

```cpp
pid.setFrequency(1000);  // 1 kHz control loop
```

This must be called before using:

```cpp
updateFixedRate()
updateVelocityFixedRate()
```

The fixed-rate path uses the configured timestep to precompute time-dependent scaling, reducing the amount of work required during each update.

---

### `setErrorDeadband(uint32_t edb)`

Treats errors within ±`edb` as zero.

```cpp
pid.setErrorDeadband(5);
```

For example, with:

```cpp
pid.setErrorDeadband(5);
```

errors from `-5` through `+5` are treated as zero.

This can be useful when sensor noise causes unnecessary small control movements around the target.

---

### `setErrorIntegralThreshold(int32_t eit)`

Controls when the integral term is allowed to accumulate.

```cpp
pid.setErrorIntegralThreshold(20);
```

With a threshold of `20`, the integral state is accumulated only while:

```text
|error| <= 20
```

This can be useful when integral action is intended mainly for correcting small steady-state errors rather than accumulating during large transients.

> Note: This behavior is an **integral threshold near the target**, rather than the more common "integral separation" scheme that disables integration during small errors.

---

### `setDerivativeFilter(uint32_t alpha)`

Applies a low-pass filter to the derivative term.

```cpp
pid.setDerivativeFilter(200);
```

`alpha` is scaled using `PID_SCALE = 1000`.

The effective range is:

```text
0 <= alpha <= 1000
```

The filter behaves approximately as:

```text
filteredD = alpha × rawD + (1 - alpha) × previousFilteredD
```

after applying the `PID_SCALE` scaling.

Therefore:

|  Alpha | Behavior                         |
| -----: | -------------------------------- |
|    `0` | Hold the previous filtered value |
|  `200` | Strong smoothing                 |
|  `500` | Moderate smoothing               |
|  `800` | Light smoothing                  |
| `1000` | No filtering                     |

**Lower alpha = more smoothing.**

The appropriate value depends on sensor noise, loop frequency, and the dynamics of the controlled system.

---

### `setDerivativeOnMeasurement(bool enable)`

By default the D term acts on the error, so a sudden change of `target` produces a one-step spike (derivative kick). With

```cpp
pid.setDerivativeOnMeasurement(true);
```

the D term acts on the measurement instead: `D = -Kd · d(input)/dt`. A setpoint step no longer kicks, and with a constant `target` the result is identical to the default.

* Default is `false`. Works with all four update functions. In velocity form only the D term moves to the measurement; P and I stay on the error.
* The first update after switching it on (or after `reset()`) only records the input, so the initial input value does not cause a kick.
* The error deadband does not apply to the D term in this mode (it sees the raw measurement). Use `setDerivativeFilter()` against sensor noise.
* It can be switched at any time.

---

### `setTuningsFloat(float kp, float ki, float kd)`

A convenience for `setup()`: write the gains as plain numbers.

```cpp
pid.setTuningsFloat(1.5f, 0.2f, 0.05f);   // same as setTunings(1500, 200, 50)
```

Each gain is multiplied by 1000 and rounded to the nearest integer, so values are resolved to 0.001. Floating point is used only inside this call, never in the update functions, so call it from `setup()` and not from the control loop.

Returns `false` if a gain is NaN (set to 0) or outside ±2,147,483.647 (saturated). The other gains are still applied.

---

## `update()` vs `updateFixedRate()`

| Mode              | Function                         | When to use                                           |
| ----------------- | -------------------------------- | ----------------------------------------------------- |
| Variable timestep | `update(target, input, dt_us)`   | Loops where the actual timestep may vary              |
| Fixed rate        | `updateFixedRate(target, input)` | Timer- or interrupt-driven loops with a stable period |

Use `update()` when the actual elapsed time is available and may vary from cycle to cycle.

Use `updateFixedRate()` when the controller is executed at a known, stable frequency and the configured frequency accurately represents the loop rate.

---

## Velocity Form

```cpp
int32_t out = pid.updateVelocity(target, input, dt_us);
```

or:

```cpp
pid.setFrequency(1000);

int32_t out = pid.updateVelocityFixedRate(target, input);
```

Velocity-form PID calculates changes to the controller output rather than directly calculating the complete positional output from the current error.

This form can be useful for incremental or rate-style actuators and for control systems where incremental output behavior is preferred.

Notes:

* By default the derivative term acts on the error, so a setpoint step produces a derivative kick. Use `setDerivativeOnMeasurement(true)` to avoid it (only D moves to the measurement; P stays on the error).
* The accumulated output is limited by `setOutputLimits()`. Velocity form does **not** use `setIntegralLimits()`.

---

## Output Limits

Output limits can be configured with:

```cpp
pid.setOutputLimits(-10000, 10000);
```

The final controller output is clamped to the configured range.

This prevents the returned output from exceeding the allowed actuator or control range.

---

## Integral Limits

Integral contribution can be limited with:

```cpp
pid.setIntegralLimits(-5000, 5000);
```

This limits the calculated integral contribution and provides an additional safeguard against excessive integral action.

Integral limits and output limits serve different purposes:

* **Output limits** constrain the final controller output.
* **Integral limits** constrain the integral contribution itself. The internal integral state is clamped as well, so it cannot wind up behind the limit and reversing the error takes effect immediately.

---

## Anti-Windup

FixedPID includes conditional integral-state updating to reduce integral windup.

When the calculated output is already saturated and the current error would push it further into saturation, the newly accumulated integral state is not saved.

The current cycle can still use the calculated integral contribution before the final output is clamped.

For example:

```cpp
pid.setOutputLimits(-10000, 10000);
pid.setIntegralLimits(-5000, 5000);
```

Using both limits provides control over both the integral contribution and the final output range.

---

## Reset

The controller state can be reset with:

```cpp
pid.reset();
```

Reset clears the internal controller state, including:

* Integral state
* Previous error
* Previous-previous error used by velocity form
* Velocity-form output state
* Filtered derivative state

This is useful when restarting a controller, changing operating conditions, or reinitializing a control loop.

---

## Testing

v1.0.0 includes an automated test suite covering functional behavior, edge cases, and fixed-rate operation.

```text
========================================
             TEST RESULT
========================================
Total : 173
PASS  : 173
FAIL  : 0
========================================
       ALL TESTS PASSED
========================================
```

Coverage includes:

* P / I / D behavior
* Variable timestep and `dt = 0`
* Reset behavior
* Output and integral limits
* Anti-windup
* Fixed-point scaling
* Integer boundary conditions
* Long-duration operation
* Timestep jitter
* Fixed-rate vs variable-dt behavior
* New API paths covered by the test suite
* Integral state clamping and unwind behavior
* Velocity-form P / I / D remainder carry
* Fixed-rate time base for frequencies that do not divide 1,000,000
* `setFrequency()` return value and swapped limit arguments
* Range protection: extreme gains, errors and timesteps on every update path
* Derivative on measurement on every update path (no kick, same result at a constant setpoint, reset, mode switching)
* `setTuningsFloat()` rounding, NaN and out-of-range gains

### Running the tests on a PC

`make test` builds the unit tests and a randomized fuzz test with AddressSanitizer and UBSan and runs them (needs `make` and GCC or Clang). The fuzz test compares every update path with a 128-bit reference model, throws random extreme configurations at all four update functions, and checks that variable-dt and fixed-rate agree and that `reset()` restores a fresh state. The same checks run in GitHub Actions on every push. See the [Usage guide](docs/USAGE.md#14-running-the-tests).

---

## Benchmark

Lower is better. Measurements represent the approximate execution time of one controller update.

### FixedPID v1.1.0 on ESP32

Platform: **ESP32 (ESP32-D0WD-V3, dual core @ 240 MHz)**.

| Mode                               |       P only |           PI |     Full PID |
| ---------------------------------- | -----------: | -----------: | -----------: |
| **Variable-dt** (`update`)         | **1.132 µs** | **1.791 µs** | **2.265 µs** |
| **Fixed-rate** (`updateFixedRate`) | **1.150 µs** | **1.719 µs** | **2.116 µs** |

Compared with v0.2.2 on the same ESP32. v1.1.0 includes range protection and is still faster, because 64-bit divisions by 1000 and 1e9 now use 32-bit hardware divides:

| Mode (Full PID)  |  v0.2.2 | v1.1.0 |
| ---------------- | ------: | -----: |
| Variable-dt      | 2.424 µs | **2.265 µs** |
| Fixed-rate       | 2.462 µs | **2.116 µs** |

The ESP32 has no hardware 64-bit divide, so fixed-rate saves little there; on the ESP32-C3 below the difference is much larger.

### ESP32-C3 results (v0.2.1)

Platform: **Nologo ESP32-C3 SuperMini @ 160 MHz**

| Mode                               |       P only |           PI |     Full PID |
| ---------------------------------- | -----------: | -----------: | -----------: |
| **Variable-dt** (`update`)         | **2.098 µs** | **3.110 µs** | **3.126 µs** |
| **Fixed-rate** (`updateFixedRate`) | **0.589 µs** | **1.897 µs** | **2.149 µs** |

### Compared with previous FixedPID measurement

Earlier measurements from the same class of platform and test setup:

| Mode                 |     Full PID |
| -------------------- | -----------: |
| Previous `update()`  |     ~4.83 µs |
| **v0.2.1 Variable-dt** | **~3.13 µs** |
| **v0.2.1 Fixed-rate**  | **~2.15 µs** |

Approximate improvement compared with the previous ~4.83 µs result:

| Path        | Approximate speedup |
| ----------- | ------------------: |
| Variable-dt |    **~1.5× faster** |
| Fixed-rate  |    **~2.2× faster** |

### Compared with other libraries

Reference measurements from earlier ESP32-C3 comparisons:

| Library                       | Approx. Full PID time |
| ----------------------------- | --------------------: |
| QuickPID                      |             ~13.62 µs |
| AutoPID (r-downing)           |             ~18.74 µs |
| **FixedPID v0.2.1 Variable-dt** |          **~3.13 µs** |
| **FixedPID v0.2.1 Fixed-rate**  |          **~2.15 µs** |

Approximate relative speed:

| Comparison                       | Approximate factor |
| -------------------------------- | -----------------: |
| FixedPID variable-dt vs QuickPID |   **~4.4× faster** |
| FixedPID variable-dt vs AutoPID  |   **~6.0× faster** |
| FixedPID fixed-rate vs QuickPID  |   **~6.3× faster** |
| FixedPID fixed-rate vs AutoPID   |   **~8.7× faster** |

```text
Full PID time (µs, lower is better)

AutoPID      |################## 18.74
QuickPID     |#############      13.62
FixedPID var |###                 3.13
FixedPID fix |##                  2.15
```

These measurements are reference results, not universal guarantees. Actual performance depends on the MCU, CPU frequency, compiler, optimization settings, and configuration.

FixedPID uses `int64_t` intermediate calculations, which can have significantly different performance characteristics across MCU architectures. The ESP32-C3 benchmark is therefore intended as a reference point rather than a representative result for 8-bit AVR platforms.

FastPID and other integer-based libraries remain relevant on 8-bit AVR platforms. FixedPID focuses on predictable fixed-point behavior and explicit timestep handling across embedded targets, with a particularly strong performance profile on modern 32-bit MCUs.

---

## Design Goals

1. Integer-only PID arithmetic
2. Predictable numerical behavior
3. Explicit timestep handling for both variable and fixed-rate operation
4. Practical output limiting, integral limiting, and anti-windup
5. Low runtime overhead on modern MCUs
6. Small and readable API
7. Portable implementation without floating-point PID calculations

---

## Roadmap

Planned for a later major release:

* Setpoint weighting / ramping
* Output slew-rate limiting
* Bumpless transfer helpers
* Additional MCU benchmarks (UNO, STM32, etc.)
* More documentation and examples

---

## Changelog

### v1.1.0

New (backward compatible, existing behavior is unchanged):

* **`setDerivativeOnMeasurement(bool)`** – optional derivative on measurement for all four update functions, so a setpoint step no longer produces a derivative kick. Default off.
* **`setTuningsFloat(kp, ki, kd)`** – set the gains as plain numbers in `setup()`; returns `false` for NaN or out-of-range gains.
* **Tests** – 140 to 173 tests. The fuzz test gained an exact derivative-on-measurement check against the 128-bit reference model and now also runs its robustness, variable-vs-fixed-rate and reset checks with the mode switched on at random.
* Usage guide updated.

### v1.0.0

First stable release. The public API is unchanged from v0.2.2.

New:

* **Range protection** – every `int32_t` input and gain and every `uint32_t` timestep is now accepted without `int64_t` overflow or undefined behavior. Intermediate products saturate instead, and the output always stays inside the output limits. Previously, extreme gains, errors or timesteps could overflow silently.
* **Velocity-form remainder** – the variable-dt derivative remainder now uses the same accumulation rule as the fixed-rate path, so both give identical results.
* **Faster on the ESP32** – 64-bit divisions by 1000 and 1e9 now use 32-bit hardware divides, and multiply overflow checks are skipped when the operands are provably small. Measured on an ESP32 (variable-dt / fixed-rate): P 1.03 / 1.03 µs, PI 1.68 / 1.62 µs, full PID 2.18 / 2.06 µs per update.
* **Tests** – 125 to 140 tests. Added a PC build of the test suite and a sanitizer-instrumented fuzz test against a 128-bit reference model (`make test`), plus a GitHub Actions workflow.
* **Documentation and packaging** – added the [Usage guide](docs/USAGE.md) (English and Traditional Chinese), `library.properties`, and one folder per example (`examples/<name>/<name>.ino`), which the Arduino IDE requires.

### v0.2.2

Fixes:

* **Integral limit** – the internal integral state is now clamped together with the integral term, so it can no longer wind up behind `setIntegralLimits()`. Reversing the error now unwinds immediately. This also removes a possible `int64_t` overflow of `ki × integral` on long runs with the default limits.
* **Velocity-form P and D** – the sub-LSB remainder of the P and D terms is now carried between calls (as the I term already was). Previously a small `Kp` with a slowly changing error could produce no output at all.
* **Fixed-rate time base** – the integral of `updateFixedRate()` / `updateVelocityFixedRate()` no longer drops the fraction of `1,000,000 / hz`. At 300 kHz the integral was about 10 % low.
* **`setFrequency()`** – now returns `bool` (`false` for 0 Hz or above 1 MHz) instead of failing silently.
* `setOutputLimits()` / `setIntegralLimits()` swap their arguments if `min > max`.
* README: the Traditional Chinese `setDerivativeFilter` description was inverted (lower alpha = stronger smoothing); the claim that velocity form reduces derivative kick was removed.

Changes:

* Duplicated update code was merged into shared helpers. The P-only fast path was removed.
* `FixedPID.cpp` no longer includes `Arduino.h`.
* Test suite grew from 104 to 125 tests.

### v0.2.1

Fixes for issues found while reviewing v0.2:

* `PID_SCALE` was applied incorrectly in the P / I / D gain calculations.
* `setFrequency()` could divide by zero; it now rejects 0 Hz and frequencies above 1 MHz.
* The velocity-form integral term lost precision; its remainder is now carried between calls.
* The fixed-rate derivative lost precision; it is now computed from `kd × hz`.
* The previous error was stored in too narrow an integer type.
* The documentation of `setErrorIntegralThreshold()` was inconsistent.
* Removed the unused `<math.h>` include.
* Test suite grew from 73 to 104 tests.

### v0.2

* Added `setFrequency` and `updateFixedRate` (fixed-rate path with precomputed time scaling).
* Added velocity-form PID: `updateVelocity` and `updateVelocityFixedRate`.
* Added tuning helpers: `setErrorDeadband`, `setErrorIntegralThreshold` and `setDerivativeFilter`.
* Leaner update path with fewer intermediate variables.
* Added the Traditional Chinese README and two more examples.
* Test suite expanded to 73 tests.

### v0.1

* First public release: P / PI / PID with fixed-point gains, `int32_t` input and output, `int64_t` intermediates, variable timestep, output and integral limits, anti-windup, `reset()` and `dt = 0` protection.

---

## Current Status

The library files are included in this release (`src/`), and the 173-test suite passes on an ESP32.

FixedPID is also being validated in real-world use on my ESP32-based drone project. As with any control library, verify the behavior on your own hardware before relying on it.

FixedPID targets modern 32-bit MCUs. It has been tested on the ESP32; 8-bit AVR boards such as the Arduino UNO are not a target platform and have not been tested.

FixedPID is also listed in the [XERJ Corpus Hub](https://hub.xerj.org/contributors/citycafe578) (xerj.org), an open index of code solutions for AI coding agents.

---

## License

See the repository license file for licensing information.
