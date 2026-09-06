# FixedPID

[English](README.md) | [繁體中文](README.zh-TW.md)

A small fixed-point PID controller designed for embedded systems.

FixedPID uses integer arithmetic with `int64_t` intermediate calculations instead of floating-point math. It is designed for applications that need predictable integer-based computation, low overhead, and explicit control over sample timing.

## Features

* P / PI / PID control
* Fixed-point gain representation (`PID_SCALE = 1000`)
* `int32_t` input and output
* `int64_t` intermediate calculations
* Variable timestep (`update`)
* Fixed-rate fast path (`updateFixedRate`)
* Velocity-form PID (`updateVelocity` / `updateVelocityFixedRate`)
* Output limiting
* Integral limiting
* Anti-windup
* Error deadband
* Integral threshold control
* Derivative low-pass filtering
* Controller state reset
* `dt = 0` protection
* Designed for embedded systems

## Status

**Version: v0.2**

This release improves performance, expands the API, and increases test coverage.

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

---

## API Details

### `setFrequency(uint32_t hz)`

Configures the controller's fixed update frequency.

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
* **Integral limits** constrain the integral contribution itself.

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

v0.2 includes an automated test suite covering functional behavior, edge cases, and fixed-rate operation.

```text
========================================
             TEST RESULT
========================================
Total : 73
PASS  : 73
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

---

## Benchmark

Platform: **Nologo ESP32-C3 SuperMini @ 160 MHz**

Lower is better. Measurements represent the approximate execution time of one controller update.

### FixedPID v0.2

| Mode                               |       P only |           PI |     Full PID |
| ---------------------------------- | -----------: | -----------: | -----------: |
| **Variable-dt** (`update`)         | **2.098 µs** | **3.110 µs** | **3.126 µs** |
| **Fixed-rate** (`updateFixedRate`) | **0.589 µs** | **1.897 µs** | **2.149 µs** |

### Compared with previous FixedPID measurement

Earlier measurements from the same class of platform and test setup:

| Mode                 |     Full PID |
| -------------------- | -----------: |
| Previous `update()`  |     ~4.83 µs |
| **v0.2 Variable-dt** | **~3.13 µs** |
| **v0.2 Fixed-rate**  | **~2.15 µs** |

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
| **FixedPID v0.2 Variable-dt** |          **~3.13 µs** |
| **FixedPID v0.2 Fixed-rate**  |          **~2.15 µs** |

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

Possible future work:

* More derivative-on-measurement options
* Setpoint weighting / ramping
* Output slew-rate limiting
* Bumpless transfer helpers
* Additional MCU benchmarks (UNO, STM32, etc.)
* More documentation and examples

---

## Acknowledgments / Feedback from v0.1

Thank you to everyone who tried v0.1 and shared feedback after the first release.

**Adopted from community  (Arduino.Taipei)  suggestions:**

* **Velocity-form PID** — added `updateVelocity()` and `updateVelocityFixedRate()` to reduce integral blow-up issues and make gain changes smoother in some use cases.

* **Fewer intermediate variables / leaner update path** — reduced unnecessary state and tightened the hot path after performance feedback (especially relevant on ESP32-C3 / RISC-V).

* **Fixed-rate fast path** — `setFrequency()` + `updateFixedRate()` for constant-period loops with lower per-step cost.

* **Practical tuning helpers** — error deadband, integral separation threshold, and derivative low-pass filter for noisy sensors and real control loops.

* **Broader tests** — suite expanded to 73 cases (73/73 pass), including fixed-rate equivalence and longer jitter runs.

**Considered but not taken in this release (and why):**

* **Power-of-two (`2^N`) gain/time scales + shift-only math** — attractive for speed, but would break the current `PID_SCALE = 1000` / `TIME_SCALE = 1000000` API and all existing tuning/docs/tests. Treated as a possible future breaking change, not a quiet v0.2 tweak.

* **Ultra-aggressive, architecture-specific micro-opts (sub-1 µs on C3)** — a prototype path reached under ~1 µs on ESP32-C3, but was not clearly a win on other cores/toolchains. Prefer portable behavior over peak numbers on one MCU.

* **Pushing latency toward ~0.5 µs** — diminishing returns for typical flight-control rates; next priority is real airframe testing and stability, not further micro-optimization.

**Next focus**

Integrate FixedPID into the author’s open drone stack for flight testing. If behavior is solid in the air, publish as an Arduino library for wider use. Further features (e.g. stronger derivative-on-measurement defaults, feed-forward, notch-oriented helpers) will follow validation, not the other way around.

---

## License

See the repository license file for licensing information.
