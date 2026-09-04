# FixedPID

A small fixed-point PID controller designed for embedded systems.

FixedPID uses integer arithmetic with `int64_t` intermediate calculations instead of floating-point PID calculations. It is designed for applications where predictable integer-based computation, low overhead, and explicit numerical behavior are useful.

## Features

* P / PI / PID control
* Fixed-point gain representation
* `int32_t` input and output
* `int64_t` intermediate calculations
* Variable timestep support
* Output limiting
* Integral limiting
* Anti-windup
* Controller state reset
* `dt = 0` protection
* Designed for embedded systems

## Status

**Version: v0.1**

This is the first public release.

The current version focuses on a small and predictable PID core rather than providing every advanced PID feature.

---

## Basic Usage

```cpp
#include "FixedPID.h"

FixedPID pid;

void setup()
{
    pid.setTunings(
        1500,   // Kp = 1.5
        200,    // Ki = 0.2
        50      // Kd = 0.05
    );

    pid.setOutputLimits(
        -10000,
        10000
    );

    pid.setIntegralLimits(
        -5000,
        5000
);
}

void loop()
{
    int32_t target = 1000;
    int32_t input = 950;

    uint32_t dt_us = 1000;

    int32_t output = pid.update(
        target,
        input,
        dt_us
    );
}
```

---

## Fixed-Point Gains

FixedPID uses a scale factor of:

```cpp
#define PID_SCALE 1000
```

Therefore:

```text
Stored value = Gain × 1000
```

For example:

| Desired gain | Value passed to `setTunings()` |
| -----------: | -----------------------------: |
|         0.01 |                             10 |
|         0.05 |                             50 |
|          0.1 |                            100 |
|          0.5 |                            500 |
|          1.0 |                           1000 |
|          1.5 |                           1500 |
|          2.0 |                           2000 |

This allows fractional gains without using floating-point values in the PID API.

---

## API

### `FixedPID()`

Creates a PID controller with all gains set to zero.

```cpp
FixedPID pid;
```

### `setTunings()`

Sets the proportional, integral, and derivative gains.

```cpp
pid.setTunings(kp, ki, kd);
```

The gains use the fixed-point scale of 1000.

For example:

```cpp
pid.setTunings(
    1000,
    200,
    50
);
```

represents:

```text
Kp = 1.0
Ki = 0.2
Kd = 0.05
```

### `setOutputLimits()`

Limits the final PID output.

```cpp
pid.setOutputLimits(
    -1000,
    1000
);
```

### `setIntegralLimits()`

Limits the integral contribution.

```cpp
pid.setIntegralLimits(
    -500,
    500
);
```

This can be used together with the built-in anti-windup behavior.

### `reset()`

Clears the PID state.

```cpp
pid.reset();
```

This resets:

* Integral state
* Previous error

### `update()`

Calculates one PID update.

```cpp
int32_t output = pid.update(
    target,
    input,
    dt_us
);
```

Arguments:

```text
target : desired value
input  : current measured value
dt_us  : elapsed time in microseconds
```

Example:

```cpp
int32_t output = pid.update(
    1000,
    950,
    1000
);
```

---

## PID Calculation

The controller conceptually follows:

```text
error = target - input

P = Kp × error

I = Ki × ∫error dt

D = Kd × d(error)/dt
```

The implementation uses `int64_t` intermediate calculations to reduce the risk of overflow during multiplication and accumulation.

The timestep is supplied in microseconds:

```text
dt_us = elapsed time in microseconds
```

For example:

```text
1000 us = 1 ms
2000 us = 2 ms
```

---

## Anti-Windup

FixedPID includes basic integral anti-windup.

When the calculated output is saturated and the current error would push the controller further into saturation, the new integral state is not stored.

This prevents the integral term from continuously accumulating while the output is already saturated.

The current cycle can still use the candidate integral contribution before the state is rejected.

---

## Numerical Safety

The implementation intentionally uses wider intermediate types:

```cpp
int64_t
```

for operations such as:

```text
error calculation
P calculation
integral accumulation
I calculation
derivative calculation
final output calculation
```

The library also handles:

* `INT32_MAX`
* `INT32_MIN`
* large positive/negative errors
* large timestep values
* `dt_us == 0`
* output saturation
* integral saturation

---

## Testing

The controller has been tested with a dedicated test suite covering:

* P response
* I accumulation
* D response
* Variable timestep
* Reset behavior
* Output limits
* Integral limits
* Anti-windup
* Fixed-point scaling
* Integer boundary conditions
* Extreme values
* Long-running updates
* Timestep jitter
* Fixed-rate update equivalence

Current test result:

```text
========================================
             TEST RESULT
========================================

Total : 42
PASS  : 42
FAIL  : 0

========================================
       ALL TESTS PASSED
========================================
```

---

## Benchmark

Benchmark results on an ESP32-C3 SuperMini at 160 MHz:

```text
Original update()

P only   : 2.630 us / update
PI       : 4.830 us / update
Full PID : 4.830 us / update
```

An optimized fixed-rate path was also tested:

```text
Optimized updateFixedRate()

P only   : 1.315 us / update
PI       : 2.327 us / update
Full PID : 2.327 us / update
```

The optimized implementation produced identical results for the tested output set:

```text
PASS : 1000/1000 outputs identical
```

These numbers are provided as reference measurements, not as a universal performance guarantee.

---

## Design Goals

FixedPID is intentionally small.

The main goals of v0.1 are:

1. Integer-based PID calculation
2. Predictable numerical behavior
3. Simple API
4. Embedded-system compatibility
5. Explicit control over timestep
6. Basic protection against integral windup
7. Low computational overhead

Advanced PID features are intentionally outside the scope of v0.1.

---

## Roadmap

Possible future versions may add:

* Controller direction
* Deadband
* Integral separation
* Derivative-on-measurement
* Derivative kick reduction
* Feed-forward
* Setpoint ramping
* Output slew-rate limiting
* Bumpless transfer
* Additional optimization
* More MCU-specific benchmarks

---

## License

See the repository license file for licensing information.
