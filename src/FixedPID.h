#ifndef FIXED_PID_H
#define FIXED_PID_H

#include <stdint.h>

#define PID_SCALE 1000LL
#define TIME_SCALE 1000000LL

// Units
//   target, input, error  int32_t, any unit (error = target - input, 33-bit range)
//   kp, ki, kd            gain x PID_SCALE (1.5 -> 1500); ki per second, kd in seconds
//   dt_us                 microseconds
//   output                int32_t, limited by setOutputLimits()
//
// Range protection: every int32 input, every int32 gain and every uint32 dt is
// accepted. Intermediate arithmetic saturates instead of overflowing, so the
// result is always inside the output limits and there is no undefined behavior.
// See docs/USAGE.md for details.
class FixedPID{
    public:
        FixedPID();

        void setTunings(int32_t kp, int32_t ki, int32_t kd);
        bool setTuningsFloat(float kp, float ki, float kd);
        void setOutputLimits(int32_t min, int32_t max);
        void setIntegralLimits(int32_t min, int32_t max);

        // Returns false (and disables the fixed-rate paths) when hz is 0 or above 1 MHz.
        bool setFrequency(uint32_t hz);

        void setErrorDeadband(uint32_t edb);
        void setErrorIntegralThreshold(int32_t eit);
        void setDerivativeFilter(uint32_t alpha);
        void setDerivativeOnMeasurement(bool enable);
        void reset();

        int32_t update(int32_t target, int32_t input, uint32_t dt_us);
        int32_t updateFixedRate(int32_t target, int32_t input);
        int32_t updateVelocity(int32_t target, int32_t input, uint32_t dt_us);
        int32_t updateVelocityFixedRate(int32_t target, int32_t input);

    private:
        int32_t kp, ki, kd;

        int64_t previousError;
        bool derivativeOnMeasurement;
        bool hasInput;
        int64_t previousInput;
        int64_t previousPreviousInput;
        int64_t integral;

        int32_t outputMin, outputMax;
        int32_t integralMin, integralMax;

        uint32_t dt_us;

        uint32_t hz;
        int64_t derivativeScale;
        int64_t dtRemainder;       // 1000000 % hz, keeps the fixed-rate integral time base exact
        int64_t integralMulLimit;  // |integral| below this: ki * integral cannot overflow int64

        uint32_t errorDeadband;
        int32_t errIntegralThreshold;

        int64_t filterDerivative(int64_t rawD);
        int64_t applyDeadband(int64_t error) const;
        void measurementChange(int64_t input, int64_t& change, int64_t& secondChange);
        bool withinIntegralThreshold(int64_t error) const;
        int32_t finishPosition(int64_t error, int64_t newIntegral, int64_t newIntegralFrac, int64_t D);
        int32_t finishVelocity(int64_t error, int64_t deltaP, int64_t deltaI, int64_t deltaD);

        int64_t previousPreviousError;
        int64_t velocityOutput;

        uint32_t alpha;
        int64_t previousFilteredD;

        int64_t integralFracRemainder;     // position-form, fixed-rate non-divisible frequencies
        int64_t velocityIntegralRemainder;
        int64_t velocityPRemainder;
        int64_t velocityDRemainder;        // fixed-rate velocity D (exact, denominator PID_SCALE)
        int64_t velocityDFraction;         // variable-dt velocity D (Q20 fraction)
};

#endif
