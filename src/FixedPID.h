#ifndef FIXED_PID_H
#define FIXED_PID_H
#include <stdint.h>

#define PID_SCALE 1000

class FixedPID{
    public:
        FixedPID();
        void setTunings(int32_t kp, int32_t ki, int32_t kd);
        void setOutputLimits(int32_t min, int32_t max);
        void setIntegralLimits(int32_t min, int32_t max);
        void reset();
        int32_t update(int32_t target, int32_t input, uint32_t dt_us);
    
    private:
        int32_t kp, ki, kd;
        int32_t previousError;
        int64_t integral;
        int32_t outputMin, outputMax;
        int32_t integralMin, integralMax;
};

#endif