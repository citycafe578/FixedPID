#include <stdint.h>
#include "FixedPID.h"

namespace {

const int64_t INTEGRAL_DIV = TIME_SCALE * PID_SCALE;
const int     FRACTION_BITS = 20;
const int64_t FRACTION_ONE  = 1LL << FRACTION_BITS;

inline int64_t divCarry(int64_t num, int64_t den, int64_t& rem){
    num += rem;
    int64_t q = num / den;
    rem = num % den;
    return q;
}

inline int64_t divCarryFraction(int64_t num, int64_t den, int64_t& frac){
    int64_t q = num / den;
    int64_t r = num % den;
    frac += (r * FRACTION_ONE) / den;
    int64_t carry = frac / FRACTION_ONE;
    frac -= carry * FRACTION_ONE;
    return q + carry;
}

inline int64_t clamp64(int64_t v, int64_t lo, int64_t hi){
    if(v > hi){
        return hi;
    }
    if(v < lo){
        return lo;
    }
    return v;
}

}

FixedPID::FixedPID(){
    kp = 0;
    ki = 0;
    kd = 0;

    outputMin = INT32_MIN;
    outputMax = INT32_MAX;

    integralMin = INT32_MIN;
    integralMax = INT32_MAX;

    errorDeadband = 0;
    errIntegralThreshold = INT32_MAX;

    dt_us = 0;
    hz = 0;
    dtRemainder = 0;
    derivativeScale = 0;
    alpha = PID_SCALE;

    previousError = 0;
    integral = 0;
    previousPreviousError = 0;
    velocityOutput = 0;
    previousFilteredD = 0;
    integralFracRemainder = 0;
    velocityIntegralRemainder = 0;
    velocityPRemainder = 0;
    velocityDRemainder = 0;
    velocityDFraction = 0;
}

void FixedPID::setTunings(int32_t kp, int32_t ki, int32_t kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    if(hz != 0){
        derivativeScale = (int64_t)kd * (int64_t)hz;
    }
}

void FixedPID::setOutputLimits(int32_t min, int32_t max){
    if(min > max){
        int32_t t = min;
        min = max;
        max = t;
    }
    outputMin = min;
    outputMax = max;
}

void FixedPID::setIntegralLimits(int32_t min, int32_t max){
    if(min > max){
        int32_t t = min;
        min = max;
        max = t;
    }
    integralMin = min;
    integralMax = max;
}

bool FixedPID::setFrequency(uint32_t hz){
    integralFracRemainder = 0;

    if(hz == 0 || hz > 1000000UL){
        dt_us = 0;
        this->hz = 0;
        derivativeScale = 0;
        dtRemainder = 0;
        return false;
    }

    this->hz = hz;
    dt_us = 1000000UL / hz;
    dtRemainder = (int64_t)(1000000UL % hz);
    derivativeScale = (int64_t)kd * (int64_t)hz;
    return true;
}

void FixedPID::setErrorDeadband(uint32_t edb){
    errorDeadband = edb;
}

void FixedPID::setErrorIntegralThreshold(int32_t eit){
    errIntegralThreshold = eit;
}

void FixedPID::setDerivativeFilter(uint32_t alpha){
    this->alpha = alpha > PID_SCALE ? PID_SCALE : alpha;
}

int64_t FixedPID::filterDerivative(int64_t rawD){
    if(alpha == PID_SCALE){
        return rawD;
    }

    int64_t filteredD =
        ((int64_t)alpha * rawD +
         (PID_SCALE - (int64_t)alpha) * previousFilteredD) /
        PID_SCALE;

    previousFilteredD = filteredD;
    return filteredD;
}

int64_t FixedPID::applyDeadband(int64_t error) const{
    if(error <= (int64_t)errorDeadband && error >= -(int64_t)errorDeadband){
        return 0;
    }
    return error;
}

bool FixedPID::withinIntegralThreshold(int64_t error) const{
    return error <= (int64_t)errIntegralThreshold && error >= -(int64_t)errIntegralThreshold;
}

void FixedPID::reset(){
    integral = 0;
    previousError = 0;
    previousPreviousError = 0;
    velocityOutput = 0;
    previousFilteredD = 0;
    integralFracRemainder = 0;
    velocityIntegralRemainder = 0;
    velocityPRemainder = 0;
    velocityDRemainder = 0;
    velocityDFraction = 0;
}

int32_t FixedPID::finishPosition(int64_t error, int64_t newIntegral, int64_t newIntegralFrac, int64_t D){
    int64_t P = (int64_t)kp * error / PID_SCALE;
    int64_t I = 0;

    if(ki != 0){
        I = (int64_t)ki * newIntegral / INTEGRAL_DIV;

        if(I > (int64_t)integralMax){
            I = integralMax;
            newIntegral = (int64_t)integralMax * INTEGRAL_DIV / (int64_t)ki;
        }else if(I < (int64_t)integralMin){
            I = integralMin;
            newIntegral = (int64_t)integralMin * INTEGRAL_DIV / (int64_t)ki;
        }
    }

    int64_t candidateOutput = P + I + D;

    if(!((candidateOutput > (int64_t)outputMax && error > 0) ||
         (candidateOutput < (int64_t)outputMin && error < 0))){
        integral = newIntegral;
        integralFracRemainder = newIntegralFrac;
    }

    previousError = error;

    return (int32_t)clamp64(candidateOutput, outputMin, outputMax);
}

int32_t FixedPID::finishVelocity(int64_t error, int64_t deltaP, int64_t deltaI, int64_t deltaD){
    velocityOutput = clamp64(velocityOutput + deltaP + deltaI + deltaD, outputMin, outputMax);
    previousPreviousError = previousError;
    previousError = error;

    return (int32_t)velocityOutput;
}

int32_t FixedPID::update(int32_t target, int32_t input, uint32_t dt_us){
    if(dt_us == 0){
        return 0;
    }

    int64_t error = applyDeadband((int64_t)target - (int64_t)input);

    int64_t newIntegral = integral;
    if(ki != 0 && withinIntegralThreshold(error)){
        newIntegral += error * (int64_t)dt_us;
    }

    int64_t D = 0;
    if(kd != 0){
        D = (int64_t)kd * (error - previousError) * TIME_SCALE / ((int64_t)dt_us * PID_SCALE);
        D = filterDerivative(D);
    }

    return finishPosition(error, newIntegral, integralFracRemainder, D);
}

int32_t FixedPID::updateFixedRate(int32_t target, int32_t input){
    if(dt_us == 0){
        return 0;
    }

    int64_t error = applyDeadband((int64_t)target - (int64_t)input);

    int64_t newIntegral = integral;
    int64_t newFrac = integralFracRemainder;
    if(ki != 0 && withinIntegralThreshold(error)){
        newIntegral += error * (int64_t)dt_us;

        // 1e6/hz is not an integer for most frequencies; carry the fraction
        // instead of dropping it, so the integral time base stays exact.
        if(dtRemainder != 0){
            int64_t n = error * dtRemainder + newFrac;
            newIntegral += n / (int64_t)hz;
            newFrac = n % (int64_t)hz;
        }
    }

    int64_t D = 0;
    if(kd != 0){
        D = (error - previousError) * derivativeScale / PID_SCALE;
        D = filterDerivative(D);
    }

    return finishPosition(error, newIntegral, newFrac, D);
}

int32_t FixedPID::updateVelocity(int32_t target, int32_t input, uint32_t dt_us){
    if(dt_us == 0){
        return 0;
    }

    int64_t error = applyDeadband((int64_t)target - (int64_t)input);
    int64_t errorChange = error - previousError;
    int64_t secondErrorChange = error - 2 * previousError + previousPreviousError;

    int64_t deltaP = divCarry((int64_t)kp * errorChange, PID_SCALE, velocityPRemainder);

    int64_t deltaI = 0;
    if(withinIntegralThreshold(error)){
        deltaI = divCarry((int64_t)ki * error * (int64_t)dt_us, INTEGRAL_DIV, velocityIntegralRemainder);
    }

    int64_t deltaD = 0;
    if(kd != 0){
        deltaD = divCarryFraction((int64_t)kd * secondErrorChange * TIME_SCALE,
                                  (int64_t)dt_us * PID_SCALE,
                                  velocityDFraction);
        deltaD = filterDerivative(deltaD);
    }

    return finishVelocity(error, deltaP, deltaI, deltaD);
}

int32_t FixedPID::updateVelocityFixedRate(int32_t target, int32_t input){
    if(dt_us == 0){
        return 0;
    }

    int64_t error = applyDeadband((int64_t)target - (int64_t)input);
    int64_t errorChange = error - previousError;
    int64_t secondErrorChange = error - 2 * previousError + previousPreviousError;

    int64_t deltaP = divCarry((int64_t)kp * errorChange, PID_SCALE, velocityPRemainder);

    int64_t deltaI = 0;
    if(withinIntegralThreshold(error)){
        int64_t term = (int64_t)ki * error * (int64_t)dt_us;
        if(dtRemainder != 0){
            term += (int64_t)ki * error * dtRemainder / (int64_t)hz;
        }
        deltaI = divCarry(term, INTEGRAL_DIV, velocityIntegralRemainder);
    }

    int64_t deltaD = 0;
    if(kd != 0){
        deltaD = divCarry(secondErrorChange * derivativeScale, PID_SCALE, velocityDRemainder);
        deltaD = filterDerivative(deltaD);
    }

    return finishVelocity(error, deltaP, deltaI, deltaD);
}
