#include <Arduino.h>
#include "FixedPID.h"

FixedPID::FixedPID(){
    kp = 0;
    ki = 0;
    kd = 0;

    previousError = 0;
    integral = 0;

    outputMin = INT32_MIN;
    outputMax = INT32_MAX;

    integralMin = INT32_MIN;
    integralMax = INT32_MAX;
}

void FixedPID::setTunings(int32_t kp, int32_t ki, int32_t kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void FixedPID::setOutputLimits(int32_t min, int32_t max){
    outputMin = min;
    outputMax = max;
}

void FixedPID::setIntegralLimits(int32_t min, int32_t max){
    integralMin = min;
    integralMax = max;
}

void FixedPID::reset(){
    integral = 0;
    previousError = 0;
}

int32_t FixedPID::update(int32_t target, int32_t input, uint32_t dt_us){
    if(dt_us == 0){
        return 0;
    }

    // ========================================================
    // ERROR
    // ========================================================

    int64_t error = (int64_t)target - (int64_t)input;

    // ========================================================
    // DERIVATIVE
    // ========================================================

    int64_t errorChange = error - (int64_t)previousError;

    int64_t derivative = errorChange * 1000000LL/(int64_t)dt_us;

    // ========================================================
    // P
    // ========================================================

    int64_t P = (int64_t)kp * error;

    // ========================================================
    // INTEGRAL CANDIDATE
    // ========================================================

    int64_t newIntegral = integral + error * (int64_t)dt_us;

    // ========================================================
    // CALCULATE CANDIDATE I
    // ========================================================

    int64_t candidateI = 0;

    if(ki != 0){
        candidateI = (int64_t)ki * newIntegral /1000000LL;
    }

    // ========================================================
    // I LIMIT
    // ========================================================

    if(candidateI > (int64_t)integralMax){
        candidateI = integralMax;
    }

    if(candidateI < (int64_t)integralMin){
        candidateI = integralMin;
    }

    // ========================================================
    // D
    // ========================================================

    int64_t D = (int64_t)kd * derivative;

    // ========================================================
    // CANDIDATE OUTPUT
    //
    // Important:
    // Even if anti-windup rejects the integral state,
    // the CURRENT output still uses candidateI.
    // ========================================================

    int64_t candidateOutput = P + candidateI + D;

    // ========================================================
    // CHECK SATURATION
    // ========================================================

    bool saturatedHigh = candidateOutput > (int64_t)outputMax;

    bool saturatedLow = candidateOutput < (int64_t)outputMin;

    // ========================================================
    // ANTI-WINDUP
    //
    // If output is saturated and error pushes it further
    // into saturation, don't save the new integral.
    //
    // But candidateI is still used for THIS output.
    // ========================================================

    bool allowIntegral = true;

    if(saturatedHigh && error > 0){
        allowIntegral = false;
    }

    if(saturatedLow && error < 0){
        allowIntegral = false;
    }

    if(allowIntegral){
        integral = newIntegral;
    }

    // ========================================================
    // RECALCULATE I FROM SAVED INTEGRAL
    // ========================================================

    int64_t I = 0;

    if(ki != 0){
        I = (int64_t)ki * integral / 1000000LL;
    }

    // ========================================================
    // I LIMIT
    // ========================================================

    if(I > (int64_t)integralMax){
        I = integralMax;
    }

    if(I < (int64_t)integralMin){
        I = integralMin;
    }

    // ========================================================
    // FINAL OUTPUT
    //
    // If anti-windup rejected the integral update,
    // use candidateI for this cycle.
    // ========================================================

    if(!allowIntegral){
        I = candidateI;
    }

    int64_t output = P + I + D;

    // ========================================================
    // OUTPUT LIMIT
    // ========================================================

    if(output > (int64_t)outputMax){
        output = outputMax;
    }

    if(output < (int64_t)outputMin){
        output = outputMin;
    }

    // ========================================================
    // SAVE ERROR
    // ========================================================

    previousError = (int32_t)error;

    return (int32_t)output;
}