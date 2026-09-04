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

    int64_t error = (int64_t)target - (int64_t)input;
    int64_t errorChange = error - (int64_t)previousError;
    int64_t derivative = errorChange * 1000000LL/(int64_t)dt_us;
    int64_t P = (int64_t)kp * error;
    int64_t newIntegral = integral + error * (int64_t)dt_us;
    int64_t candidateI = 0;

    if(ki != 0){
        candidateI = (int64_t)ki * newIntegral /1000000LL;
    }

    if(candidateI > (int64_t)integralMax){
        candidateI = integralMax;
    }

    if(candidateI < (int64_t)integralMin){
        candidateI = integralMin;
    }

    int64_t D = (int64_t)kd * derivative;
    int64_t candidateOutput = P + candidateI + D;
    bool saturatedHigh = candidateOutput > (int64_t)outputMax;
    bool saturatedLow = candidateOutput < (int64_t)outputMin;


    // ANTI-WINDUP
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

    int64_t I = 0;

    if(ki != 0){
        I = (int64_t)ki * integral / 1000000LL;
    }

    if(I > (int64_t)integralMax){
        I = integralMax;
    }

    if(I < (int64_t)integralMin){
        I = integralMin;
    }


    // output
    if(!allowIntegral){
        I = candidateI;
    }

    int64_t output = P + I + D;
    if(output > (int64_t)outputMax){
        output = outputMax;
    }

    if(output < (int64_t)outputMin){
        output = outputMin;
    }

    previousError = (int32_t)error;

    return (int32_t)output;
}