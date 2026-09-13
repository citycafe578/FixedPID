#include <Arduino.h>
#include "FixedPID.h"


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

    derivativeScale = 0;
    previousPreviousError = 0;
    velocityOutput = 0;
    alpha = PID_SCALE;
    previousFilteredD = 0;
    previousError = 0;
    integral = 0;
    velocityIntegralRemainder = 0;
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
    outputMin = min;
    outputMax = max;
}

void FixedPID::setIntegralLimits(int32_t min, int32_t max){
    integralMin = min;
    integralMax = max;
}

void FixedPID::setFrequency(uint32_t hz){
    if(hz == 0 || hz > 1000000UL){
        dt_us = 0;
        this->hz = 0;
        derivativeScale = 0;
        return;
    }

    this->hz = hz;
    dt_us = 1000000UL / hz;
    derivativeScale = (int64_t)kd * (int64_t)hz;
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

int32_t FixedPID::updateVelocity(int32_t target, int32_t input, uint32_t dt_us){
    if(dt_us == 0){
        return 0;
    }

    int64_t error = (int64_t)target - (int64_t)input;

    if(error <= (int64_t)errorDeadband && error >= -(int64_t)errorDeadband){
        error = 0;
    }

    int64_t errorChange = error - (int64_t)previousError;
    int64_t secondErrorChange = error - 2 * (int64_t)previousError + (int64_t)previousPreviousError;
    int64_t deltaP = (int64_t)kp * errorChange / PID_SCALE;
    int64_t deltaI = 0;

    if(error <= (int64_t)errIntegralThreshold && error >= -(int64_t)errIntegralThreshold){
        int64_t integralTerm = (int64_t)ki * error * (int64_t)dt_us;
        integralTerm += velocityIntegralRemainder;
        deltaI = integralTerm / (TIME_SCALE * PID_SCALE);
        velocityIntegralRemainder = integralTerm % (TIME_SCALE * PID_SCALE);
    }

    int64_t deltaD = (int64_t)kd * secondErrorChange * TIME_SCALE / ((int64_t)dt_us * PID_SCALE);
    deltaD = filterDerivative(deltaD);
    int64_t candidateOutput = velocityOutput + deltaP + deltaI + deltaD;

    if(candidateOutput > (int64_t)outputMax){
        candidateOutput = outputMax;
    }

    if(candidateOutput < (int64_t)outputMin){
        candidateOutput = outputMin;
    }

    velocityOutput = candidateOutput;
    previousPreviousError = previousError;
    previousError = error;

    return (int32_t)velocityOutput;
}


int32_t FixedPID::updateVelocityFixedRate(int32_t target, int32_t input){
    if(dt_us == 0){
        return 0;
    }

    int64_t error = (int64_t)target - (int64_t)input;

    if(error <= (int64_t)errorDeadband && error >= -(int64_t)errorDeadband){
        error = 0;
    }

    int64_t errorChange = error - (int64_t)previousError;
    int64_t secondErrorChange = error - 2 * (int64_t)previousError + (int64_t)previousPreviousError;
    int64_t deltaP = (int64_t)kp * errorChange / PID_SCALE;
    int64_t deltaI = 0;

    if(error <= (int64_t)errIntegralThreshold && error >= -(int64_t)errIntegralThreshold){
        int64_t integralTerm = (int64_t)ki * error * (int64_t)dt_us;
        integralTerm += velocityIntegralRemainder;
        deltaI = integralTerm / (TIME_SCALE * PID_SCALE);
        velocityIntegralRemainder = integralTerm % (TIME_SCALE * PID_SCALE);
    }

    int64_t deltaD = secondErrorChange * derivativeScale / PID_SCALE;
    deltaD = filterDerivative(deltaD);
    int64_t candidateOutput = velocityOutput + deltaP + deltaI + deltaD;

    if(candidateOutput > (int64_t)outputMax){
        candidateOutput = outputMax;
    }

    if(candidateOutput < (int64_t)outputMin){
        candidateOutput = outputMin;
    }

    velocityOutput = candidateOutput;
    previousPreviousError = previousError;
    previousError = error;

    return (int32_t)velocityOutput;
}

void FixedPID::reset(){
    integral = 0;
    previousError = 0;
    previousPreviousError = 0;
    velocityOutput = 0;
    previousFilteredD = 0;
    velocityIntegralRemainder = 0;
}

int32_t FixedPID::update(int32_t target, int32_t input, uint32_t dt_us){
    if(dt_us == 0){
        return 0;
    }

    int64_t error = (int64_t)target - (int64_t)input;
    if(error <= (int64_t)errorDeadband && error >= -(int64_t)errorDeadband){
        error = 0;
    }

    int64_t errorChange = error - (int64_t)previousError;

    // P
    int64_t candidateOutput = (int64_t)kp * error / PID_SCALE;

    // I
    int64_t newIntegral = integral;

    if(error <= (int64_t)errIntegralThreshold && error >= -(int64_t)errIntegralThreshold){
        newIntegral += error * (int64_t)dt_us;
    }

    int64_t candidateI = 0;

    if(ki != 0){
        candidateI = (int64_t)ki * newIntegral / (TIME_SCALE * PID_SCALE);
    }

    if(candidateI > (int64_t)integralMax){
        candidateI = integralMax;
    }

    if(candidateI < (int64_t)integralMin){
        candidateI = integralMin;
    }

    // D
    int64_t D = (int64_t)kd * errorChange * TIME_SCALE / ((int64_t)dt_us * PID_SCALE);

    D = filterDerivative(D);
    candidateOutput += candidateI + D;

    if(!((candidateOutput > (int64_t)outputMax && error > 0) || (candidateOutput < (int64_t)outputMin && error < 0))){
        integral = newIntegral;
    }

    int64_t output = candidateOutput;

    if(output > (int64_t)outputMax){
        output = outputMax;
    }

    if(output < (int64_t)outputMin){
        output = outputMin;
    }

    previousError = error;

    return (int32_t)output;
}

int32_t FixedPID::updateFixedRate(int32_t target, int32_t input){
    if(dt_us == 0){
        return 0;
    }

    if(ki == 0 && kd == 0 && errorDeadband == 0){
        int64_t output = (int64_t)kp * ((int64_t)target - (int64_t)input) / PID_SCALE;

        if(output > (int64_t)outputMax){
            output = outputMax;
        }

        if(output < (int64_t)outputMin){
            output = outputMin;
        }

        previousError = (int64_t)target - (int64_t)input;
        return (int32_t)output;
    }

    if(kd == 0){
        int64_t error = (int64_t)target - (int64_t)input;

        if(error <= (int64_t)errorDeadband && error >= -(int64_t)errorDeadband){
            error = 0;
        }

        int64_t candidateOutput = (int64_t)kp * error / PID_SCALE;
        int64_t newIntegral = integral;
        if(error <= (int64_t)errIntegralThreshold && error >= -(int64_t)errIntegralThreshold){
            newIntegral += error * (int64_t)dt_us;
        }

        int64_t candidateI = 0;

        if(ki != 0){
            candidateI = (int64_t)ki * newIntegral / (TIME_SCALE * PID_SCALE);
        }

        if(candidateI > (int64_t)integralMax){
            candidateI = integralMax;
        }

        if(candidateI < (int64_t)integralMin){
            candidateI = integralMin;
        }

        candidateOutput += candidateI;

        if(!((candidateOutput > (int64_t)outputMax && error > 0) || (candidateOutput < (int64_t)outputMin && error < 0))){
            integral = newIntegral;
        }

        int64_t output = candidateOutput;

        if(output > (int64_t)outputMax){
            output = outputMax;
        }

        if(output < (int64_t)outputMin){
            output = outputMin;
        }
        previousError = error;

        return (int32_t)output;
    }

    int64_t error = (int64_t)target - (int64_t)input;

    if(error <= (int64_t)errorDeadband && error >= -(int64_t)errorDeadband){
        error = 0;
    }

    int64_t errorChange = error - (int64_t)previousError;

    int64_t D = errorChange * derivativeScale / PID_SCALE;

    D = filterDerivative(D);

    int64_t candidateOutput = (int64_t)kp * error / PID_SCALE;

    int64_t newIntegral = integral;

    if(error <= (int64_t)errIntegralThreshold && error >= -(int64_t)errIntegralThreshold){
        newIntegral += error * (int64_t)dt_us;
    }
    int64_t candidateI = 0;

    if(ki != 0){
        candidateI = (int64_t)ki * newIntegral / (TIME_SCALE * PID_SCALE);
    }

    if(candidateI > (int64_t)integralMax){
        candidateI = integralMax;
    }

    if(candidateI < (int64_t)integralMin){
        candidateI = integralMin;
    }

    candidateOutput += candidateI + D;

    if(!((candidateOutput > (int64_t)outputMax && error > 0) || (candidateOutput < (int64_t)outputMin && error < 0))){
        integral = newIntegral;
    }

    int64_t output = candidateOutput;

    if(output > (int64_t)outputMax){
        output = outputMax;
    }

    if(output < (int64_t)outputMin){
        output = outputMin;
    }

    previousError = error;

    return (int32_t)output;
}