#include <Arduino.h>
#include "FixedPID.h"

FixedPID pid;

void setup(){
    Serial.begin(115200);

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

void loop(){
    int32_t target = 1000;
    int32_t input = 900;

    uint32_t dt_us = 1000;

    int32_t output = pid.update(
        target,
        input,
        dt_us
    );

    Serial.print("Target: ");
    Serial.print(target);

    Serial.print("  Input: ");
    Serial.print(input);

    Serial.print("  Output: ");
    Serial.println(output);

    delay(1);
}