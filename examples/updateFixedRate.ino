#include <Arduino.h>
#include "FixedPID.h"

FixedPID pid;

int32_t simulatedInput = 0;
uint32_t sample = 0;

void setup(){
    Serial.begin(115200);

    // Fixed-rate mode precomputes the time-dependent PID scaling.
    pid.setTunings(
        1500,   // Kp = 1.5
        200,    // Ki = 0.2
        50      // Kd = 0.05
    );

    pid.setOutputLimits(-10000, 10000);
    pid.setIntegralLimits(-3000, 3000);
    pid.setFrequency(1000);
    pid.setErrorDeadband(8);
    pid.setErrorIntegralThreshold(250);
    pid.setDerivativeFilter(250);
}

void loop(){
    int32_t target = 1000;

    // Add a small, deterministic sensor disturbance to show deadband and D filtering.
    int32_t noise = (int32_t)(sample % 7) - 3;
    int32_t measuredInput = simulatedInput + noise*3;
    int32_t output = pid.updateFixedRate(target, measuredInput);

    // A tiny first-order plant model: the output changes the measured position.
    simulatedInput += output/500;

    if (sample % 25 == 0) {
        Serial.print("Target: ");
        Serial.print(target);
        Serial.print("  Input: ");
        Serial.print(measuredInput);
        Serial.print("  Output: ");
        Serial.println(output);
    }

    sample++;

    delay(1);
}