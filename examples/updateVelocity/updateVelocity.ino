#include <Arduino.h>
#include "FixedPID.h"

FixedPID pid;

int32_t actuatorCommand = 0;
uint32_t sample = 0;

void setup(){
	Serial.begin(115200);

	// Velocity form updates the actuator command from changes in error.
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

	// This example treats actuatorCommand as a position-like actuator value.
	int32_t input = actuatorCommand/10;
	int32_t output = pid.updateVelocityFixedRate(target, input);
	actuatorCommand = output;

	if (sample % 25 == 0) {
		Serial.print("Target: ");
		Serial.print(target);
		Serial.print("  Input: ");
		Serial.print(input);
		Serial.print("  Actuator command: ");
		Serial.println(actuatorCommand);
	}

	// Reset the complete velocity-form state after a simulated restart.
	if (sample == 3000) {
		pid.reset();
		actuatorCommand = 0;
		Serial.println("Controller reset");
	}

	sample++;
	delay(1);
}
