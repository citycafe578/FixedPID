#include <Arduino.h>
#include "esp_timer.h"
#include "FixedPID.h"

// ========================================================
// TEST RESULT
// ========================================================

int totalTests = 0;
int passedTests = 0;

void testResult(const char* name, int32_t expected, int32_t actual) {
  totalTests++;

  if (expected == actual) {
    passedTests++;
    Serial.print("[PASS] ");
    Serial.println(name);
  } else {
    Serial.print("[FAIL] ");
    Serial.println(name);
    Serial.print("       Expected : ");
    Serial.println(expected);
    Serial.print("       Actual   : ");
    Serial.println(actual);
  }
}

void testBool(const char* name, bool pass) {
  totalTests++;

  if (pass) {
    passedTests++;
    Serial.print("[PASS] ");
    Serial.println(name);
  } else {
    Serial.print("[FAIL] ");
    Serial.println(name);
  }
}

// ========================================================
// P TEST
// ========================================================

void testP() {
  FixedPID pid;
  pid.setTunings(1000, 0, 0);

  testResult("P #1", 100, pid.update(100, 0, 1000));

  pid.reset();
  testResult("P #2", 50, pid.update(100, 50, 1000));

  pid.reset();
  testResult("P #3 negative", -50, pid.update(50, 100, 1000));

  pid.reset();
  testResult("P #4 zero", 0, pid.update(100, 100, 1000));
}

// ========================================================
// I TEST
// ========================================================

void testI() {
  FixedPID pid;
  pid.setTunings(0, 1000000, 0);

  testResult("I #1", 100, pid.update(100, 0, 1000));
  testResult("I #2", 200, pid.update(100, 0, 1000));
  testResult("I #3", 300, pid.update(100, 0, 1000));
  testResult("I #4", 400, pid.update(100, 0, 1000));
}

// ========================================================
// D TEST
// ========================================================

void testD() {
  FixedPID pid;
  pid.setTunings(0, 0, 1000);

  testResult("D #1", 100000, pid.update(100, 0, 1000));
  testResult("D #2", 100000, pid.update(200, 0, 1000));
  testResult("D #3", -200000, pid.update(0, 0, 1000));
  testResult("D #4", -100000, pid.update(0, 100, 1000));

  pid.reset();
  testResult("D #5 reset", -100000, pid.update(0, 100, 1000));
}

// ========================================================
// DERIVATIVE FILTER TEST
// ========================================================

void testDerivativeFilter() {
  FixedPID pid;
  pid.setTunings(0, 0, 1000);
  pid.setDerivativeFilter(500);

  testResult("D filter variable #1", 50000,
             pid.update(100, 0, 1000));

  testResult("D filter variable #2", 75000,
             pid.update(200, 0, 1000));

  FixedPID fixedPid;
  fixedPid.setTunings(0, 0, 1000);
  fixedPid.setFrequency(1000);
  fixedPid.setDerivativeFilter(500);

  testResult("D filter fixed rate #1", 50000,
             fixedPid.updateFixedRate(100, 0));

  FixedPID velocityPid;
  velocityPid.setTunings(0, 0, 1000);
  velocityPid.setDerivativeFilter(500);

  testResult("D filter velocity #1", 50000,
             velocityPid.updateVelocity(100, 0, 1000));

  FixedPID velocityFixedPid;
  velocityFixedPid.setTunings(0, 0, 1000);
  velocityFixedPid.setFrequency(1000);
  velocityFixedPid.setDerivativeFilter(500);

  testResult("D filter velocity fixed rate #1", 50000,
             velocityFixedPid.updateVelocityFixedRate(100, 0));
}

// ========================================================
// DERIVATIVE FILTER EDGE CASE TEST
// ========================================================

void testDerivativeFilterEdgeCases() {

  // alpha = PID_SCALE
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setDerivativeFilter(PID_SCALE);

    testResult("D filter alpha=1000 #1",
               100000,
               pid.update(100, 0, 1000));

    testResult("D filter alpha=1000 #2",
               100000,
               pid.update(200, 0, 1000));
  }

  // alpha = 0
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setDerivativeFilter(0);

    testResult("D filter alpha=0 #1",
               0,
               pid.update(100, 0, 1000));

    testResult("D filter alpha=0 #2",
               0,
               pid.update(200, 0, 1000));
  }

  // alpha > PID_SCALE
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setDerivativeFilter(2000);

    testResult("D filter alpha overflow",
               100000,
               pid.update(100, 0, 1000));
  }

  // reset previousFilteredD
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setDerivativeFilter(500);

    pid.update(100, 0, 1000);
    pid.reset();

    testResult("D filter reset",
               50000,
               pid.update(100, 0, 1000));
  }
}

// ========================================================
// D / DT TEST
// ========================================================

void testDDt() {
  FixedPID pid;
  pid.setTunings(0, 0, 1000);

    testResult("D dt=1000",
               100000,
             pid.update(100, 0, 1000));

  pid.reset();

    testResult("D dt=2000",
               50000,
             pid.update(100, 0, 2000));
}

// ========================================================
// BASIC TEST
// ========================================================

void testBasic() {
  FixedPID pid;
  pid.setTunings(1000, 0, 0);

  testResult("Zero error",
             0,
             pid.update(100, 100, 1000));

  testResult("Negative error",
             -50,
             pid.update(50, 100, 1000));
}

// ========================================================
// RESET TEST
// ========================================================

void testReset() {
  FixedPID pid;
  pid.setTunings(0, 1000000, 0);

  pid.update(100, 0, 1000);
  pid.update(100, 0, 1000);

  pid.reset();

  testResult("Reset integral",
             100,
             pid.update(100, 0, 1000));

  pid.reset();

  bool pass = true;

  for (int i = 0; i < 100; i++) {
    pid.update(100, 0, 1000);
    pid.reset();

    int32_t output = pid.update(100, 0, 1000);

    if (output != 100) {
      pass = false;
      break;
    }
  }

  testBool("100 repeated resets", pass);
}

// ========================================================
// DT TEST
// ========================================================

void testDt() {
  FixedPID pid;
  pid.setTunings(1000, 0, 0);

  testResult("dt=0",
             0,
             pid.update(100, 0, 0));

  testResult("dt=1",
             100,
             pid.update(100, 0, 1));

  testResult("Large dt",
             100,
             pid.update(100, 0, 1000000));
}

// ========================================================
// OUTPUT LIMIT
// ========================================================

void testOutputLimit() {
  FixedPID pid;
  pid.setTunings(1000, 0, 0);
  pid.setOutputLimits(-50, 50);

  testResult("Output limit high",
             50,
             pid.update(100, 0, 1000));

  pid.reset();

  testResult("Output limit low",
             -50,
             pid.update(0, 100, 1000));
}

// ========================================================
// INTEGRAL LIMIT
// ========================================================

void testIntegralLimit() {
  FixedPID pid;
  pid.setTunings(0, 1000000, 0);
  pid.setIntegralLimits(-500, 500);

  for (int i = 0; i < 5; i++) {
    pid.update(100, 0, 1000);
  }

  testResult("Integral limit high",
             500,
             pid.update(0, 0, 1000));

  pid.reset();

  for (int i = 0; i < 5; i++) {
    pid.update(0, 100, 1000);
  }

  testResult("Integral limit low",
             -500,
             pid.update(0, 0, 1000));
}

// ========================================================
// ANTI WINDUP
// ========================================================

void testAntiWindup() {

  // 上限飽和
  {
    FixedPID pid;
    pid.setTunings(1000000, 1000000, 0);
    pid.setOutputLimits(-100, 100);
    pid.setIntegralLimits(-1000000, 1000000);

    pid.update(100, 0, 1000);

    int32_t output =
        pid.update(100, 0, 1000);

    testResult("Anti-windup high",
               100,
               output);
  }

  // 下限飽和
  {
    FixedPID pid;
    pid.setTunings(1000000, 1000000, 0);
    pid.setOutputLimits(-100, 100);
    pid.setIntegralLimits(-1000000, 1000000);

    pid.update(0, 100, 1000);

    int32_t output =
        pid.update(0, 100, 1000);

    testResult("Anti-windup low",
               -100,
               output);
  }

  // unwind
  {
    FixedPID pid;
    pid.setTunings(0, 1000000, 0);
    pid.setOutputLimits(-1000000, 1000000);

    pid.update(100, 0, 1000);
    pid.update(100, 0, 1000);
    pid.update(0, 0, 1000);

    int32_t output =
        pid.update(0, 0, 1000);

    testResult("Integral unwind",
               200,
               output);
  }

  // 長時間上限飽和
  {
    FixedPID pid;
    pid.setTunings(1000000, 1000000, 0);
    pid.setOutputLimits(-100, 100);
    pid.setIntegralLimits(-1000000000, 1000000000);

    for (int i = 0; i < 10000; i++) {
      pid.update(100, 0, 1000);
    }

    int32_t output =
        pid.update(0, 0, 1000);

    testResult("Anti-windup long high",
               0,
               output);
  }

  // 長時間下限飽和
  {
    FixedPID pid;
    pid.setTunings(1000000, 1000000, 0);
    pid.setOutputLimits(-100, 100);
    pid.setIntegralLimits(-1000000000, 1000000000);

    for (int i = 0; i < 10000; i++) {
      pid.update(0, 100, 1000);
    }

    int32_t output =
        pid.update(0, 0, 1000);

    testResult("Anti-windup long low",
               0,
               output);
  }

  // 上限飽和後反向
  {
    FixedPID pid;
    pid.setTunings(1000000, 1000000, 0);
    pid.setOutputLimits(-100, 100);
    pid.setIntegralLimits(-1000000, 1000000);

    for (int i = 0; i < 100; i++) {
      pid.update(100, 0, 1000);
    }

    int32_t output =
        pid.update(0, 100, 1000);

    testResult("Anti-windup reverse high",
               -100,
               output);
  }

  // 下限飽和後反向
  {
    FixedPID pid;
    pid.setTunings(1000000, 1000000, 0);
    pid.setOutputLimits(-100, 100);
    pid.setIntegralLimits(-1000000, 1000000);

    for (int i = 0; i < 100; i++) {
      pid.update(0, 100, 1000);
    }

    int32_t output =
        pid.update(100, 0, 1000);

    testResult("Anti-windup reverse low",
               100,
               output);
  }

  // 無飽和時 I 正常累積
  {
    FixedPID pid;
    pid.setTunings(0, 1000000, 0);
    pid.setOutputLimits(-1000000, 1000000);

    pid.update(100, 0, 1000);
    pid.update(100, 0, 1000);

    int32_t output =
        pid.update(100, 0, 1000);

    testResult("Integral without saturation",
               300,
               output);
  }
}

// ========================================================
// FIXED POINT
// ========================================================

void testFixedPoint() {
  FixedPID pid;
  pid.setTunings(1500, 0, 0);

  testResult("Fixed point P=1.5",
             150,
             pid.update(100, 0, 1000));
}

// ========================================================
// INT32 BOUNDARY
// ========================================================

void testInt32() {
  FixedPID pid;
  pid.setTunings(1000, 0, 0);

  testResult("INT32_MAX",
             INT32_MAX,
             pid.update(INT32_MAX, 0, 1000));

  pid.reset();

  testResult("INT32_MIN",
             INT32_MIN,
             pid.update(INT32_MIN, 0, 1000));

  pid.reset();

  testResult("MAX-MIN error",
             INT32_MAX,
             pid.update(INT32_MAX, INT32_MIN, 1000));

  pid.reset();

  testResult("MIN-MAX error",
             INT32_MIN,
             pid.update(INT32_MIN, INT32_MAX, 1000));
}

// ========================================================
// MAX / MIN KP
// ========================================================

void testKpLimits() {

  // INT32_MAX * 1 / 1000
  {
    FixedPID pid;
    pid.setTunings(INT32_MAX, 0, 0);
    pid.setOutputLimits(INT32_MIN, INT32_MAX);

    int32_t expected =
        (int32_t)(((int64_t)INT32_MAX) / PID_SCALE);

    testResult("MAX Kp",
               expected,
               pid.update(1, 0, 1000));
  }

  // INT32_MIN * 1 / 1000
  {
    FixedPID pid;
    pid.setTunings(INT32_MIN, 0, 0);
    pid.setOutputLimits(INT32_MIN, INT32_MAX);

    int32_t expected =
        (int32_t)(((int64_t)INT32_MIN) / PID_SCALE);

    testResult("MIN Kp",
               expected,
               pid.update(1, 0, 1000));
  }
}

// ========================================================
// DEAD BAND
// ========================================================

void testDeadband() {
  FixedPID pid;
  pid.setTunings(1000, 0, 0);
  pid.setErrorDeadband(10);

  testResult("Deadband inside",
             0,
             pid.update(100, 95, 1000));

  pid.reset();

  testResult("Deadband negative inside",
             0,
             pid.update(95, 100, 1000));

  pid.reset();

    testResult("Deadband outside",
               15,
             pid.update(100, 85, 1000));
}

// ========================================================
// ERROR INTEGRAL THRESHOLD
// ========================================================

void testErrorIntegralThreshold() {
  FixedPID pid;
  pid.setTunings(0, 1000000, 0);
  pid.setErrorIntegralThreshold(10);

  // error = 100，I 不應該累積
  testResult("Integral threshold blocked",
             0,
             pid.update(100, 0, 1000));

  // error = 10，可以累積
  pid.reset();

  testResult("Integral threshold allowed",
             10,
             pid.update(10, 0, 1000));
}

// ========================================================
// FREQUENCY TEST
// ========================================================

void testFrequency() {

  // 1000 Hz
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(1000);

    testResult("Frequency 1000 Hz",
               100000,
               pid.updateFixedRate(100, 0));
  }

  // 500 Hz
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(500);

    testResult("Frequency 500 Hz",
               50000,
               pid.updateFixedRate(100, 0));
  }

  // 2000 Hz
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(2000);

    testResult("Frequency 2000 Hz",
               200000,
               pid.updateFixedRate(100, 0));
  }

  // 0 Hz → invalid
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(0);

    testResult("Frequency 0 Hz",
               0,
               pid.updateFixedRate(100, 0));
  }

  // > 1 MHz → invalid
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(1000001);

    testResult("Frequency overflow",
               0,
               pid.updateFixedRate(100, 0));
  }
}

// ========================================================
// LONG RUN
// ========================================================

void testLongRun() {
  FixedPID pid;
  pid.setTunings(100, 10, 10);
  pid.setOutputLimits(-1000000000, 1000000000);

  bool pass = true;

  for (int i = 0; i < 10000; i++) {
    int32_t output =
        pid.update(1000, 950, 1000);

    if (output == 0) {
      pass = false;
      break;
    }
  }

  testBool("10000 repeated updates", pass);

  pid.reset();

  pass = true;

  for (int i = 0; i < 20000; i++) {
    uint32_t dt =
        900 + (i % 201);

    int32_t output =
        pid.update(1000, 950, dt);

    if (output == 0) {
      pass = false;
      break;
    }
  }

  testBool("20000 updates with dt jitter", pass);

  pid.reset();

  int32_t output =
      pid.update(1000, 950, 1000);

  FixedPID fresh;
  fresh.setTunings(100, 10, 10);
  fresh.setOutputLimits(-1000000000, 1000000000);

  testResult("Reset after long run",
             fresh.update(1000, 950, 1000),
             output);
}

// ========================================================
// FIXED RATE CORRECTNESS
// ========================================================

void testFixedRate() {
  FixedPID reference;
  FixedPID optimized;

  reference.setTunings(1500, 200, 800);

  optimized.setTunings(1500, 200, 800);
  optimized.setFrequency(1000);

  bool pass = true;

  for (int i = 0; i < 1000; i++) {

    int32_t target =
        1000 + ((i * 37) % 500);

    int32_t input =
        800 + ((i * 23) % 400);

    int32_t outReference =
        reference.update(target, input, 1000);

    int32_t outOptimized =
        optimized.updateFixedRate(target, input);

    if (outReference != outOptimized) {

      pass = false;

      Serial.println();
      Serial.print("Fixed rate mismatch at ");
      Serial.println(i);

      Serial.print("Reference : ");
      Serial.println(outReference);

      Serial.print("Optimized : ");
      Serial.println(outOptimized);

      break;
    }
  }

  testBool("Fixed rate 1000/1000 identical", pass);
}

// ========================================================
// VELOCITY P TEST
// ========================================================

void testVelocityP() {
  FixedPID pid;
  pid.setTunings(1000, 0, 0);

  testResult("Velocity P #1",
             100,
             pid.updateVelocity(100, 0, 1000));

  testResult("Velocity P #2",
             200,
             pid.updateVelocity(200, 0, 1000));

  testResult("Velocity P #3",
             150,
             pid.updateVelocity(150, 0, 1000));

  testResult("Velocity P #4",
             100,
             pid.updateVelocity(100, 0, 1000));
}

// ========================================================
// VELOCITY PI TEST
// ========================================================

void testVelocityPI() {
  FixedPID pid;
  pid.setTunings(1000, 1000000, 0);

  testResult("Velocity PI #1",
             200,
             pid.updateVelocity(100, 0, 1000));

  testResult("Velocity PI #2",
             500,
             pid.updateVelocity(200, 0, 1000));

  testResult("Velocity PI #3",
             600,
             pid.updateVelocity(150, 0, 1000));
}

// ========================================================
// VELOCITY D TEST
// ========================================================

void testVelocityD() {
  FixedPID pid;
  pid.setTunings(0, 0, 1000);

  testResult("Velocity D #1",
             100000,
             pid.updateVelocity(100, 0, 1000));

  testResult("Velocity D #2",
             100000,
             pid.updateVelocity(200, 0, 1000));

  testResult("Velocity D #3",
             -200000,
             pid.updateVelocity(0, 0, 1000));

  pid.reset();

  testResult("Velocity D reset",
             100000,
             pid.updateVelocity(100, 0, 1000));
}

// ========================================================
// VELOCITY D / DT TEST
// ========================================================

void testVelocityDDt() {
  FixedPID pid;
  pid.setTunings(0, 0, 1000);

  testResult("Velocity D dt=1000",
             100000,
             pid.updateVelocity(100, 0, 1000));

  pid.reset();

  testResult("Velocity D dt=2000",
             50000,
             pid.updateVelocity(100, 0, 2000));
}

// ========================================================
// VELOCITY RESET TEST
// ========================================================

void testVelocityReset() {
  FixedPID pid;
  pid.setTunings(1000, 0, 0);

  pid.updateVelocity(100, 0, 1000);
  pid.updateVelocity(200, 0, 1000);

  pid.reset();

  testResult("Velocity reset",
             100,
             pid.updateVelocity(100, 0, 1000));
}

// ========================================================
// VELOCITY OUTPUT LIMIT
// ========================================================

void testVelocityOutputLimit() {
  FixedPID pid;
  pid.setTunings(1000, 0, 0);
  pid.setOutputLimits(-150, 150);

  testResult("Velocity output limit high",
             150,
             pid.updateVelocity(200, 0, 1000));

  testResult("Velocity output limit low",
             -50,
             pid.updateVelocity(0, 0, 1000));
}

// ========================================================
// VELOCITY FIXED RATE CORRECTNESS
// ========================================================

void testVelocityFixedRate() {
  FixedPID reference;
  FixedPID optimized;

  reference.setTunings(1500, 200, 800);

  optimized.setTunings(1500, 200, 800);
  optimized.setFrequency(1000);

  bool pass = true;

  for (int i = 0; i < 1000; i++) {

    int32_t target =
        1000 + ((i * 37) % 500);

    int32_t input =
        800 + ((i * 23) % 400);

    int32_t outReference =
        reference.updateVelocity(
            target,
            input,
            1000);

    int32_t outOptimized =
        optimized.updateVelocityFixedRate(
            target,
            input);

    if (outReference != outOptimized) {

      pass = false;

      Serial.println();
      Serial.print("Velocity fixed rate mismatch at ");
      Serial.println(i);

      Serial.print("Reference : ");
      Serial.println(outReference);

      Serial.print("Optimized : ");
      Serial.println(outOptimized);

      break;
    }
  }

  testBool(
      "Velocity fixed rate 1000/1000 identical",
      pass);
}

// ========================================================
// PID SCALE CONTRACT
// ========================================================

void testPIDScale() {

  // P
  {
    FixedPID pid;
    pid.setTunings(1500, 0, 0);

    testResult("PID_SCALE P",
               75,
               pid.update(50, 0, 1000));
  }

  // I
  {
    FixedPID pid;
    // ki uses PID_SCALE, so 200.0 is represented by 200000.
    pid.setTunings(0, 200000, 0);

    testResult("PID_SCALE I #1",
               10,
               pid.update(50, 0, 1000));

    testResult("PID_SCALE I #2",
               20,
               pid.update(50, 0, 1000));
  }

  // D
  {
    FixedPID pid;
    pid.setTunings(0, 0, 500);

    testResult("PID_SCALE D",
               25000,
               pid.update(50, 0, 1000));
  }
}

// ========================================================
// FREQUENCY BOUNDARY TEST
// ========================================================

void testFrequencyBoundary() {

  // 最大有效頻率
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(1000000);

    testResult("Frequency max valid",
               100000000,
               pid.updateFixedRate(100, 0));
  }

  // 超過最大頻率
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(1000001);

    testResult("Frequency above max invalid",
               0,
               pid.updateFixedRate(100, 0));
  }

  // 零頻率
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(0);

    testResult("Frequency zero invalid",
               0,
               pid.updateFixedRate(100, 0));
  }
}

// ========================================================
// VELOCITY INTEGRAL REMAINDER
// ========================================================

void testVelocityIntegralRemainder() {

  FixedPID pid;

  // ki = 1
  // error = 1000
  // dt = 1000 us
  //
  // 每次：
  // 1 * 1000 * 1000 / 1,000,000 / 1000
  //
  // 單次不足 1 output，
  // 必須靠 remainder 長期累積。
  pid.setTunings(0, 1, 0);

  int32_t output = 0;

  for(int i = 0; i < 1000; i++){
    output =
        pid.updateVelocity(
          1000,
            0,
            1000);
  }

  testResult("Velocity integral remainder",
             1,
             output);

  pid.reset();

  output = 0;

  for(int i = 0; i < 2000; i++){
    output =
        pid.updateVelocity(
          1000,
            0,
            1000);
  }

  testResult("Velocity integral remainder x2",
             2,
             output);
}
// ========================================================
// NON-DIVISIBLE FREQUENCY TEST
// ========================================================

void testNonDivisibleFrequency() {

  // 3000 Hz
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(3000);

    testResult("Non-divisible frequency 3000 Hz",
               300000,
               pid.updateFixedRate(100, 0));
  }

  // 3333 Hz
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(3333);

    testResult("Non-divisible frequency 3333 Hz",
               333300,
               pid.updateFixedRate(100, 0));
  }

  // 1234 Hz
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(1234);

    testResult("Non-divisible frequency 1234 Hz",
               123400,
               pid.updateFixedRate(100, 0));
  }

  // Fixed-rate derivative must use kd * hz directly instead of
  // truncating TIME_SCALE / dt_us before multiplying by kd.
  {
    FixedPID pid;
    pid.setTunings(0, 0, 1000);
    pid.setFrequency(3);

    testResult("Frequency 3 Hz derivative scale",
               3000000,
               pid.updateFixedRate(1000000, 0));
  }
}

// ========================================================
// INT64 ERROR CHANGE
// ========================================================

void testInt64ErrorChange() {

  FixedPID pid;
  pid.setTunings(0, 0, 1000);

  // 第一次：
  // error = INT32_MAX
  pid.update(INT32_MAX, 0, 1000);

  // 第二次：
  // error = INT32_MIN
  //
  // errorChange =
  // INT32_MIN - INT32_MAX
  //
  // 這個值超過 int32_t，
  // 因此 previousError 必須是 int64_t。
  int32_t output =
      pid.update(
          INT32_MIN,
          0,
          1000);

  bool pass =
      output < 0;

  testBool(
      "INT64 error change",
      pass);

    FixedPID repeated;
    repeated.setTunings(0, 0, 1);

    int32_t first =
      repeated.update(
        INT32_MAX,
        INT32_MIN,
        1000000);

    int32_t second =
      repeated.update(
        INT32_MAX,
        INT32_MIN,
        1000000);

    testResult(
      "INT64 first extreme error",
      4294967,
      first);

    testResult(
      "INT64 repeated error has no D change",
      0,
      second);
}

// ========================================================
// INTEGRAL THRESHOLD TRANSITION
// ========================================================

void testIntegralThresholdTransition() {

  FixedPID pid;
  pid.setTunings(0, 1000000, 0);
  pid.setErrorIntegralThreshold(10);

  // error = 100
  // 不允許積分
  testResult("Threshold transition blocked",
             0,
             pid.update(100, 0, 1000));

  // error = 10
  // 開始積分
  testResult("Threshold transition allowed",
             10,
             pid.update(10, 0, 1000));

  // 再一次 error = 10
  testResult("Threshold transition accumulate",
             20,
             pid.update(10, 0, 1000));

  // 再次超過 threshold
  testResult("Threshold transition blocked again",
             20,
             pid.update(100, 0, 1000));

  // 回到 threshold
  testResult("Threshold transition resume",
             30,
             pid.update(10, 0, 1000));
}

// ========================================================
// BENCHMARK
// ========================================================

const int BENCH_LOOPS = 20000;
const uint32_t BENCH_DT_US = 1000;
const uint32_t BENCH_HZ = 1000;
const int32_t BENCH_TARGET = 1500;

volatile int32_t g_input = 1000;
volatile int32_t g_output = 0;
volatile uint32_t g_sink = 0;

void setupBenchCommon(FixedPID& pid) {
  pid.setOutputLimits(-5000, 5000);
  pid.setIntegralLimits(-2000000L, 2000000L);
  pid.setErrorDeadband(0);
  pid.setErrorIntegralThreshold(INT32_MAX);
  pid.setFrequency(BENCH_HZ);
  pid.reset();
}

float runBenchVariable(
    FixedPID& pid,
    const char* name) {

  g_input = 1000;
  g_output = 0;
  g_sink = 0;

  int64_t start =
      esp_timer_get_time();

  for (int i = 0; i < BENCH_LOOPS; i++) {

    g_output =
        pid.update(
            BENCH_TARGET,
            g_input,
            BENCH_DT_US);

    g_sink += (uint32_t)g_output;

    g_input +=
        (i & 1) ? 1 : -1;
  }

  int64_t end =
      esp_timer_get_time();

  float avg_us =
      (float)(end - start) /
      (float)BENCH_LOOPS;

  Serial.printf(
      "[Bench Variable] %s\n",
      name);

  Serial.printf(
      "  Average    : %.3f us / update\n",
      avg_us);

  Serial.printf(
      "  Last out   : %d\n\n",
      (int)g_output);

  return avg_us;
}

float runBenchFixed(
    FixedPID& pid,
    const char* name) {

  g_input = 1000;
  g_output = 0;
  g_sink = 0;

  int64_t start =
      esp_timer_get_time();

  for (int i = 0; i < BENCH_LOOPS; i++) {

    g_output =
        pid.updateFixedRate(
            BENCH_TARGET,
            g_input);

    g_sink += (uint32_t)g_output;

    g_input +=
        (i & 1) ? 1 : -1;
  }

  int64_t end =
      esp_timer_get_time();

  float avg_us =
      (float)(end - start) /
      (float)BENCH_LOOPS;

  Serial.printf(
      "[Bench FixedRate] %s\n",
      name);

  Serial.printf(
      "  Average    : %.3f us / update\n",
      avg_us);

  Serial.printf(
      "  Last out   : %d\n\n",
      (int)g_output);

  return avg_us;
}

void runBenchmark() {

  Serial.println();
  Serial.println("========================================");
  Serial.println("          FixedPID BENCHMARK");
  Serial.println("========================================");
  Serial.println();

  FixedPID pid;

  // Variable dt
  setupBenchCommon(pid);
  pid.setTunings(1000, 0, 0);

  float v_p =
      runBenchVariable(pid, "P only");

  setupBenchCommon(pid);
  pid.setTunings(1000, 100, 0);

  float v_pi =
      runBenchVariable(pid, "PI");

  setupBenchCommon(pid);
  pid.setTunings(1000, 100, 50);

  float v_pid =
      runBenchVariable(pid, "Full PID");

  // Fixed rate
  setupBenchCommon(pid);
  pid.setTunings(1000, 0, 0);

  float f_p =
      runBenchFixed(pid, "P only");

  setupBenchCommon(pid);
  pid.setTunings(1000, 100, 0);

  float f_pi =
      runBenchFixed(pid, "PI");

  setupBenchCommon(pid);
  pid.setTunings(1000, 100, 50);

  float f_pid =
      runBenchFixed(pid, "Full PID");

  Serial.println("=== Benchmark Summary ===");

  Serial.println("Variable-dt:");

  Serial.printf(
      "  P only   : %.3f us\n",
      v_p);

  Serial.printf(
      "  PI       : %.3f us\n",
      v_pi);

  Serial.printf(
      "  Full PID : %.3f us\n",
      v_pid);

  Serial.println("Fixed-rate:");

  Serial.printf(
      "  P only   : %.3f us\n",
      f_p);

  Serial.printf(
      "  PI       : %.3f us\n",
      f_pi);

  Serial.printf(
      "  Full PID : %.3f us\n",
      f_pid);

  Serial.println("========================================");
}

// ========================================================
// MAIN
// ========================================================

void setup() {

  Serial.begin(115200);

  unsigned long start = millis();

  while (!Serial && millis() - start < 5000) {
    delay(10);
  }

  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("          FixedPID TEST SUITE");
  Serial.println("========================================");

  totalTests = 0;
  passedTests = 0;

  testP();
  testI();
  testD();
  testDerivativeFilter();
  testDerivativeFilterEdgeCases();
  testDDt();
  testBasic();
  testReset();
  testDt();
  testOutputLimit();
  testIntegralLimit();
  testAntiWindup();
  testFixedPoint();
  testInt32();
  testKpLimits();
  testDeadband();
  testErrorIntegralThreshold();
  testFrequency();
  testLongRun();
  testFixedRate();

  // Velocity form
  testVelocityP();
  testVelocityPI();
  testVelocityD();
  testVelocityDDt();
  testVelocityReset();
  testVelocityOutputLimit();
  testVelocityFixedRate();

  // Additional regression tests
  testPIDScale();
  testFrequencyBoundary();
  testVelocityIntegralRemainder();
  testNonDivisibleFrequency();
  testInt64ErrorChange();
  testIntegralThresholdTransition();

  Serial.println();
  Serial.println("========================================");
  Serial.println("             TEST RESULT");
  Serial.println("========================================");
  Serial.print("Total : ");
  Serial.println(totalTests);
  Serial.print("PASS  : ");
  Serial.println(passedTests);
  Serial.print("FAIL  : ");
  Serial.println(totalTests - passedTests);
  Serial.println("========================================");

  if (passedTests == totalTests) {
    Serial.println("       ALL TESTS PASSED");
  } else {
    Serial.println("       SOME TESTS FAILED");
  }

  Serial.println("========================================");

  runBenchmark();

  float temp_c =
      temperatureRead();

  Serial.printf(
      "Chip Temperature: %.2f °C\n",
      temp_c);
}

void loop() {
  static unsigned long lastSummary = 0;

  if(millis() - lastSummary >= 5000){
    lastSummary = millis();

    Serial.println();
    Serial.println("[TEST SUMMARY]");
    Serial.print("Total : ");
    Serial.println(totalTests);
    Serial.print("PASS  : ");
    Serial.println(passedTests);
    Serial.print("FAIL  : ");
    Serial.println(totalTests - passedTests);
  }
}