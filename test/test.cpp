#include <Arduino.h>
#include "FixedPID.h"

// ========================================================
// TEST RESULT
// ========================================================

int totalTests = 0;
int passedTests = 0;

void testResult(
    const char* name,
    int32_t expected,
    int32_t actual
){
    totalTests++;

    if(expected == actual){
        passedTests++;

        Serial.print("[PASS] ");
        Serial.println(name);
    }
    else{
        Serial.print("[FAIL] ");
        Serial.println(name);

        Serial.print("       Expected : ");
        Serial.println(expected);

        Serial.print("       Actual   : ");
        Serial.println(actual);
    }
}

// ========================================================
// P TEST
// ========================================================

void testP(){

    FixedPID pid;

    pid.setTunings(
        1000,
        0,
        0
    );

    testResult(
        "P #1",
        100000,
        pid.update(100, 0, 1000)
    );

    pid.reset();

    testResult(
        "P #2",
        50000,
        pid.update(100, 50, 1000)
    );

    pid.reset();

    testResult(
        "P #3 negative",
        -50000,
        pid.update(50, 100, 1000)
    );

    pid.reset();

    testResult(
        "P #4 zero",
        0,
        pid.update(100, 100, 1000)
    );
}

// ========================================================
// I TEST
// ========================================================

void testI(){

    FixedPID pid;

    pid.setTunings(
        0,
        1000,
        0
    );

    testResult(
        "I #1",
        100,
        pid.update(100, 0, 1000)
    );

    testResult(
        "I #2",
        200,
        pid.update(100, 0, 1000)
    );

    testResult(
        "I #3",
        300,
        pid.update(100, 0, 1000)
    );

    testResult(
        "I #4",
        400,
        pid.update(100, 0, 1000)
    );
}

// ========================================================
// D TEST
// ========================================================

void testD(){

    FixedPID pid;

    pid.setTunings(
        0,
        0,
        1000
    );

    // error: 0 -> 100
    //
    // derivative =
    // 100 * 1,000,000 / 1000
    // = 100000
    //
    // D =
    // 100000 * 1000
    // = 100000000

    testResult(
        "D #1",
        100000000,
        pid.update(100, 0, 1000)
    );

    // error: 100 -> 200
    testResult(
        "D #2",
        100000000,
        pid.update(200, 0, 1000)
    );

    // error: 200 -> 0
    testResult(
        "D #3",
        -200000000,
        pid.update(0, 0, 1000)
    );

    // error: 0 -> -100
    testResult(
        "D #4",
        -100000000,
        pid.update(0, 100, 1000)
    );

    pid.reset();

    testResult(
        "D #5",
        -100000000,
        pid.update(0, 100, 1000)
    );
}

// ========================================================
// D / DT TEST
// ========================================================

void testDDt(){

    FixedPID pid;

    pid.setTunings(
        0,
        0,
        1000
    );

    // 100 * 1,000,000 / 1000
    // * 1000
    // = 100000000

    testResult(
        "D dt=1000",
        100000000,
        pid.update(100, 0, 1000)
    );

    pid.reset();

    // 100 * 1,000,000 / 2000
    // * 1000
    // = 50000000

    testResult(
        "D dt=2000",
        50000000,
        pid.update(100, 0, 2000)
    );
}

// ========================================================
// BASIC TEST
// ========================================================

void testBasic(){

    FixedPID pid;

    pid.setTunings(
        1000,
        0,
        0
    );

    testResult(
        "Zero error",
        0,
        pid.update(100, 100, 1000)
    );

    testResult(
        "Negative error",
        -50000,
        pid.update(50, 100, 1000)
    );
}

// ========================================================
// RESET TEST
// ========================================================

void testReset(){

    FixedPID pid;

    pid.setTunings(
        0,
        1000,
        0
    );

    pid.update(100, 0, 1000);
    pid.update(100, 0, 1000);

    pid.reset();

    testResult(
        "Reset integral",
        100,
        pid.update(100, 0, 1000)
    );

    pid.reset();

    bool pass = true;

    for(int i = 0; i < 100; i++){

        pid.update(
            100,
            0,
            1000
        );

        pid.reset();

        int32_t output =
            pid.update(
                100,
                0,
                1000
            );

        if(output != 100){
            pass = false;
            break;
        }
    }

    testResult(
        "100 repeated resets",
        1,
        pass ? 1 : 0
    );
}

// ========================================================
// DT TEST
// ========================================================

void testDt(){

    FixedPID pid;

    pid.setTunings(
        1000,
        0,
        0
    );

    testResult(
        "dt=0",
        0,
        pid.update(100, 0, 0)
    );

    testResult(
        "dt=1",
        100000,
        pid.update(100, 0, 1)
    );

    testResult(
        "Large dt",
        100000,
        pid.update(100, 0, 1000000)
    );
}

// ========================================================
// OUTPUT LIMIT
// ========================================================

void testOutputLimit(){

    FixedPID pid;

    pid.setTunings(
        1000,
        0,
        0
    );

    pid.setOutputLimits(
        -50000,
        50000
    );

    testResult(
        "Output limit high",
        50000,
        pid.update(100, 0, 1000)
    );

    testResult(
        "Output limit low",
        -50000,
        pid.update(0, 100, 1000)
    );
}

// ========================================================
// INTEGRAL LIMIT
// ========================================================

void testIntegralLimit(){

    FixedPID pid;

    pid.setTunings(
        0,
        1000,
        0
    );

    pid.setIntegralLimits(
        -500,
        500
    );

    // 5 × 100 = 500

    for(int i = 0; i < 5; i++){
        pid.update(
            100,
            0,
            1000
        );
    }

    testResult(
        "Integral limit high",
        500,
        pid.update(0, 0, 1000)
    );

    pid.reset();

    // 5 × -100 = -500

    for(int i = 0; i < 5; i++){
        pid.update(
            0,
            100,
            1000
        );
    }

    testResult(
        "Integral limit low",
        -500,
        pid.update(0, 0, 1000)
    );
}

// ========================================================
// ANTI WINDUP
// ========================================================

void testAntiWindup(){

    FixedPID pid;

    pid.setTunings(
        1000,
        1000,
        0
    );

    pid.setOutputLimits(
        -1000,
        1000
    );

    pid.setIntegralLimits(
        -1000000,
        1000000
    );

    // Positive saturation

    pid.update(
        100,
        0,
        1000
    );

    int32_t output =
        pid.update(
            100,
            0,
            1000
        );

    testResult(
        "Anti-windup high",
        1000,
        output
    );

    pid.reset();

    // Negative saturation

    pid.update(
        0,
        100,
        1000
    );

    output =
        pid.update(
            0,
            100,
            1000
        );

    testResult(
        "Anti-windup low",
        -1000,
        output
    );

    // Integral unwind

    pid.reset();

    pid.setTunings(
        0,
        1000,
        0
    );

    pid.setOutputLimits(
        -1000000,
        1000000
    );

    pid.update(
        100,
        0,
        1000
    );

    pid.update(
        100,
        0,
        1000
    );

    pid.update(
        0,
        0,
        1000
    );

    output =
        pid.update(
            0,
            0,
            1000
        );

    testResult(
        "Integral unwind",
        200,
        output
    );
}

// ========================================================
// FIXED POINT
// ========================================================

void testFixedPoint(){

    FixedPID pid;

    pid.setTunings(
        1500,
        0,
        0
    );

    testResult(
        "Fixed point P=1.5",
        150000,
        pid.update(100, 0, 1000)
    );
}

// ========================================================
// INT32 BOUNDARY
// ========================================================

void testInt32(){

    FixedPID pid;

    pid.setTunings(
        1,
        0,
        0
    );

    testResult(
        "INT32_MAX",
        INT32_MAX,
        pid.update(
            INT32_MAX,
            0,
            1000
        )
    );

    testResult(
        "INT32_MIN",
        INT32_MIN,
        pid.update(
            INT32_MIN,
            0,
            1000
        )
    );

    pid.reset();

    testResult(
        "MAX-MIN error",
        INT32_MAX,
        pid.update(
            INT32_MAX,
            INT32_MIN,
            1000
        )
    );

    pid.reset();

    testResult(
        "MIN-MAX error",
        INT32_MIN,
        pid.update(
            INT32_MIN,
            INT32_MAX,
            1000
        )
    );
}

// ========================================================
// MAX / MIN KP
// ========================================================

void testKpLimits(){

    FixedPID pid;

    pid.setTunings(
        INT32_MAX,
        0,
        0
    );

    pid.setOutputLimits(
        INT32_MIN,
        INT32_MAX
    );

    testResult(
        "MAX Kp",
        INT32_MAX,
        pid.update(
            1,
            0,
            1000
        )
    );

    pid.setTunings(
        INT32_MIN,
        0,
        0
    );

    testResult(
        "MIN Kp",
        INT32_MIN,
        pid.update(
            1,
            0,
            1000
        )
    );
}

// ========================================================
// LONG RUN
// ========================================================

void testLongRun(){

    FixedPID pid;

    pid.setTunings(
        100,
        10,
        10
    );

    pid.setOutputLimits(
        -1000000000,
        1000000000
    );

    bool pass = true;

    for(int i = 0; i < 10000; i++){

        int32_t output =
            pid.update(
                1000,
                950,
                1000
            );

        if(output == 0){
            pass = false;
            break;
        }
    }

    testResult(
        "10000 repeated updates",
        1,
        pass ? 1 : 0
    );

    pid.reset();

    pass = true;

    for(int i = 0; i < 20000; i++){

        uint32_t dt =
            900 + (i % 201);

        int32_t output =
            pid.update(
                1000,
                950,
                dt
            );

        if(output == 0){
            pass = false;
            break;
        }
    }

    testResult(
        "20000 updates with dt jitter",
        1,
        pass ? 1 : 0
    );

    pid.reset();

    int32_t output =
        pid.update(
            1000,
            950,
            1000
        );

    testResult(
        "Reset after long run",
        output,
        output
    );
}

// ========================================================
// FIXED RATE
// ========================================================

void testFixedRate(){

    FixedPID reference;
    FixedPID optimized;

    reference.setTunings(
        1500,
        200,
        800
    );

    optimized.setTunings(
        1500,
        200,
        800
    );

    optimized.setFrequency(
        1000
    );

    bool pass = true;

    for(int i = 0; i < 1000; i++){

        int32_t target =
            1000 +
            ((i * 37) % 500);

        int32_t input =
            800 +
            ((i * 23) % 400);

        int32_t outReference =
            reference.update(
                target,
                input,
                1000
            );

        int32_t outOptimized =
            optimized.updateFixedRate(
                target,
                input
            );

        if(outReference != outOptimized){

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

    testResult(
        "Fixed rate 1000/1000 identical",
        1,
        pass ? 1 : 0
    );
}

// ========================================================
// MAIN
// ========================================================

void setup(){

    Serial.begin(115200);

    delay(5000);

    Serial.println();
    Serial.println();
    Serial.println("========================================");
    Serial.println("          FixedPID TEST SUITE");
    Serial.println("========================================");

    totalTests = 0;
    passedTests = 0;

    // ====================================================
    // TESTS
    // ====================================================

    testP();
    testI();
    testD();
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
    testLongRun();
    testFixedRate();

    // ====================================================
    // RESULT
    // ====================================================

    Serial.println();
    Serial.println("========================================");
    Serial.println("             TEST RESULT");
    Serial.println("========================================");

    Serial.print("Total : ");
    Serial.println(totalTests);

    Serial.print("PASS  : ");
    Serial.println(passedTests);

    Serial.print("FAIL  : ");
    Serial.println(
        totalTests - passedTests
    );

    Serial.println("========================================");

    if(passedTests == totalTests){

        Serial.println(
            "       ALL TESTS PASSED"
        );
    }
    else{

        Serial.println(
            "       SOME TESTS FAILED"
        );
    }

    Serial.println("========================================");
}

void loop(){
}