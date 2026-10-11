#include <stdint.h>
#include "FixedPID.h"

// Range protection
// ----------------
// Any int32 target/input/gain, any uint32 dt and any valid frequency are
// accepted. Every intermediate product that could leave the int64 range goes
// through satAdd()/satMul(), which saturate instead of overflowing, so there is
// no undefined behavior and the result always ends up inside the output limits.
// Results are exact whenever the intermediate values fit in int64.

namespace {

const int64_t INTEGRAL_DIV = TIME_SCALE * PID_SCALE;
const int64_t TIME_PER_SCALE = TIME_SCALE / PID_SCALE;   // 1000, exact
const int     FRACTION_BITS = 20;
const int64_t FRACTION_ONE  = 1LL << FRACTION_BITS;
const int64_t FILTER_LIMIT  = INT64_MAX / PID_SCALE;
const int64_t CARRY_LIMIT   = 1LL << 40;
const int64_t MUL1000_LIMIT = INT64_MAX / TIME_PER_SCALE; // |num| below this: num * 1000 fits

// True when v is inside the int32 range. Two such values always multiply inside int64.
inline bool fitsInt32(int64_t v){
    return (uint64_t)v + 0x80000000ULL <= 0xFFFFFFFFULL;
}

inline int64_t satAdd(int64_t a, int64_t b){
    int64_t r;
    if(__builtin_add_overflow(a, b, &r)){
        return b > 0 ? INT64_MAX : INT64_MIN;
    }
    return r;
}

inline int64_t satMul(int64_t a, int64_t b){
    if(fitsInt32(a) && fitsInt32(b)){
        return a * b;                            // common case: no overflow check needed
    }
    int64_t r;
    if(__builtin_mul_overflow(a, b, &r)){
        return ((a < 0) != (b < 0)) ? INT64_MIN : INT64_MAX;
    }
    return r;
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

// num / den with an exact remainder carried to the next call (constant denominator).
// When the quotient is far beyond any int32 output the remainder is meaningless
// and is dropped (the variable-dt helper below does the same, so both stay identical).
inline int64_t divCarry(int64_t num, int64_t den, int64_t& rem){
    int64_t q = num / den;
    if(q > CARRY_LIMIT || q < -CARRY_LIMIT){
        rem = 0;
        return q;
    }
    if(rem == 0){
        rem = num - q * den;                     // no carried remainder: one division is enough
        return q;
    }
    num = satAdd(num, rem);                      // num may already be saturated
    q = num / den;
    rem = num - q * den;
    return q;
}

// num * 1000 / den for den > 0, exact (no overflow of the multiplication).
// 64-bit division is a libgcc software routine on the ESP32 (no hardware divider
// for it), but 32-bit division is a single instruction. These helpers divide
// |x| by a constant below 2^21 using only 32-bit divides, then restore the sign,
// which is exactly C's truncation toward zero.
inline uint64_t udivSmall(uint64_t u, uint32_t d){
    if((u >> 32) == 0){
        return (uint32_t)u / d;                       // whole value fits: one divide
    }
    uint32_t hi = (uint32_t)(u >> 32);
    uint32_t lo = (uint32_t)u;
    uint32_t q  = hi / d;
    uint32_t r  = hi - q * d;                         // r < d < 2^21
    uint64_t result = (uint64_t)q << 32;

    uint32_t t, c;
    t = (r << 11) | (lo >> 21);  c = t / d; r = t - c * d; result += (uint64_t)c << 21;
    t = (r << 11) | ((lo >> 10) & 0x7FFu); c = t / d; r = t - c * d; result += (uint64_t)c << 10;
    t = (r << 10) | (lo & 0x3FFu); c = t / d; result += c;
    return result;
}

// x / 1000
inline int64_t div1000(int64_t x){
    if(x >= 0){
        return (int64_t)udivSmall((uint64_t)x, 1000u);
    }
    return -(int64_t)udivSmall(0 - (uint64_t)x, 1000u);
}

// x / 1e9  (= x >> 9, then / 5^9; the shift of |x| keeps truncation exact)
inline int64_t div1e9(int64_t x){
    if(x >= 0){
        return (int64_t)udivSmall(((uint64_t)x) >> 9, 1953125u);
    }
    return -(int64_t)udivSmall((0 - (uint64_t)x) >> 9, 1953125u);
}

inline int64_t mulDiv1000(int64_t num, int64_t den){
    if(num < MUL1000_LIMIT && num > -MUL1000_LIMIT){
        int64_t scaled = num * TIME_PER_SCALE;
        if(den > 0 && fitsInt32(scaled)){               // small values: hardware divide
            if(scaled >= 0){
                return (uint32_t)scaled / (uint32_t)den;
            }
            return -(int64_t)((uint32_t)(-scaled) / (uint32_t)den);
        }
        return scaled / den;
    }
    int64_t q = num / den;
    int64_t r = num % den;                       // |r| < den <= 2^32
    return satAdd(satMul(q, TIME_PER_SCALE), (r * TIME_PER_SCALE) / den);
}

// a * num / den for 0 <= num < den, exact and overflow-free:
// a = q*den + r  =>  a*num/den = q*num + r*num/den   (|q*num| < |a|).
inline int64_t mulFrac(int64_t a, int64_t num, int64_t den){
    int64_t q = a / den;
    int64_t r = a % den;                         // |r| < den <= 1e6
    return q * num + (r * num) / den;
}

// Same as mulDiv1000, but the sub-LSB remainder is carried to the next call as a
// signed Q20 fraction, so it stays valid when den (dt) changes between calls.
// The carry is added to the exact value of this call BEFORE truncating, which
// is the same accumulation rule divCarry() uses for a constant denominator.
inline int64_t mulDiv1000Carry(int64_t num, int64_t den, int64_t& frac){
    int64_t whole;
    int64_t r2;                                  // remainder of (num * 1000) / den

    if(num < MUL1000_LIMIT && num > -MUL1000_LIMIT){
        int64_t t = num * TIME_PER_SCALE;
        whole = t / den;
        r2 = t - whole * den;
    }else{
        int64_t q = num / den;
        int64_t r = num % den;
        whole = satAdd(satMul(q, TIME_PER_SCALE), (r * TIME_PER_SCALE) / den);
        r2 = (r * TIME_PER_SCALE) % den;
    }

    if(whole > CARRY_LIMIT || whole < -CARRY_LIMIT){
        frac = 0;                                // saturating territory: the fraction is irrelevant
        return whole;
    }

    if(r2 == 0 && frac == 0){
        return whole;                            // exact division, nothing to carry
    }

    int64_t total = whole * FRACTION_ONE + (r2 * FRACTION_ONE) / den + frac;
    frac = total % FRACTION_ONE;
    return total / FRACTION_ONE;
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
    integralMulLimit = INT64_MAX;
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
    integralMulLimit = (ki == 0) ? INT64_MAX
                                 : INT64_MAX / (ki < 0 ? -(int64_t)ki : (int64_t)ki);
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

    // Keep alpha * rawD inside int64 (alpha <= PID_SCALE).
    rawD = clamp64(rawD, -FILTER_LIMIT, FILTER_LIMIT);

    int64_t filteredD = div1000(
        (int64_t)alpha * rawD +
        (PID_SCALE - (int64_t)alpha) * previousFilteredD);

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

// Shared tail of the position-form paths: I clamp (state is clamped too, so it
// cannot wind up behind the limit), anti-windup, output clamp.
int32_t FixedPID::finishPosition(int64_t error, int64_t newIntegral, int64_t newIntegralFrac, int64_t D){
    // |kp| <= 2^31 and |error| < 2^32, so this product always fits in int64.
    int64_t P = div1000((int64_t)kp * error);
    int64_t I = 0;

    if(ki != 0){
        // The integral is a large int64 after a while; compare against a limit that
        // setTunings() precomputed instead of running the overflow-checked multiply.
        int64_t prod = (newIntegral < integralMulLimit && newIntegral > -integralMulLimit)
                           ? (int64_t)ki * newIntegral
                           : satMul((int64_t)ki, newIntegral);
        I = div1e9(prod);

        if(I > (int64_t)integralMax){
            I = integralMax;
            newIntegral = (int64_t)integralMax * INTEGRAL_DIV / (int64_t)ki;
        }else if(I < (int64_t)integralMin){
            I = integralMin;
            newIntegral = (int64_t)integralMin * INTEGRAL_DIV / (int64_t)ki;
        }
    }

    // |P| < 2^54 and |I| <= 2^31 (clamped above), so P + I cannot overflow; only D can be huge.
    int64_t candidateOutput = satAdd(P + I, D);

    if(!((candidateOutput > (int64_t)outputMax && error > 0) ||
         (candidateOutput < (int64_t)outputMin && error < 0))){
        integral = newIntegral;
        integralFracRemainder = newIntegralFrac;
    }

    previousError = error;

    return (int32_t)clamp64(candidateOutput, outputMin, outputMax);
}

int32_t FixedPID::finishVelocity(int64_t error, int64_t deltaP, int64_t deltaI, int64_t deltaD){
    velocityOutput = clamp64(satAdd(satAdd(satAdd(velocityOutput, deltaP), deltaI), deltaD),
                             outputMin, outputMax);
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
        newIntegral = satAdd(newIntegral, satMul(error, (int64_t)dt_us));
    }

    int64_t D = 0;
    if(kd != 0){
        D = mulDiv1000(satMul((int64_t)kd, error - previousError), (int64_t)dt_us);
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
        // error < 2^32 and dt_us <= 1e6, so this product fits.
        int64_t inc = error * (int64_t)dt_us;

        // 1e6/hz is not an integer for most frequencies; carry the fraction
        // instead of dropping it, so the integral time base stays exact.
        if(dtRemainder != 0){
            int64_t n = error * dtRemainder + newFrac;
            inc += n / (int64_t)hz;
            newFrac = n % (int64_t)hz;
        }
        newIntegral = satAdd(newIntegral, inc);
    }

    int64_t D = 0;
    if(kd != 0){
        D = div1000(satMul(error - previousError, derivativeScale));
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

    int64_t deltaP = divCarry(satMul((int64_t)kp, errorChange), PID_SCALE, velocityPRemainder);

    int64_t deltaI = 0;
    if(withinIntegralThreshold(error)){
        deltaI = divCarry(satMul(satMul((int64_t)ki, error), (int64_t)dt_us),
                          INTEGRAL_DIV, velocityIntegralRemainder);
    }

    int64_t deltaD = 0;
    if(kd != 0){
        deltaD = mulDiv1000Carry(satMul((int64_t)kd, secondErrorChange),
                                 (int64_t)dt_us, velocityDFraction);
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

    int64_t deltaP = divCarry(satMul((int64_t)kp, errorChange), PID_SCALE, velocityPRemainder);

    int64_t deltaI = 0;
    if(withinIntegralThreshold(error)){
        int64_t kiError = (int64_t)ki * error;     // fits: |ki| <= 2^31, |error| < 2^32
        int64_t term = satMul(kiError, (int64_t)dt_us);
        if(dtRemainder != 0){
            term = satAdd(term, mulFrac(kiError, dtRemainder, (int64_t)hz));
        }
        deltaI = divCarry(term, INTEGRAL_DIV, velocityIntegralRemainder);
    }

    int64_t deltaD = 0;
    if(kd != 0){
        deltaD = divCarry(satMul(secondErrorChange, derivativeScale), PID_SCALE, velocityDRemainder);
        deltaD = filterDerivative(deltaD);
    }

    return finishVelocity(error, deltaP, deltaI, deltaD);
}
