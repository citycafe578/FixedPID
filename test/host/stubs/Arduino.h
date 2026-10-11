// Minimal Arduino/ESP32 stand-ins so test/test.cpp builds and runs on a PC.
#pragma once
#include <stdint.h>
#include <stdio.h>

struct HostSerial {
    void begin(int) {}
    operator bool() const { return true; }
    void print(const char* s)        { ::printf("%s", s); }
    void print(int v)                { ::printf("%d", v); }
    void print(long v)               { ::printf("%ld", v); }
    void print(unsigned v)           { ::printf("%u", v); }
    void print(unsigned long v)      { ::printf("%lu", v); }
    void print(long long v)          { ::printf("%lld", v); }
    void println()                   { ::puts(""); }
    void println(const char* s)      { ::puts(s); }
    void println(int v)              { ::printf("%d\n", v); }
    void println(long v)             { ::printf("%ld\n", v); }
    void println(unsigned v)         { ::printf("%u\n", v); }
    void println(unsigned long v)    { ::printf("%lu\n", v); }
    void println(long long v)        { ::printf("%lld\n", v); }
    template <class... A> void printf(const char* f, A... a) { ::printf(f, a...); }
};

static HostSerial Serial;
static inline unsigned long millis() { return 100000; }
static inline void delay(int) {}
static inline float temperatureRead() { return 0; }
