#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Modified these to be non-constant if they need to be changed at runtime
extern float Vref;
extern float GAIN;

extern float setpoint;

constexpr float RREF = 402.0;
constexpr float R0 = 100.0;

constexpr float Kp = 5.0;
constexpr float Ki = 1.0;
constexpr float Kd = 1.0;

constexpr float UpperTemperatureThreshold = 95;
constexpr float LowerTemperatureThreshold = 60;
constexpr int TotalCycles = 30;

const int led = 5;
const int fan = 3;

extern float temp_PT100;
extern float Voltage_therm;
extern float tempfinal;
extern float Rawdata;
extern float input;
extern float output;
extern float iTerm;
extern float lastInput;
extern float dInput;
extern float error;
extern int ledBrightness;
extern int cycleCount;
extern float RRTD;
extern float DataT1;
extern float Voltage;

#endif // CONFIG_H
