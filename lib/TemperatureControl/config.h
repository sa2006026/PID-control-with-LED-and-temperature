// globals.h
#ifndef CONFIG_H
#define CONFIG_H

extern float Vref;
extern float GAIN;
extern float RREF;
extern float RRTD;
extern float temp_PT100;
extern float R0;
extern float Voltage_therm;
extern float DataT1;
extern float Voltage;

extern float tempfinal;
extern float Rawdata;
// PID parameters
extern float Kp;
extern float Ki;
extern float Kd;

extern float setpoint;
extern float input, output;
extern float iTerm, lastInput;
extern float dInput, error;

// LED control variables
extern int led;
extern int led2;
extern int ledBrightness;

extern int fan;
extern int fan2;

// Thermocycle control constants
extern const float UpperTemperatureThreshold;
extern const float LowerTemperatureThreshold;

extern int cycleCount;
extern const int TotalCycles;

#endif
