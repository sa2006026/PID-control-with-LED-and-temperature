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

extern bool isHeatingPhase;
extern float cycleStartTime;
extern float heatingHoldTime;
extern float coolingHoldTime;

extern int cycleCount;
extern const int TotalCycles;

extern  float tempPhase1; // Target temperature for phase 1
extern float tempPhase2; // Target temperature for phase 2
extern  float tempPhase3; // Target temperature for phase 3

extern  unsigned long holdTimePhase1 ; // Hold time for phase 1 in milliseconds (15 seconds)
extern  unsigned long holdTimePhase2 ; // Hold time for phase 2 in milliseconds (30 seconds)
extern  unsigned long holdTimePhase3 ; // Hold time for phase 3 in milliseconds (30 seconds)

extern int phase ;  // Start with phase 1

extern unsigned long initialDenaturationTime; // 2 minutes in milliseconds
extern unsigned long finalExtensionTime;      // 5 minutes in milliseconds
extern unsigned long phaseStartTime;

#endif
