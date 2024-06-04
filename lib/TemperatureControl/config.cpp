#include "config.h"

float Vref;
float GAIN;
float RREF = 402.0;
float RRTD;
float temp_PT100;
float R0 = 100.0;
float Voltage_therm;
float DataT1;
float Voltage;

float tempfinal;
float Rawdata;
// PID parameters
float Kp = 5.0;
float Ki = 1.0;
float Kd = 1.0;

float setpoint = 95;        //set this equal to UpperTemperatureThreshold
float input, output;
float iTerm, lastInput = 0;
float dInput, error;

// LED control variables
int led = 2;                //led pin
int ledBrightness = 0;      //led pwm control (0-255)
int fan = 5;                //fan pin


// Thermocycle control constants
const float UpperTemperatureThreshold = 95;
const float LowerTemperatureThreshold = 60;

int cycleCount = 1;
const int TotalCycles = 30;
