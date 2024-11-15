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
float Kp = 50.0;      //7  //50 cannot be small
float Ki = 0.1;      //0.1
float Kd = 0.1;        //1

      //set this equal to LowerTemperatureThreshold

// LED control variables
int led = 5;                //led pin
int ledBrightness = 0;      //led pwm control (0-255)
int fan = 6;                //fan pin


// Thermocycle control constants
const float UpperTemperatureThreshold = 95;
const float LowerTemperatureThreshold = 72;

float heatingHoldTime = 15000;   // Time to hold heating (15s)
float coolingHoldTime = 60000;   // Time to hold cooling (1 min)

unsigned long previousTime = 0;

float setpoint = 0;
float input = 0;   // Read temperature sensor value
float output = 0;
float error = 0;
float iTerm = 0;
float dInput = 0;
float lastInput = 0;
float cycleStartTime = 0;
bool isHeatingPhase = true;  // To track the heating or cooling phase

int cycleCount = 1;
const int TotalCycles = 1;


float tempPhase1 = 95.0; // Target temperature for phase 1
float tempPhase2 = 56.2; // Target temperature for phase 2
float tempPhase3 = 72.0; // Target temperature for phase 3

unsigned long holdTimePhase1 = 15000; // Hold time for phase 1 in milliseconds (15 seconds)
unsigned long holdTimePhase2 = 30000; // Hold time for phase 2 in millisecondsß (30 seconds)
unsigned long holdTimePhase3 = 30000; // Hold time for phase 3 in milliseconds (30 seconds)

int phase = 1;  // Start with phase 1

unsigned long initialDenaturationTime = 120000; // 2 minutes in milliseconds
unsigned long finalExtensionTime = 300000;      // 5 minutes in milliseconds
unsigned long phaseStartTime = 0;