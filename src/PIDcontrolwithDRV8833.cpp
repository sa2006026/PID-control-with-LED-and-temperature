#include <Arduino.h>
#include "AD7793.h"
#include "Communication.h"
#include "SPI.h"
#include <math.h>

// Temperature sensor variables and constants
float Vref;
float GAIN;
float RREF = 402.0;
float RRTD;
float temp_PT100;
float R0 = 100.0;
float Voltage_therm;
float tempfinal;
float Rawdata;
// PID parameters
float Kp = 5.0;     // Proportional gain
float Ki = 1;       // Integral gain
float Kd = 1.0;     // Derivative gain

float setpoint = 95; // Initial setpoint, halfway for demonstration
float input, output;
float iTerm, lastInput = 0;
float dInput, error;

// LED control variables
int led = 5;
int led2 = 3;
int ledBrightness = 0;

int fan = 3;
int fan2 = 10;

// Thermocycle control constants
const float UpperTemperatureThreshold = 95;
const float LowerTemperatureThreshold = 60;


int cycleCount = 0;
const int TotalCycles = 30; // Target cycle count

unsigned long previousMillis = 0;
const long interval = 500;


// Function to read temperature from the sensor
float Get_Thermocouple_init(){
  uint32_t conv_therm;
  int32_t DataT1;
  float Voltage;
  
  AD7793_Reset(); /* Sends 32 consecutive on SPI in order to reset  the part */  
  // delay(100);     
  AD7793_SetChannel(AD7793_CH_AIN1P_AIN1M); /* Selects channel  2 of AD7793 */
  AD7793_SetGain(AD7793_GAIN_32); /* Sets the gain to 1 */
  AD7793_SetIntReference(AD7793_REFSEL_INT); /* Sets the reference source for  the ADC.*/
  AD7793_DisableBipolar();/* Sets to the bipolar mode*/
  // Bipolar mode + Internal reference voltage + Gain 1 --> input analog voltage range: -1.17 ~ +1.17 V, corresponding to value range: 0x000000 ~ 0xffffff

  float Vref = 1.17;  /* This is the internal reference voltage provided by the AD7793, expressed in volt  */
  GAIN = 32.0; 

  /* As the gain of the internal  instrumentation amplifier has been changed, Analog Devices recommends performing  a calibration  */
  AD7793_Calibrate(AD7793_MODE_CAL_INT_ZERO, AD7793_CH_AIN1P_AIN1M);  /* Performs Internal Zero calibration to the specified channel. */
  AD7793_Calibrate(AD7793_MODE_CAL_INT_FULL,  AD7793_CH_AIN1P_AIN1M); /* Performs Internal Full Calibration to the specified channel.  */

  //AD7793_SetExcitDirection(AD7793_DIR_IEXC1_IEXC2_IOUT1);  /*Sets the direction of the internal excitation current source */

  AD7793_SetMode(AD7793_MODE_CONT);  /* Continuous Conversion Mode */
  AD7793_SetClockSource(AD7793_CLK_INT_CO); /* Internal 64 kHz Clk available at the CLK pin */
  AD7793_SetFilterUpdateRate(AD7793_MODE_RATE(0xB));/* Filter update rate 12.5 Hz */
  //AD7793_SetExcitCurrent(AD7793_EN_IXCEN_210uA);/*  Sets the current of the AD7793 internal excitation current source */
  delay(200);

  conv_therm = AD7793_ContinuousSingleRead();
  DataT1 = (conv_therm - 0x800000);
  Voltage = DataT1 * 2 * Vref / 0xFFFFFF;
  Voltage *= 24937.65586;
  Voltage /= 32;
  return Voltage;
}

float Get_RawData(){
  uint32_t conv_therm;
  int32_t DataT1;
  float Voltage;
  
  AD7793_Reset(); /* Sends 32 consecutive on SPI in order to reset  the part */  
  // delay(100);     
  AD7793_SetChannel(AD7793_CH_AIN1P_AIN1M); /* Selects channel  2 of AD7793 */
  AD7793_SetGain(AD7793_GAIN_32); /* Sets the gain to 1 */
  AD7793_SetIntReference(AD7793_REFSEL_INT); /* Sets the reference source for  the ADC.*/
  AD7793_DisableBipolar();/* Sets to the bipolar mode*/
  // Bipolar mode + Internal reference voltage + Gain 1 --> input analog voltage range: -1.17 ~ +1.17 V, corresponding to value range: 0x000000 ~ 0xffffff

  float Vref = 1.17;  /* This is the internal reference voltage provided by the AD7793, expressed in volt  */
  GAIN = 32.0; 

  /* As the gain of the internal  instrumentation amplifier has been changed, Analog Devices recommends performing  a calibration  */
  AD7793_Calibrate(AD7793_MODE_CAL_INT_ZERO, AD7793_CH_AIN1P_AIN1M);  /* Performs Internal Zero calibration to the specified channel. */
  AD7793_Calibrate(AD7793_MODE_CAL_INT_FULL,  AD7793_CH_AIN1P_AIN1M); /* Performs Internal Full Calibration to the specified channel.  */

  //AD7793_SetExcitDirection(AD7793_DIR_IEXC1_IEXC2_IOUT1);  /*Sets the direction of the internal excitation current source */

  AD7793_SetMode(AD7793_MODE_CONT);  /* Continuous Conversion Mode */
  AD7793_SetClockSource(AD7793_CLK_INT_CO); /* Internal 64 kHz Clk available at the CLK pin */
  AD7793_SetFilterUpdateRate(AD7793_MODE_RATE(0xB));/* Filter update rate 12.5 Hz */
  //AD7793_SetExcitCurrent(AD7793_EN_IXCEN_210uA);/*  Sets the current of the AD7793 internal excitation current source */
  delay(200);

  conv_therm = AD7793_ContinuousSingleRead();
  DataT1 = (conv_therm - 0x800000);
  Voltage = DataT1 * 2 * Vref / 0xFFFFFF;
  Voltage *= 24937.65586;
  Voltage /= 32;
  return conv_therm;
}

float Get_PT100_init(){

  uint32_t conv_PT100;
  float temp;
  
  AD7793_Reset(); /* Sends 32 consecutive on SPI in order to reset  the part */  
  // delay(100);     
  AD7793_SetChannel(AD7793_CH_AIN2P_AIN2M); /* Selects channel  2 of AD7793 */
  AD7793_SetGain(AD7793_GAIN_1); /* Sets the gain to 1 */
  AD7793_SetIntReference(AD7793_REFSEL_EXT); /* Sets the reference source for  the ADC.*/
  AD7793_DisableBipolar();/* Sets to the bipolar mode*/
  // Bipolar mode + Internal reference voltage + Gain 1 --> input analog voltage range: -1.17 ~ +1.17 V, corresponding to value range: 0x000000 ~ 0xffffff

  Vref = 1.17;  /* This is the internal reference voltage provided by the AD7793, expressed in volt  */
  GAIN = 1.0; 

  /* As the gain of the internal  instrumentation amplifier has been changed, Analog Devices recommends performing  a calibration  */
  AD7793_Calibrate(AD7793_MODE_CAL_INT_ZERO, AD7793_CH_AIN2P_AIN2M);  /* Performs Internal Zero calibration to the specified channel. */
  AD7793_Calibrate(AD7793_MODE_CAL_INT_FULL,  AD7793_CH_AIN2P_AIN2M); /* Performs Internal Full Calibration to the specified channel.  */

  //AD7793_SetIntReference(AD7793_REFSEL_INT); /* Sets the voltage reference source for the ADC */
  AD7793_SetExcitDirection(AD7793_DIR_IEXC1_IEXC2_IOUT1);  /*Sets the direction of the internal excitation current source */

  AD7793_SetMode(AD7793_MODE_CONT);  /* Continuous Conversion Mode */
  AD7793_SetClockSource(AD7793_CLK_INT_CO); /* Internal 64 kHz Clk available at the CLK pin */
  AD7793_SetFilterUpdateRate(AD7793_MODE_RATE(0xB));/* Filter update rate 12.5 Hz */
  AD7793_SetExcitCurrent(AD7793_EN_IXCEN_210uA);/*  Sets the current of the AD7793 internal excitation current source */
  delay(200);

  conv_PT100 = AD7793_ContinuousSingleRead();
  conv_PT100 = conv_PT100 - 0x800000;
  //Serial.print(conv);
  //Serial.print("\n");
  RRTD = RREF * conv_PT100 / 0x7fffff; /* Computes the RTD resistance from the conversion  code */
  //Serial.print(RRTD);
  //Serial.print("\n");
  temp = (RRTD-100)/0.400574;    /* Callender-Van  Dusen equation temp > 0C */
  //Serial.println(temp);
  //delay(1000);
  return temp;
}

float Get_Thermocouple(){
  uint32_t conv_PT100;
  float temp;
  
  Vref = 1.17;  /* This is the internal reference voltage provided by the AD7793, expressed in volt  */
  GAIN = 1.0; 

  conv_PT100 = AD7793_ContinuousSingleRead();
  conv_PT100 = conv_PT100 - 0x800000;
  //Serial.print(conv);
  //Serial.print("\n");
  RRTD = RREF * conv_PT100 / 0x7fffff; /* Computes the RTD resistance from the conversion  code */
  //Serial.print(RRTD);
  //Serial.print("\n");
  temp = (RRTD-100)/0.385055;    /* Callender-Van  Dusen equation temp > 0C */
  //Serial.println(temp);
  //delay(1000);
  return temp;
}

void setup() {
    Serial.begin(9600);
    pinMode(led, OUTPUT);
    // Initialize temperature sensor
    AD7793_Init();

    temp_PT100 = Get_PT100_init();
    Voltage_therm = Get_Thermocouple_init();

    Serial.print("PT100: "); Serial.println(temp_PT100);

    // lastInput = Get_PT100_init() + Get_Thermocouple_init();
}

void loop() {
    //Read current temperature
      // temp_PT100 = Get_PT100();
      // Voltage_therm = Get_Thermocouple_init();
      unsigned long currentTime = millis();
      int32_t DataT1;
      float Voltage;
      float Vref = 1.17;

      
      Rawdata = Get_RawData();
      DataT1 = (Rawdata - 0x800000);
      Voltage = DataT1 * 2 * Vref / 0xFFFFFF;
      Voltage *= 27910.611650905546;
      Voltage /= 32;
      input = Voltage + temp_PT100;
      
      // input =  temp_PT100 + Voltage_therm; //get temperature
      // input = Rawdata;
      
    // temp_PT100 = Get_PT100();
    // Voltage_therm = Get_Thermocouple();
    //input = temp_PT100 + Voltage_therm;

    // Thermocycle logic
    // if (increasingTemperature) {
    //     if (input >= UpperTemperatureThreshold) {
    //         increasingTemperature = false;
    //         setpoint = LowerTemperatureThreshold;
    //     } else {
    //         setpoint = UpperTemperatureThreshold;
    //     }
    // } else {
    //     if (input <= LowerTemperatureThreshold) {
    //         increasingTemperature = true;
    //         setpoint = UpperTemperatureThreshold;
    //     } else {
    //         setpoint = LowerTemperatureThreshold;
    //     }
    // }

    // // Calculate PID
    // float error = setpoint - input;
    // iTerm += (Ki * error);
    // iTerm = constrain(iTerm, 0, 255);

    // dInput = (input - lastInput);

    // // Compute PID output
    // output = Kp * error + iTerm - Kd * dInput;

    // // Adjust LED brightness based on PID output
    // if (increasingTemperature) {
    //     ledBrightness = constrain((int)output, 0, 255);
    // } else {
    //     ledBrightness = 255 - constrain((int)output, 0, 255);
    // }
    
    if (cycleCount < TotalCycles * 2 -1) {
        if (setpoint == UpperTemperatureThreshold && input >= UpperTemperatureThreshold - 1) {
            setpoint = LowerTemperatureThreshold;
            analogWrite(led2, 255);
            cycleCount++;
        } else if (setpoint == LowerTemperatureThreshold && input <= LowerTemperatureThreshold + 1) {
            setpoint = UpperTemperatureThreshold;
            analogWrite(led2, 0);
            cycleCount++;
        }
    }

    // Calculate PID
    error = setpoint - input;
    iTerm += (Ki * error);
    // Prevent integral windup
    iTerm = constrain(iTerm, 0, 255);
    dInput = (input - lastInput);

    // Compute PID Output
    output = Kp * error + iTerm - Kd * dInput;
    ledBrightness = constrain((int)output, 0, 255);
    analogWrite(led, ledBrightness);
    
    // analogWrite(led, 255); //Test Focus
    // analogWrite(led2, ledBrightness);

    // Debugging prints
    Serial.print("Time: ");
    Serial.print(currentTime);
    Serial.print(" Temperature: "); Serial.print(input);
    Serial.print(" PIDoutput "); Serial.print(ledBrightness);
    Serial.print(" cycle "); Serial.println(cycleCount);

    // Serial.print("Base: "); Serial.println(temp_PT100);
    // Serial.print("Rawdata: "); Serial.println(Rawdata);

    // Serial.print("Setpoint: "); Serial.println(setpoint);
    // Serial.print("PID Output: "); Serial.println(output);
    // Serial.print("LED Brightness: "); Serial.println(ledBrightness);

    // Add a delay for stability
    delay(1);

    // Update lastInput for next iteration
    // lastInput = input;
}