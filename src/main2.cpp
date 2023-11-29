#include <Arduino.h>
#include "AD7793.h"
#include "Communication.h"
#include "SPI.h"
#include <math.h>

// Temperature sensor variables and constants
float  Vref; /* The external reference voltage applied between pins REFIN(+) and REFIN(-)and  resulting from the excitation current flowing through the reference resistor  */
float  GAIN; /* Gain of the AD7793 unternal instrumentation amplifier */
float RREF = 402.0; /* The reference resistor: here, 402 ohm or 4.02  Kohm, 0.1%, 10ppm/C */
float RRTD; /* The measured resistance of the RTD */ 
float temp_PT100; /* The temperature read on the analog input channel 1 */
float R0 = 100.0;  /* RTD resistance at 0C */
float Voltage_therm;
float tempfinal;
// PID parameters
float Kp = 5.0;     // Proportional gain
float Ki = 1;     // Integral gain
float Kd = 1.0;     // Derivative gain

float setpoint = 33.0;  // Desired temperature
float input, output, lastInput;
float iTerm, dInput;

// LED control variables
int led = 6;
int ledBrightness = 0;

// Function to read temperature from the sensor
float Get_Thermocouple(){

  uint32_t conv_therm;
  int32_t DataT1;
  float Voltage;
  
  AD7793_Reset(); /* Sends 32 consecutive 1's on SPI in order to reset  the part */  
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

float Get_PT100(){

  uint32_t conv_PT100;
  float temp;
  
  AD7793_Reset(); /* Sends 32 consecutive 1's on SPI in order to reset  the part */  
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
    lastInput = Get_PT100() + Get_Thermocouple();  // Initialize lastInput with the current temperature
}

void loop() {
    // Read current temperature
    temp_PT100 = Get_PT100();
    Voltage_therm = Get_Thermocouple();
    input = temp_PT100 + Voltage_therm;

    // Dynamic setpoint adjustment
    if (input >= 33.0) {
        setpoint = 28.0;  // Change setpoint to 28°C once temperature reaches or exceeds 33°C
    } else if (input <= 28.0) {
        setpoint = 33.0;  // Change setpoint to 33°C once temperature drops to or below 28°C
    }

    // Calculate PID
    // float error = setpoint - input;
    // iTerm += (Ki * error);
    // iTerm = constrain(iTerm, 0, 255);  // Constrain iTerm to prevent integral windup

    // dInput = (input - lastInput);

    // // Compute PID output
    // output = Kp * error + iTerm - Kd * dInput;
    // // output = constrain(output, 0, 255);  // Constrain the output to be between 0 and 255

    // // Convert PID output to LED brightness
    // ledBrightness = constrain((int)output,0,255);

    // Update LED brightness
    analogWrite(led, 255);

    // Debugging prints
    Serial.print("Temperature: "); Serial.println(input);
    // Serial.print("Setpoint: "); Serial.println(setpoint);
    // Serial.print("PID Output: "); Serial.println(output);
    // Serial.print("LED Brightness: "); Serial.println(ledBrightness);

    // Add a delay for stability
    delay(1);  // Delay of 1 millisecond
}
