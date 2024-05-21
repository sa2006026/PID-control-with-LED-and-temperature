#include "config.h"
#include "AD7793.h"
#include "Communication.h"
#include "SPI.h"
#include <math.h>

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

void handleThermocyclingAndPID() {
    unsigned long currentTime = millis();
    if (cycleCount < TotalCycles * 2 - 1) {
        if (setpoint == UpperTemperatureThreshold && input >= UpperTemperatureThreshold - 1) {
            setpoint = LowerTemperatureThreshold;
            analogWrite(fan, 255); // Turn fan on to cool down
            cycleCount++;
        } else if (setpoint == LowerTemperatureThreshold && input <= LowerTemperatureThreshold + 1) {
            setpoint = UpperTemperatureThreshold;
            analogWrite(fan, 0); // Turn fan off to heat up
            cycleCount++;
        }
        // Calculate PID
        error = setpoint - input;
        iTerm += (Ki * error);
        // Prevent integral windup
        iTerm = constrain(iTerm, 0, 255);
        dInput = (input - lastInput);
        lastInput = input; // Update last input for next iteration

        // Compute PID Output
        output = Kp * error + iTerm - Kd * dInput;
        ledBrightness = constrain(static_cast<int>(output), 0, 255);
        analogWrite(led, ledBrightness);

        // Debugging prints
        Serial.print("Time: "); Serial.print(currentTime);
        Serial.print(" Temperature: "); Serial.print(input);
        Serial.print(" PIDoutput "); Serial.print(ledBrightness);
        Serial.print(" cycle "); Serial.println(cycleCount);

        // Minimal delay for stability
        delay(1);
    }
}

void handleThermocycling() {
    unsigned long currentTime = millis();
    if (cycleCount < TotalCycles * 2 - 1) {
        if (setpoint == UpperTemperatureThreshold && input >= UpperTemperatureThreshold - 1) {
            setpoint = LowerTemperatureThreshold;
            analogWrite(fan, 255); // Turn fan on to cool down
            analogWrite(led,0);
            cycleCount++;
        } else if (setpoint == LowerTemperatureThreshold && input <= LowerTemperatureThreshold + 1) {
            setpoint = UpperTemperatureThreshold;
            analogWrite(fan, 0); // Turn fan off to heat up
            analogWrite(led,255);
            cycleCount++;
        }

        // Debugging prints
        Serial.print("Time: "); Serial.print(currentTime);
        Serial.print(" Temperature: "); Serial.print(input);
        Serial.print(" PIDoutput "); Serial.print(ledBrightness);
        Serial.print(" cycle "); Serial.println(cycleCount);

        // Minimal delay for stability
        delay(1);
    }
}

void TuneFocus() {
    analogWrite(led, 20);
}


void ReadTemperature() {
    // Fetch raw data from the sensor
    Rawdata = Get_RawData();
    
    // Convert raw data to a signed integer
    DataT1 = (Rawdata - 0x800000);

    // Calculate voltage based on reference voltage and gain
    Voltage = DataT1 * 2 * Vref / 0xFFFFFF;
    Voltage *= 27910.611650905546;
    Voltage /= 32;

    // Update the global 'input' variable by adding the PT100 temperature
    input = Voltage + temp_PT100;
}

void ThermocoupleSetup(){
    AD7793_Init();
    temp_PT100 = Get_PT100_init();
    Voltage_therm = Get_Thermocouple_init();
}

void setup() {
    Serial.begin(9600);
    pinMode(led, OUTPUT);
    ThermocoupleSetup();
}

void loop() {

    ReadTemperature();              //Using thermocouple to read temperature
    //TuneFocus();                  //Use small power of led for focus
    handleThermocyclingAndPID();   //Go through 30 thermocycles and do PID
    //handleThermocycling();        //Go through 30 thermocycles without PID
}