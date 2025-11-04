#include "AD7793.h"
#include "Communication.h"
#include "SPI.h"
#include <math.h>

int i = 0;

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
float Kp = 50.0;  
float Ki = 0.1;  
float Kd = 0.1;  

// LED and Fan control
int led = 5;  
int ledBrightness = 0;  
int fan = 6;  

// Thermocycle control constants
unsigned long previousTime = 0;
float setpoint = 60;   
float input = 0;   
float output = 0;
float error = 0;
float iTerm = 0;
float dInput = 0;
float lastInput = 0;
float cycleStartTime = 0;

int cycleCount = 1;
int TotalCycles = 30;
int phase = 1;  
unsigned long phaseStartTime = 0;

// Declare arrays for temperature and hold times
float  tempPhase2, tempPhase3, tempPhase4, tempPhase5;
int  holdTimePhase2, holdTimePhase3, holdTimePhase4, holdTimePhase5;

float  tempPhase1 = 95;
float  holdTimePhase1 = 240000;

const float UpperTemperatureThreshold = 95;
const float LowerTemperatureThreshold = 60;

bool receivedData = false;  // Flag to track data reception

enum ThermoCyclingPhase {
    INITIAL_DENATURATION,
    CYCLING,
    FINAL_EXTENSION,
    COMPLETE,
    STOPPED
};

ThermoCyclingPhase currentPhase = INITIAL_DENATURATION;

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



/** Thermocycling control */
void handleFiveStageThermocycling() {
    float currentTime = millis();


    switch (currentPhase) {
        case INITIAL_DENATURATION:
            setpoint = tempPhase1-1;  // Initial denaturation temperature (95°C)
            if (input >= setpoint-1) {
                if (cycleStartTime == 0) {
                    cycleStartTime = currentTime;  // Start the timer
                }
                if (currentTime - cycleStartTime >= holdTimePhase1) {
                    // Move to the cycling phase
                    currentPhase = CYCLING;
                    cycleStartTime = 0;  // Reset for cycling phase
                    phaseStartTime = 0;  // Reset phase timer
                    Serial.println("Initial Denaturation Complete. Starting Cycling...");
                }
            }
            break;

        case CYCLING:
            if (cycleCount <= TotalCycles) {
                // Phase 1: Heat to 95°C and hold for 15 seconds
                if (phase == 1) {
                    setpoint = tempPhase2;

                    if (input >= tempPhase2 - 2) {  // When close enough to the setpoint
                        if (cycleStartTime == 0) {
                            cycleStartTime = currentTime/1000;  // Start the timer when reaching the setpoint
                        }

                        if (currentTime/1000 - cycleStartTime >= holdTimePhase2) {
                            phase = 2;  // Move to the next phase
                            cycleStartTime = 0;  // Reset the timer for the next phase
                            analogWrite(fan, 255);  // Turn on fan for cooling
                        }
                    }
                }

                // Phase 2: Cool to 55°C and hold for 30 seconds
                else if (phase == 2) {
                    setpoint = tempPhase3;

                    if (input <= tempPhase3 + 2) {  // When close enough to the setpoint
                        if (cycleStartTime == 0) {
                            cycleStartTime = currentTime/1000;  // Start the timer when reaching the setpoint
                            
                        }

                        if (currentTime/1000 - cycleStartTime >= holdTimePhase2) {
                            phase = 3;  // Move to the next phase
                            cycleStartTime = 0;  // Reset the timer for the next phase
                            analogWrite(fan, 0);  // Turn off fan for heating
                        }
                    }
                }

                // Phase 3: Heat to 72°C and hold for 30 seconds
                else if (phase == 3) {
                    setpoint = tempPhase4;

                    if (input >= tempPhase4 - 2) {  // When close enough to the setpoint
                        if (cycleStartTime == 0) {
                            cycleStartTime = currentTime/1000;  // Start the timer when reaching the setpoint
                        
                        }

                        if (currentTime/1000 - cycleStartTime >= holdTimePhase3) {
                            phase = 1;  // Move to the end of the cycle
                            cycleStartTime = 0;  // Reset the timer
                            analogWrite(fan, 0);  // Ensure the fan is off
                            cycleCount+=1;
                        }
                    }
                }
            } else {
                // Move to the final extension phase
                currentPhase = FINAL_EXTENSION;
                cycleStartTime = 0;
                Serial.println("Cycling Complete. Starting Final Extension...");
            }
            break;

        case FINAL_EXTENSION:
            setpoint = tempPhase5;  // Final extension temperature (72°C)
            if (input >= tempPhase5-1) {
                if (cycleStartTime == 0) {
                    cycleStartTime = currentTime/1000;  // Start the timer
                }
                if (currentTime/1000 - cycleStartTime >= holdTimePhase5) {
                    // Move to the complete phase
                    currentPhase = STOPPED;
                    Serial.println("Final Extension Complete. Thermocycling finished.");
                }
            }
            
        case STOPPED:
            analogWrite(led, 0);
            analogWrite(fan, 0);
            Serial.println("System Stopped. All outputs are off.");
            return;
    }
        

    // PID Calculations (same as before)
    error = setpoint - input;
    iTerm += (Ki * error);
    iTerm = constrain(iTerm, 0, 255);
    dInput = (input - lastInput);
    lastInput = input;

    output = Kp * error + iTerm - Kd * dInput;
    int ledBrightness = constrain(static_cast<int>(output), 0, 255);
    analogWrite(led, ledBrightness);

    Serial.print("Time: "); Serial.print(currentTime);
    Serial.print(" Temperature: "); Serial.print(input);
    Serial.print(" PID Output: "); Serial.print(ledBrightness);
    Serial.print(" Time Count: "); Serial.print((currentTime - cycleStartTime));
    Serial.print(" Setpoint: "); Serial.print(setpoint);
    Serial.print(" Cycle: "); Serial.print(cycleCount);
    Serial.print(" Phase: "); Serial.println(phase);


    // Minimal delay for stability
    delay(1);
}






void ReadTemperature() {
    // Fetch raw data from the sensor
    Rawdata = Get_RawData();
    // Convert raw data to a signed integer
    DataT1 = (Rawdata - 0x800000);

    // Calculate voltage based on reference voltage and gain
    Voltage = DataT1 * 2 * Vref / 0xFFFFFF;
    Voltage *= 27910.611650905546;
    Voltage /= 32; //x*872.2+22.69

    // Update the global 'input' variable by adding the PT100 temperature
    input = Voltage + temp_PT100; //temp_PT100: 22.69C   fitted equation: y=1.1561x−3.6771
    input = (input + 3.6771)/1.1561;
    input = input +1.5;
    //Serial.println(input);
}

/**
 * Performs initialization of the ADC used for thermocouple and PT100 sensors, including calibration and sensor selection.
 */
void ThermocoupleSetup(){
    AD7793_Init();
    temp_PT100 = Get_PT100_init(); 
    Voltage_therm = Get_Thermocouple_init();
}

void handleSerialCommands() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        Serial.println("Received command: " + command); // Debugging output

        if (command.startsWith("C")) {
            // Handle cycle number
            int cycleNumber = command.substring(1).toInt();
            Serial.print("Cycle Number: ");
            Serial.println(cycleNumber); // Debugging output
            TotalCycles = cycleNumber;
        } 
        else if (command.startsWith("F")) {
            // Handle temperature and hold time
            int phaseIndex = 0;
            int startIndex = 1; // Start after 'F'
            while (startIndex < command.length()) {
                int endIndex = command.indexOf(';', startIndex);
                if (endIndex == -1) {
                    endIndex = command.length(); // Last command
                }
                String phaseData = command.substring(startIndex, endIndex);
                int commaIndex = phaseData.indexOf(',');
                if (commaIndex != -1) {
                    float temp = phaseData.substring(0, commaIndex).toFloat();
                    int holdTime = phaseData.substring(commaIndex + 1).toInt();
                    // Debugging output
                    Serial.print("Phase "); Serial.print(phaseIndex + 1); 
                    Serial.print(" - Temp: "); Serial.print(temp); 
                    Serial.print(", Hold Time: "); Serial.println(holdTime);
                    // Update the corresponding phase variables
                    switch (phaseIndex) {
                        case 0: tempPhase1 = temp; holdTimePhase1 = holdTime; break;
                        case 1: tempPhase2 = temp; holdTimePhase2 = holdTime*1000; break;
                        case 2: tempPhase3 = temp; holdTimePhase3 = holdTime*1000; break;
                        case 3: tempPhase4 = temp; holdTimePhase4 = holdTime*1000; break;
                        case 4: tempPhase5 = temp; holdTimePhase5 = holdTime*1000; break;
                    }
                    phaseIndex++;
                }
                startIndex = endIndex + 1; // Move to the next command
            }
        } 
        
        else if (command == "START") {
            currentPhase = INITIAL_DENATURATION; // Restart the process
            cycleCount = 0; // Reset cycle count
            i = 1;
            Serial.println("RESTARTING PROGRAM...");
            // Start the thermocycling logic here
        } 
        else if (command == "STOP") {
            Serial.println("EXITING PROGRAM...");
            i = 0;
            analogWrite(led, 0);
            analogWrite(fan, 0);
        }
    }
}

void handleThermocyclingAndPID() {                           
    unsigned long currentTime = millis();
    if (cycleCount <= TotalCycles) {
        if (setpoint == UpperTemperatureThreshold && input >= UpperTemperatureThreshold - 1) {
            setpoint = LowerTemperatureThreshold;
            analogWrite(fan, 255); // Turn fan on to cool down
            cycleCount += 1;
        } else if (setpoint == LowerTemperatureThreshold && input <= LowerTemperatureThreshold + 1) {
            setpoint = UpperTemperatureThreshold;
            analogWrite(fan, 0); // Turn fan off to heat up
            
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

void TuneFocus() {
    analogWrite(led, 25);
    Serial.println(input);
}

void handleThermocycling() {
    unsigned long currentTime = millis();
    if (cycleCount <= TotalCycles) {
        if (setpoint == UpperTemperatureThreshold && input >= UpperTemperatureThreshold - 1) {
            setpoint = LowerTemperatureThreshold;
            analogWrite(fan, 255); // Turn fan on to cool down
            analogWrite(led,0);
            cycleCount++;
        } else if (setpoint == LowerTemperatureThreshold && input <= LowerTemperatureThreshold + 1) {
            setpoint = UpperTemperatureThreshold;
            analogWrite(fan, 0); // Turn fan off to heat up
            analogWrite(led,255);
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

void setup() {
    Serial.begin(9600);
    pinMode(led, OUTPUT);
    pinMode(fan, OUTPUT);
    ThermocoupleSetup();

}

void loop() {
        ReadTemperature();   //cuvette sample height 31mm
        //TuneFocus();  
        //handleSerialCommands();
        
        //if (i == 1){
        //handleThermocyclingAndPID();
        //handleThermocycling();
            handleFiveStageThermocycling();
        //}
    }
    
    // Add your thermocycling logic here



