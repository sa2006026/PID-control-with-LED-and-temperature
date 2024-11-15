/*
 * Thermocouple and PT100 Temperature Measurement
 * Description:
 *  This program interfaces with the AD7793 ADC to read temperatures from a thermocouple and a PT100 sensor. It includes functionalities
 *  for initializing the device, reading data, and handling temperature control cycles with PID regulation.
 *
 * Dependencies:
 *  - AD7793.h: Manages interactions with the AD7793 ADC.
 *  - SPI.h: Required for SPI communication.
 *  - config.h: Contains configuration constants and settings.
 *  - Communication.h: Handles communication functionalities.
 *  - math.h: Provides access to mathematical functions for data processing.
 *
 * Hardware:
 *  - Any compatible Arduino board connected to AD7793 and the appropriate temperature sensors.
 *
 * Date: [30/5/2024]
 */


#include "config.h"
#include "AD7793.h"
#include "Communication.h"
#include "SPI.h"
#include <math.h>

enum ThermoCyclingPhase {
    INITIAL_DENATURATION,
    CYCLING,
    FINAL_EXTENSION,
    COMPLETE,
    STOPPED
};

ThermoCyclingPhase currentPhase = INITIAL_DENATURATION;

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

/**
 * Manages thermocycling process and applies PID control to maintain temperature within defined thresholds.
 * This function controls a heating and cooling cycle based on set temperature thresholds and uses PID control 
 * to maintain temperature stability.
 */
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

void handleBenchtopThermocycling() {
    unsigned long currentTime = millis();

    if (cycleCount <= TotalCycles) {
        // Check if we're in the heating phase
        if (isHeatingPhase) {
            setpoint = UpperTemperatureThreshold-1;

            // Check if the temperature is at or above the threshold
            if (input >= UpperTemperatureThreshold - 1) {
                // Start the holding period only when reaching the setpoint for the first time
                if (cycleStartTime == 0) {
                    cycleStartTime = currentTime;  // Start the timer only once
                }

                // Hold for the heating duration
                if (currentTime - cycleStartTime >= heatingHoldTime) {
                    // Switch to cooling phase
                    isHeatingPhase = false;
                    cycleStartTime = 0;  // Reset the timer for the cooling phase
                    analogWrite(fan, 255);  // Turn on the fan to start cooling
                }
            }
        } 
        // Cooling phase
        else {
            setpoint = LowerTemperatureThreshold+0.5;

            // Check if the temperature is at or below the threshold
            if (input <= LowerTemperatureThreshold + 1) {
                // Start the holding period only when reaching the setpoint for the first time
                if (cycleStartTime == 0) {
                    cycleStartTime = currentTime;  // Start the timer only once
                }

                // Hold for the cooling duration
                if (currentTime - cycleStartTime >= coolingHoldTime) {
                    // Switch back to heating phase
                    isHeatingPhase = true;
                    cycleStartTime = 0;  // Reset the timer for the heating phase
                    cycleCount += 1;     // Increment the cycle count
                    analogWrite(fan, 0);  // Turn off the fan to start heating
                }
            }
        }

        // PID Calculations
        error = setpoint - input;
        iTerm += (Ki * error);

        // Prevent integral windup
        iTerm = constrain(iTerm, 0, 255);
        dInput = (input - lastInput);
        lastInput = input; // Update last input for next iteration

        // Compute PID Output
        output = Kp * error + iTerm - Kd * dInput;
        int ledBrightness = constrain(static_cast<int>(output), 0, 255);
        analogWrite(led, ledBrightness);

        // Debugging prints
        Serial.print("Time: "); Serial.print(currentTime);
        Serial.print(" Temperature: "); Serial.print(input);
        Serial.print(" PID Output: "); Serial.print(ledBrightness);
        Serial.print(" Time Since Reaching Setpoint: "); Serial.print(currentTime - cycleStartTime);
        Serial.print(" Cycle: "); Serial.println(cycleCount);

    } else {
        // End of cycles, stop the thermocycling process
        Serial.println("Thermocycling complete!");
        analogWrite(fan, 0); // Ensure the fan is turned off
        analogWrite(led, 0);
    }

    // Minimal delay for stability
    delay(1);
}

void handle3stageBenchtopThermocycling() {
    unsigned long currentTime = millis();

    if (cycleCount <= TotalCycles) {

        // Phase 1: Heat to 95°C and hold for 15 seconds
        if (phase == 1) {
            setpoint = tempPhase1 -1;

            if (input >= tempPhase1 - 1) {  // When close enough to the setpoint
                if (cycleStartTime == 0) {
                    cycleStartTime = currentTime;  // Start the timer when reaching the setpoint
                }

                if (currentTime - cycleStartTime >= holdTimePhase1) {
                    phase = 2;  // Move to the next phase
                    cycleStartTime = 0;  // Reset the timer for the next phase
                    analogWrite(fan, 255);  // Turn on fan for cooling
                }
            }
        }

        // Phase 2: Cool to 55°C and hold for 30 seconds
        else if (phase == 2) {
            setpoint = tempPhase2+0.5;

            if (input <= tempPhase2 + 1) {  // When close enough to the setpoint
                if (cycleStartTime == 0) {
                    cycleStartTime = currentTime;  // Start the timer when reaching the setpoint
                }

                if (currentTime - cycleStartTime >= holdTimePhase2) {
                    phase = 3;  // Move to the next phase
                    cycleStartTime = 0;  // Reset the timer for the next phase
                    analogWrite(fan, 0);  // Turn off fan for heating
                }
            }
        }

        // Phase 3: Heat to 72°C and hold for 30 seconds
        else if (phase == 3) {
            setpoint = tempPhase3;

            if (input >= tempPhase3 - 1) {  // When close enough to the setpoint
                if (cycleStartTime == 0) {
                    cycleStartTime = currentTime;  // Start the timer when reaching the setpoint
                }

                if (currentTime - cycleStartTime >= holdTimePhase3) {
                    phase = 1;  // Move to the end of the cycle
                    cycleStartTime = 0;  // Reset the timer
                    analogWrite(fan, 0);  // Ensure the fan is off
                    Serial.println("Thermocycling complete!");
                    cycleCount+=1;
                }
            }
        }

        // PID Calculations (unchanged)
        error = setpoint - input;
        iTerm += (Ki * error);

        // Prevent integral windup
        iTerm = constrain(iTerm, 0, 255);
        dInput = (input - lastInput);
        lastInput = input;  // Update last input for next iteration

        // Compute PID Output
        output = Kp * error + iTerm - Kd * dInput;
        int ledBrightness = constrain(static_cast<int>(output), 0, 255);
        analogWrite(led, ledBrightness);

        // Debugging prints
        Serial.print("Time: "); Serial.print(currentTime);
        Serial.print(" Temperature: "); Serial.print(input);
        Serial.print(" PID Output: "); Serial.print(ledBrightness);
        Serial.print(" Time Since Reaching Setpoint: "); Serial.print(currentTime - cycleStartTime);
        Serial.print(" Phase: "); Serial.println(cycleCount);

        
    }else {
        // End of cycles, stop the thermocycling process
        Serial.println("Thermocycling complete!");
        analogWrite(fan, 0); // Ensure the fan is turned off
        analogWrite(led, 0);
    }
    // Minimal delay for stability
        delay(1);
}

void handleFiveStageThermocycling() {
    unsigned long currentTime = millis();

    switch (currentPhase) {
        case INITIAL_DENATURATION:
            setpoint = tempPhase1 -1;  // Initial denaturation temperature (95°C)
            if (input >= setpoint) {
                if (cycleStartTime == 0) {
                    cycleStartTime = currentTime;  // Start the timer
                    setpoint = tempPhase1;
                }
                if (currentTime - cycleStartTime >= initialDenaturationTime) {
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
                    setpoint = tempPhase1 -1;

                    if (input >= tempPhase1 - 1) {  // When close enough to the setpoint
                        if (cycleStartTime == 0) {
                            cycleStartTime = currentTime;  // Start the timer when reaching the setpoint
                            setpoint = tempPhase1;
                        }

                        if (currentTime - cycleStartTime >= holdTimePhase1) {
                            phase = 2;  // Move to the next phase
                            cycleStartTime = 0;  // Reset the timer for the next phase
                            analogWrite(fan, 255);  // Turn on fan for cooling
                        }
                    }
                }

                // Phase 2: Cool to 55°C and hold for 30 seconds
                else if (phase == 2) {
                    setpoint = tempPhase2+0.5;

                    if (input <= tempPhase2 + 0.5) {  // When close enough to the setpoint
                        if (cycleStartTime == 0) {
                            cycleStartTime = currentTime;  // Start the timer when reaching the setpoint
                            setpoint = tempPhase2;
                        }

                        if (currentTime - cycleStartTime >= holdTimePhase2) {
                            phase = 3;  // Move to the next phase
                            cycleStartTime = 0;  // Reset the timer for the next phase
                            analogWrite(fan, 0);  // Turn off fan for heating
                        }
                    }
                }

                // Phase 3: Heat to 72°C and hold for 30 seconds
                else if (phase == 3) {
                    setpoint = tempPhase3 - 0.5;

                    if (input >= tempPhase3 - 0.5) {  // When close enough to the setpoint
                        if (cycleStartTime == 0) {
                            cycleStartTime = currentTime;  // Start the timer when reaching the setpoint
                            setpoint = tempPhase3;
                        }

                        if (currentTime - cycleStartTime >= holdTimePhase3) {
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
            setpoint = 72.0;  // Final extension temperature (72°C)
            if (input >= 72.0) {
                if (cycleStartTime == 0) {
                    cycleStartTime = currentTime;  // Start the timer
                }
                if (currentTime - cycleStartTime >= finalExtensionTime) {
                    // Move to the complete phase
                    currentPhase = COMPLETE;
                    Serial.println("Final Extension Complete. Thermocycling finished.");
                }
            }
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
    Serial.print(" Time Since Reaching Setpoint: "); Serial.print(currentTime - cycleStartTime);
    Serial.print(" Setpoint: "); Serial.print(setpoint);
    Serial.print(" Phase: "); Serial.println(cycleCount);

    // Minimal delay for stability
    delay(1);
}

void handleFiveStageThermocyclingWithHeatingRate() {
    unsigned long currentTime = millis();
    static unsigned long lastUpdateTime = 0; // For ramp rate control
    const float maxRampRate = 4.0; // Degrees per second

    // Calculate elapsed time since last update
    float elapsedTime = (currentTime - lastUpdateTime) / 1000.0; // Time in seconds

    switch (currentPhase) {
        case INITIAL_DENATURATION:
            float desiredTemperatureDenaturation = tempPhase1; // Target temperature for initial denaturation
            float currentSetpointDenaturation = setpoint; // Current setpoint for denaturation
            float temperatureChangeDenaturation = maxRampRate * elapsedTime; // Allowed change

            // Ramp up or down to the target temperature
            if (desiredTemperatureDenaturation > currentSetpointDenaturation) {
                setpoint = min(currentSetpointDenaturation + temperatureChangeDenaturation, desiredTemperatureDenaturation);
            } else {
                setpoint = max(currentSetpointDenaturation - temperatureChangeDenaturation, desiredTemperatureDenaturation);
            }

            //Serial.println(currentSetpointDenaturation);
            if (input >= tempPhase1) {
                if (cycleStartTime == 0) {
                    cycleStartTime = currentTime;  // Start the timer
                }
                if (currentTime - cycleStartTime >= initialDenaturationTime) {
                    // Move to the cycling phase
                    currentPhase = CYCLING;
                    cycleStartTime = 0;  // Reset for cycling phase
                    phaseStartTime = 0;  // Reset phase timer
                    Serial.println("Initial Denaturation Complete. Starting Cycling...");
                }
            }
            break;

        case CYCLING:
            Serial.println("Starting Cycling...");
            if (cycleCount <= TotalCycles) {
                // Phase 1: Heat to 95°C and hold for 15 seconds
                if (phase == 1) {
                    Serial.println("Starting Cycling...");
                    float desiredTemperature = tempPhase1; // Target for Phase 1
                    float currentSetpoint = setpoint; // Existing setpoint
                    float temperatureChange = maxRampRate * elapsedTime; // Allowed change

                    // Ramp up or down based on target temperature
                    if (desiredTemperature > currentSetpoint) {
                        setpoint = min(currentSetpoint + temperatureChange, desiredTemperature);
                    } else {
                        setpoint = max(currentSetpoint - temperatureChange, desiredTemperature);
                    }

                    // Proceed with PID control
                    if (input >=  tempPhase1- 1) {  // When close enough to the setpoint
                        if (cycleStartTime == 0) {
                            cycleStartTime = currentTime;  // Start the timer when reaching the setpoint
                        }

                        if (currentTime - cycleStartTime >= holdTimePhase1) {
                            phase = 2;  // Move to the next phase
                            cycleStartTime = 0;  // Reset the timer for the next phase
                            analogWrite(fan, 255);  // Turn on fan for cooling
                        }
                    }
                }

                // Phase 2: Cool to 55°C and hold for 30 seconds
                else if (phase == 2) {


                    if (input <= tempPhase2 + 0.5) {  // When close enough to the setpoint
                        if (cycleStartTime == 0) {
                            cycleStartTime = currentTime;  // Start the timer when reaching the setpoint
                        }

                        if (currentTime - cycleStartTime >= holdTimePhase2) {
                            phase = 3;  // Move to the next phase
                            cycleStartTime = 0;  // Reset the timer for the next phase
                            analogWrite(fan, 0);  // Turn off fan for heating
                        }
                    }
                }

                // Phase 3: Heat to 72°C and hold for 30 seconds
                else if (phase == 3) {
                    float desiredTemperature = tempPhase3; // Target for Phase 3
                    float currentSetpoint = setpoint;
                    float temperatureChange = maxRampRate * elapsedTime;

                    if (desiredTemperature > currentSetpoint) {
                        setpoint = min(currentSetpoint + temperatureChange, desiredTemperature);
                    } else {
                        setpoint = max(currentSetpoint - temperatureChange, desiredTemperature);
                    }

                    if (input >= tempPhase3 - 0.5) {  // When close enough to the setpoint
                        if (cycleStartTime == 0) {
                            cycleStartTime = currentTime;  // Start the timer when reaching the setpoint
                        }

                        if (currentTime - cycleStartTime >= holdTimePhase3) {
                            phase = 1;  // Move to the end of the cycle
                            cycleStartTime = 0;  // Reset the timer
                            analogWrite(fan, 0);  // Ensure the fan is off
                            cycleCount += 1;
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
            float desiredTemperatureExtension = 72.0; // Final extension temperature
            float currentSetpointExtension = setpoint; // Current setpoint for extension
            float temperatureChangeExtension = maxRampRate * elapsedTime; // Allowed change

            // Ramp to the final extension temperature
            if (desiredTemperatureExtension > currentSetpointExtension) {
                setpoint = min(currentSetpointExtension + temperatureChangeExtension, desiredTemperatureExtension);
            } else {
                setpoint = max(currentSetpointExtension - temperatureChangeExtension, desiredTemperatureExtension);
            }

            if (input >= tempPhase3) {
                if (cycleStartTime == 0) {
                    cycleStartTime = currentTime;  // Start the timer
                }
                if (currentTime - cycleStartTime >= finalExtensionTime) {
                    // Move to the complete phase
                    currentPhase = STOPPED; // Transition to stopped phase
                    Serial.println("Final Extension Complete. Stopping...");
                }
            }
            break;

        case STOPPED:
            // Turn off all outputs and reset state
            analogWrite(led, 0);  // Turn off LED
            analogWrite(fan, 0);  // Turn off fan
            Serial.println("System Stopped. All outputs are off.");
            return; // Exit the function
    }

    // Update last update time for ramp rate control
    lastUpdateTime = currentTime;

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
    Serial.print(" Time Since Reaching Setpoint: "); Serial.print(currentTime - cycleStartTime);
    Serial.print(" Setpoint: "); Serial.print(setpoint);
    Serial.print(" Phase: "); Serial.println(cycleCount);


    // Minimal delay for stability
    delay(1);
}

void handleFiveStageThermocyclingWithControlledRampRate() {
    unsigned long currentTime = millis();
    static unsigned long lastUpdateTime = 0; 
    static float lastInputTemperature = input; // Store the last input temperature for rate control
    const float maxTemperatureIncrease = 1.0; // Max 1 degree increase per time step

    float elapsedTime = (currentTime - lastUpdateTime) / 1000.0; // Time in seconds

    // Helper function to limit temperature ramping rate
    auto limitTemperatureIncrease = [&](float targetTemp) {
        float allowedIncrease = maxTemperatureIncrease * elapsedTime;
        if (input > lastInputTemperature + allowedIncrease) {
            setpoint = lastInputTemperature + allowedIncrease;
        } else {
            setpoint = targetTemp;
        }
    };

    // Main control loop for phases
    switch (currentPhase) {
        case INITIAL_DENATURATION:
            limitTemperatureIncrease(tempPhase1);
            if (input >= tempPhase1) {
                if (cycleStartTime == 0) cycleStartTime = currentTime;
                if (currentTime - cycleStartTime >= initialDenaturationTime) {
                    currentPhase = CYCLING;
                    cycleStartTime = phaseStartTime = 0;
                    Serial.println("Initial Denaturation Complete. Starting Cycling...");
                }
            }
            break;

        case CYCLING:
            Serial.println("Starting Cycling...");
            if (cycleCount <= TotalCycles) {
                if (phase == 1) {
                    limitTemperatureIncrease(tempPhase1);
                    if (input >= tempPhase1 - 1) {
                        if (cycleStartTime == 0) cycleStartTime = currentTime;
                        if (currentTime - cycleStartTime >= holdTimePhase1) {
                            phase = 2;
                            cycleStartTime = 0;
                            analogWrite(fan, 255);
                        }
                    }
                } else if (phase == 2) {
                    limitTemperatureIncrease(tempPhase2);
                    if (input <= tempPhase2 + 0.5) {
                        if (cycleStartTime == 0) cycleStartTime = currentTime;
                        if (currentTime - cycleStartTime >= holdTimePhase2) {
                            phase = 3;
                            cycleStartTime = 0;
                            analogWrite(fan, 0);
                        }
                    }
                } else if (phase == 3) {
                    limitTemperatureIncrease(tempPhase3);
                    if (input >= tempPhase3 - 0.5) {
                        if (cycleStartTime == 0) cycleStartTime = currentTime;
                        if (currentTime - cycleStartTime >= holdTimePhase3) {
                            phase = 1;
                            cycleStartTime = 0;
                            analogWrite(fan, 0);
                            cycleCount++;
                        }
                    }
                }
            } else {
                currentPhase = FINAL_EXTENSION;
                cycleStartTime = 0;
                Serial.println("Cycling Complete. Starting Final Extension...");
            }
            break;

        case FINAL_EXTENSION:
            limitTemperatureIncrease(72.0);
            if (input >= tempPhase3) {
                if (cycleStartTime == 0) cycleStartTime = currentTime;
                if (currentTime - cycleStartTime >= finalExtensionTime) {
                    currentPhase = STOPPED;
                    Serial.println("Final Extension Complete. Stopping...");
                }
            }
            break;

        case STOPPED:
            analogWrite(led, 0);
            analogWrite(fan, 0);
            Serial.println("System Stopped. All outputs are off.");
            return;
    }

    // Update last update time and temperature for rate control
    lastUpdateTime = currentTime;
    lastInputTemperature = input;

    // PID Calculations
    error = setpoint - input;
    iTerm += (Ki * error);
    iTerm = constrain(iTerm, 0, 255);
    dInput = (input - lastInput);
    lastInput = input;

    output = Kp * error + iTerm - Kd * dInput;
    int ledBrightness = constrain(static_cast<int>(output), 0, 255);
    analogWrite(led, ledBrightness);

    // Debug Output
    Serial.print("Time: "); Serial.print(currentTime);
    Serial.print(" Temperature: "); Serial.print(input);
    Serial.print(" PID Output: "); Serial.print(ledBrightness);
    Serial.print(" Time Since Reaching Setpoint: "); Serial.print(currentTime - cycleStartTime);
    Serial.print(" Setpoint: "); Serial.print(setpoint);
    Serial.print(" Phase: "); Serial.println(cycleCount);

    delay(1);
}

unsigned long lastUpdateTime = 0;

void rampToTemperature(float targetTemp) {
    const float maxRampRate = 4.0; // Degrees per second
    float elapsedTime = (millis() - lastUpdateTime) / 1000.0;
    float temperatureChange = maxRampRate * elapsedTime;

    // Update setpoint gradually
    if (targetTemp >= setpoint) {
        setpoint = min(setpoint + temperatureChange, targetTemp);
    } else {
        setpoint = max(setpoint - temperatureChange, targetTemp);
    }
}

float i = 0;

void handleFiveStageThermocyclingWithTemperatureSlopeControl_v2() {
    unsigned long currentTime = millis();
    static unsigned long lastUpdateTime = 0; 
    float elapsedTime = (currentTime - lastUpdateTime) / 1000.0; // Time in seconds

    // Calculate the temperature change since the last update
    float temperatureChange = input - lastInput;
    const float maxTemperatureIncreasePerInterval = 1.0; // 1 degree max increase per interval

    // Handle each phase
    switch (currentPhase) {
        case INITIAL_DENATURATION:
            //rampToTemperature(tempPhase1);
            // if(i == 0){
            //     setpoint = tempPhase1-2;
            // }
            // else{
                setpoint = tempPhase1-1;
            
            if (input >= tempPhase1-1) {
                i = 1;
                if (cycleStartTime == 0) cycleStartTime = currentTime;
                if (currentTime - cycleStartTime >= initialDenaturationTime) {
                    currentPhase = CYCLING;
                    cycleStartTime = phaseStartTime = 0;
                    i = 0;
                    Serial.println("Initial Denaturation Complete. Starting Cycling...");
                }
            }
            break;

        case CYCLING:
            if (cycleCount <= TotalCycles) {
                if (phase == 1) {
                    setpoint = tempPhase1-1;
                    //rampToTemperature(tempPhase1);
                    if (input >= tempPhase1 - 1) {
                        if (cycleStartTime == 0) cycleStartTime = currentTime;
                        if (currentTime - cycleStartTime >= holdTimePhase1) {
                            phase = 2;
                            cycleStartTime = 0;
                            analogWrite(fan, 255); // Turn on fan for cooling
                        }
                    }
                } else if (phase == 2) {
                    //rampToTemperature(tempPhase2);
                    setpoint = tempPhase2+0.7;
                    if (input <= tempPhase2 + 0.7) {
                        analogWrite(fan, 0);
                        if (cycleStartTime == 0) cycleStartTime = currentTime;
                        if (currentTime - cycleStartTime >= holdTimePhase2) {
                            phase = 3;
                            cycleStartTime = 0;
                        }
                    }
                } else if (phase == 3) {
                    //rampToTemperature(tempPhase3);
                    setpoint = tempPhase3;
                    if (input >= tempPhase3) {
                        if (cycleStartTime == 0) cycleStartTime = currentTime;
                        if (currentTime - cycleStartTime >= holdTimePhase3) {
                            phase = 1;
                            cycleStartTime = 0;
                            analogWrite(fan, 0); // Ensure fan is off
                            cycleCount++;
                        }
                    }
                }
            } else {
                currentPhase = FINAL_EXTENSION;
                cycleStartTime = 0;
                Serial.println("Cycling Complete. Starting Final Extension...");
            }
            break;

        case FINAL_EXTENSION:
            //rampToTemperature(72.0);
            setpoint = tempPhase3;
            if (input >= tempPhase3) {
                if (cycleStartTime == 0) cycleStartTime = currentTime;
                if (currentTime - cycleStartTime >= finalExtensionTime) {
                    currentPhase = STOPPED;
                    Serial.println("Final Extension Complete. Stopping...");
                }
            }
            break;

        case STOPPED:
            analogWrite(led, 0);
            analogWrite(fan, 0);
            Serial.println("System Stopped. All outputs are off.");
            return;
    }

    // Update last update time and temperature for rate control
    lastUpdateTime = currentTime;

    // PID Calculations
    error = setpoint - input;

    // Accumulate integral term with constraints to prevent wind-up
    iTerm += (Ki * error);
    iTerm = constrain(iTerm, 0, 255);

    // Calculate derivative term based on the rate of change of the input
    dInput = (input - lastInput);

    // Calculate the raw PID output
    output = Kp * error + iTerm - Kd * dInput;

    // Apply temperature slope limiting


    // Constrain the final output to the allowable range
    output = constrain(output, 0, 255);

    if (temperatureChange > maxTemperatureIncreasePerInterval) {
        // If the temperature increase is too high, scale down the output
        output = output * (maxTemperatureIncreasePerInterval / temperatureChange);
    }

    output = constrain(output, 0, 255);


    // Apply the final constrained output to control LED brightness
    int ledBrightness = static_cast<int>(output);
    analogWrite(led, ledBrightness);

    // Update last input temperature
    lastInput = input;

    // Debug Output
    Serial.print("Time: "); Serial.print(currentTime);
    Serial.print(" Temperature: "); Serial.print(input);
    Serial.print(" PID Output: "); Serial.print(ledBrightness);
    Serial.print(" Temperature Change: "); Serial.print(temperatureChange);
    Serial.print(" Time count "); Serial.print(currentTime - cycleStartTime);
    Serial.print(" Setpoint: "); Serial.print(setpoint);
    Serial.print(" Phase: "); Serial.print(phase);
    Serial.print(" cycle "); Serial.println(cycleCount);

    delay(1);
}

// Helper function for ramping temperature


/**
 * Manages basic thermocycling without PID control.
 * This function alternates between high and low temperature thresholds to cycle the temperature control,
 * directly manipulating the hardware components like fans and LEDs to indicate state changes.
 */
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

/**
 * Sets LED brightness to a low level for tunning focus
 */
void TuneFocus() {
    analogWrite(led, 30);
    Serial.println(input);
}

/**
 * Reads and processes temperature data from sensors, updating system's main input variable.
 * Fetches raw sensor data, converts it to a useful voltage representation, and updates the global input variable.
 *
 Fetches raw data using the Get_RawData function, converts this raw data to a voltage based on system gain and reference voltage,
 and updates the global input variable for use in system control algorithms.
 */
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

/**
 * Performs initialization of the ADC used for thermocouple and PT100 sensors, including calibration and sensor selection.
 */
void ThermocoupleSetup(){
    AD7793_Init();
    temp_PT100 = Get_PT100_init();
    Voltage_therm = Get_Thermocouple_init();
}

/**
 * Performs initial setup of the Arduino board and associated sensors.
 * Initializes serial communication, configures pin settings, and prepares the thermocouple system.
 */
void setup() {
    Serial.begin(9600);
    pinMode(led, OUTPUT);
    pinMode(fan, OUTPUT);
    ThermocoupleSetup();
}

void loop() {
    //analogWrite(fan, 255);
    ReadTemperature();           //Using thermocouple to read temperature
    //TuneFocus();                              //Use small power of led for focus
    //handleBenchtopThermocycling();                //SimulateBenchtopThermocycling
    //handle3stageBenchtopThermocycling();
    //handleFiveStageThermocycling();                 
    //handleFiveStageThermocyclingWithHeatingRate();
    handleFiveStageThermocyclingWithTemperatureSlopeControl_v2();
    //handleThermocyclingAndPID();   //Go through 30 thermocycles and do PID
    //handleThermocycling();        //Go through 30 thermocycles without PID
}