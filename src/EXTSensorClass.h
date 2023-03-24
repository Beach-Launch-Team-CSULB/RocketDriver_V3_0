#ifndef EXTSENSORCLASS_H
#define EXTSENSORCLASS_H

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "States/SensorStates.hpp"
#include "Base_Classes/Sensor.hpp"
#include "fluidSystemSimulation.h"
#include "ALARAUtilityFunctions.h"


//Declaring setup of the ADC itself for main to find it
void MCUADCSetup(ADC& adc, ADC_REFERENCE refIn0, ADC_REFERENCE refIn1, uint8_t averagesIn0, uint8_t averagesIn1);

class EXT_SENSOR : public Sensor
{
private:
    const uint32_t sampleRateSlowMode_Default = 1;        //the sample rate this given sensor will be read at
    const uint32_t sampleRateMedMode_Default = 10;         //the sample rate this given sensor will be read at
    const uint32_t sampleRateFastMode_Default = 200;        //the sample rate this given sensor will be read at
    const uint32_t sampleRateCalibrationMode_Default = 10;        //the sample rate this given sensor will be read at
    elapsedMicros timer;                      // timer for sensor timing operations
  
  FluidSystemSimulation &fluidSim;

  public:
    void begin();                     // run in setup to get pins going
    void read(ADC& adc);              // updates currentRawValue with current reading, using an activated ADC object
    void stateOperations();
    
    // constructor 1 - standard MCU external ADC read
    EXT_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, FluidSystemSimulation* setFluidSim, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off);
    // constructor 2 - simulated sensor object
    EXT_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput, FluidSystemSimulation* setFluidSim, float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, ADCType setSensorSource = simulatedInput);

    // Access functions defined in place
    
    float getLinRegSlope(){if(nodeIDCheck){currentLinReg_a1 = linearRegressionLeastSquared_PID();} return currentLinReg_a1;}

    // further fuctions defined in SensorClass.cpp

void resetTimer();                // resets timer to zero
    // reset all configurable settings to defaults
    void resetAll();
    void linearConversion();          //Runs a linear sensor conversion 
    void exponentialMovingAverage();
    void initializeLinReg(uint8_t arraySizeIn); //not in use at the moment
    float linearRegressionLeastSquared_PID();
    void accumulatedI_float();
};


#endif