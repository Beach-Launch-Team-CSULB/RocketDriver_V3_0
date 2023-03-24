#ifndef EXTDIGITALDIFFLCSENSORCLASS_H
#define EXTDIGITALDIFFLCSENSORCLASS_H

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "States/SensorStates.hpp"
#include "Base_Classes/Sensor.hpp"
#include "fluidSystemSimulation.h"
#include "ALARAUtilityFunctions.h"

class DIG_LC_SENSOR : public Sensor
{
private:
    uint8_t ADCinput2;               //the input that will be read for this sensor that will get used in the ADC read main loop
    const uint32_t sampleRateSlowMode_Default = 1;        //the sample rate this given sensor will be read at
    const uint32_t sampleRateMedMode_Default = 10;         //the sample rate this given sensor will be read at
    const uint32_t sampleRateFastMode_Default = 200;        //the sample rate this given sensor will be read at
    const uint32_t sampleRateCalibrationMode_Default = 10;        //the sample rate this given sensor will be read at
    const uint32_t conversionSendRate_Default = 100;        //the sample rate this given sensor will be read at
    uint32_t conversionSendRate;        //the sample rate this given sensor will be read at
    elapsedMicros timer;                      // timer for sensor timing operations
    uint32_t currentRawValue2{};               // holds the current value for the sensor
    uint32_t currentRawDiffValue{};               // holds the current value for the sensor
  
  FluidSystemSimulation &fluidSim;

  public:
//bool pullTimestamp = false;
    void begin();                     // run in setup to get pins going
    void read(ADC& adc);              // updates currentRawValue with current reading, using an activated ADC object
    void stateOperations();
    
    // constructor 1 - standard MCU external ADC read
    DIG_LC_SENSOR(uint32_t setSensorID, uint32_t setSensorNodeID, uint8_t setADCinput1, uint8_t setADCinput2, FluidSystemSimulation* setFluidSim, uint32_t setSampleRateSlowMode_Default, uint32_t setSampleRateMedMode_Default, uint32_t setSampleRateFastMode_Default, uint32_t setConversionSendRate_Default, float setLinConvCoef1_m_Default = 1, float setLinConvCoef1_b_Default = 0, float setLinConvCoef2_m_Default = 1, float setLinConvCoef2_b_Default = 0, float setMaxIntegralSum_Default = 2500, float setMinIntegralSum_Default = -2500, uint32_t setCurrentSampleRate = 0, SensorState setSensorState = Off);

    // Access functions defined in place
    uint32_t getADCinput2(){return ADCinput2;}
    uint32_t getCurrentRawValue(bool input1, bool resetRawRead){if (resetRawRead) {newSensorValueCheck_CAN = false;} if (!input1) {return currentRawValue;} else{return currentRawValue2;};} //reads and clears new value bool

    float getLinRegSlope(){currentLinReg_a1 = linearRegressionLeastSquared_PID(); return currentLinReg_a1;}

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