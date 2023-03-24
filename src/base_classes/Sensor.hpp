
#ifndef BASECLASS_SENSORCLASS_H_
#define BASECLASS_SENSORCLASS_H_

#include <Arduino.h>
#include <string>
#include <bitset>
#include <ADC.h>
#include "States/SensorStates.hpp"
//#include "fluidSystemSimulation.h"
//#include "ALARAUtilityFunctions.h"
//#pragma once
//#pragma GCC diagnostic ignored "-Wreorder" // go away cringe

// enum for holding ADC input types, may not use this way
enum ADCType
{
    TeensyMCUADC, //built in ADC
    ADS1258,  //not in use yet
    ADS1263,  //not in use yet
    simulatedInput, //for simulated sensor inputs
};
const ADCType ADCType_Default = ADCType::TeensyMCUADC;  //default source here is Teensy ADC

////////////////////////////////////////////////////////////////////////////////
// Base sensor class to hold the various derived classes.
//     Currently, this class is the common elements of
// EXTDigitalDiffLCSensorClass, ALARAVRailSensorClass, ALARAHPSensorClass, and
// EXTSensorClass.  The items in private have been more thoroughly refactored
// than the items in protected.  Ideally, everything should have getters and 
// setters defined in a seperate .cpp, but the linker is whining when I try.
// - Joe
class Sensor
{

private:
    // Constant sensor values.  Usually hardware identifiers.
    const uint32_t _sensorID;
    const uint32_t _sensorNodeID;                      // NodeID the sensor is controlled by
    const ADCType _sensorSource = ADCType::TeensyMCUADC;  //default source here is Teensy ADC
    const uint8_t _ADCinput; //the input that will be read for this sensor that will get used in the ADC read main loop

protected:
    
    SensorState sensorState;

    uint32_t sampleRateSlowMode;        //the sample rate this given sensor will be read at
    uint32_t sampleRateMedMode;         //the sample rate this given sensor will be read at
    uint32_t sampleRateFastMode;        //the sample rate this given sensor will be read at
    uint32_t sampleRateCalibrationMode; //the sample rate this given sensor will be read at

    uint32_t _currentSampleRate = 10;
// This doesn't seem to want to link.  Dang!!
//elapsedMicros _____timer;                      // timer for sensor timing operations
    uint32_t currentRawValue{};               // holds the current value for the sensor

    float maxIntegralSum_Default;
    float minIntegralSum_Default;
    float maxIntegralSum;
    float minIntegralSum;
    
    uint16_t currentCANtimestamp = 0;
    uint32_t currentTimestampSeconds = 0;
    uint32_t currentTimestampMicros = 0;
    uint32_t priorTimestampSeconds = 0;
    uint32_t priorTimestampMicros = 0;
    //bool pullTimestamp = false; // This is in public, for some reason.

    bool nodeIDCheck = false;                           // Whether this object should operate on this node
    bool internalMCUTemp;                       // Is this sensor the MCU internal temp
    bool newSensorValueCheck_CAN = false;                      // Is the current raw value a new read that hasn't been sent yet?
    bool newSensorConvertedValueCheck_CAN = false;                      // Is the current raw value a new read that hasn't been sent yet?
    bool newSensorValueCheck_Log = false;                      // Is the current raw value a new read that hasn't been sent yet?
    bool newConversionCheck = false;                      // Is the current raw value a new read that hasn't been sent yet? 

    float currentConvertedValue{};
    float priorConvertedValue{};

    float linConvCoef1_m_Default;                     // Base calibration coefficients
    float linConvCoef1_b_Default;                     // Base calibration coefficients
    float linConvCoef2_m_Default;                     // adjustment calibration coefficients (intended for application specifics like angle load cell mounting)
    float linConvCoef2_b_Default;                     // adjustment calibration coefficients (intended for application specifics like angle load cell mounting)
    float linConvCoef1_m;                     // Base calibration coefficients
    float linConvCoef1_b;                     // Base calibration coefficients
    float linConvCoef2_m;                     // adjustment calibration coefficients (intended for application specifics like angle load cell mounting)
    float linConvCoef2_b;                     // adjustment calibration coefficients (intended for application specifics like angle load cell mounting)

    //uint16_t rollingSensorArrayRaw[10];       // Array for doing averages of readings
    //uint8_t currentRollingArrayPosition = 0;
    uint32_t currentCalibrationValue{};               // holds the current value for the sensor
    //uint32_t currentRunningSUM = 0;

    bool EMA_Default = true;  //needs a set function still
    bool EMA;  //needs a set function still
    float priorEMAOutput = 0;
    float alphaEMA_Default = 0.7; //1 is no weight to old values, 0 has no weight to new value and will brick
    float alphaEMA;
    float newEMAOutput = 0;

    bool enableIntegralCalc = false;
    bool enableLinearRegressionCalc = true; //not currently using, linreg only calculates when get func requests it
    float currentIntegralSum = 0;
    float currentLinReg_a1 = 0;
    const uint32_t regressionSamples_Default = 5;
    uint32_t regressionSamples;
    float convertedValueArray[5+3] = {};  //should be the same size as regression samples +3 for rolling array index stuff
    float timeStep = 0.01; //timeStep in seconds
    float targetValue = 0;

public:
    //SENSORBASE(){};
    Sensor(uint32_t sensorID,       uint32_t sensorNodeID,    uint8_t ADCinput)
        : _sensorID{sensorID}, _sensorNodeID{sensorNodeID}, _ADCinput{ADCinput}{};

    Sensor(uint32_t sensorID,       uint32_t sensorNodeID,    uint8_t ADCinput,        ADCType sensorSource)
        : _sensorID{sensorID}, _sensorNodeID{sensorNodeID}, _ADCinput{ADCinput}, _sensorSource{sensorSource}{};


    bool pullTimestamp = false;
    virtual void begin();                     //
    virtual void read(ADC& adc);              // updates currentRawValue with current reading, using an activated ADC object
    virtual void stateOperations();

    // Access functions defined in place
    uint32_t getSensorID() const {return _sensorID;}
    uint32_t getSensorNodeID() const {return _sensorNodeID;}
    ADCType getADCtype() const {return _sensorSource;}

    uint32_t getADCinput(){return _ADCinput;};
    //virtual uint32_t getADCinput(bool input1);
    uint32_t getCurrentSampleRate(){return _currentSampleRate;}
    //uint32_t getCurrentRawValue(){return currentRawValue;}// <---------------------------------------------------------------------------------------------------

    //virtual uint32_t getCurrentSampleRate();
    //virtual uint32_t getCurrentRawValue();

    //virtual uint32_t getCurrentRawValue(bool resetRawRead);
    //virtual uint32_t getCurrentRawValue(bool input1, bool resetRawRead);
    uint32_t getCurrentRawValue(){return currentRawValue;}
    uint32_t getCurrentRawValue(bool resetRawRead){if (resetRawRead) {newSensorValueCheck_CAN = false;} return currentRawValue;} //reads and clears new value bool
    float getCurrentConvertedValue(){return currentConvertedValue;}
    //float getCurrentConvertedValue(bool resetConvertedRead){if (resetConvertedRead) {newSensorConvertedValueCheck_CAN = false;} return currentConvertedValue;} //reads and clears new value bool
    float getCurrentConvertedValue(bool resetConvertedRead){if (resetConvertedRead) {newConversionCheck = false;} return currentConvertedValue;} //reads and clears new value bool
    uint16_t getCANTimestamp(){return currentCANtimestamp;}
    uint32_t getTimestampSeconds(){return currentTimestampSeconds;}
    uint32_t getTimestampMicros(){return currentTimestampMicros;}
    //virtual uint8_t getCurrentRollingArrayPosition(){return currentRollingArrayPosition;}
    uint32_t getCurrentRollingAverage(){return currentCalibrationValue;}

    bool getNodeIDCheck(){return nodeIDCheck;}
    bool getNewSensorValueCheckCAN(){return newSensorValueCheck_CAN;}
    bool getNewSensorValueCheckLog(){return newSensorValueCheck_Log;}
    bool getNewSensorConversionCheck(){return newConversionCheck;}
    bool getEnableLinearRegressionCalc(){return enableLinearRegressionCalc;}
    bool getEnableIntegralCalc(){return enableIntegralCalc;}

    float getMaxIntegralSum(){return maxIntegralSum;}
    float getMinIntegralSum(){return minIntegralSum;}

    void setMaxIntegralSum(float maxIntegralSumIn){maxIntegralSum = maxIntegralSumIn;}
    void setMinIntegralSum(float minIntegralSumIn){maxIntegralSum = minIntegralSumIn;}


    void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;} // set the Node ID Check bool function
    void setState(SensorState newState){sensorState = newState;}
    void setSYSTimestamp(uint32_t timestampSeconds, uint32_t timestampMicros)
    {
        if (pullTimestamp)
        {
        priorTimestampSeconds = currentTimestampSeconds;  //shifts the previous current into prior variables
        priorTimestampMicros = currentTimestampMicros;
        currentTimestampSeconds = timestampSeconds;       //sets the new current timestamps from input arguments
        currentTimestampMicros = timestampMicros;
        pullTimestamp = false;
        }
    }

    virtual void linearConversion();
    virtual void exponentialMovingAverage();
    void setTargetValue(float targetValueIn){targetValue = targetValueIn;}

    float getEMAConvertedValue(){return newEMAOutput;}
    //virtual float getDeengergizeOffsetValue();
    float getIntegralSum(){return currentIntegralSum;}
    virtual float getLinRegSlope();

    virtual void initializeLinReg(uint8_t arraySizeIn); //not in use at the moment
    void setEnableIntegralCalc(bool setEnableIn){enableIntegralCalc = setEnableIn;}
    void resetIntegralCalc(bool resetBoolIn, float integralCalcIn = 0){if(resetBoolIn){currentIntegralSum = integralCalcIn;}}  //resets the integral sum, default arg zeros it

    virtual void resetAll();

    void setCurrentRawValue(uint32_t updateCurrentRawValue){currentRawValue = updateCurrentRawValue;} //Why is set raw value here again?
    //resets both the CAN and log bools for when new sample is read
    void setNewSensorValueCheck(bool updateNewSensorValueCheck){newSensorValueCheck_CAN = updateNewSensorValueCheck; newSensorValueCheck_Log = updateNewSensorValueCheck;} 
    void setNewConversionCheck(bool updateNewConversionCheck){newConversionCheck = updateNewConversionCheck;}
    void setCANTimestamp(uint16_t CANTimestamp){currentCANtimestamp = CANTimestamp;}

    //void setCurrentSampleRate(uint32_t updateCurrentSampleRate) {_currentSampleRate = updateCurrentSampleRate; newSensorValueCheck = true; newConversionCheck = false;}
    void setCurrentSampleRate(uint32_t updateCurrentSampleRate) {_currentSampleRate = updateCurrentSampleRate;}
    //virtual void setCurrentSampleRate(uint32_t updateCurrentSampleRate);
    
    void setSampleRateSlowMode(uint32_t updateSampleRateSlowMode) {if(updateSampleRateSlowMode<=2000){sampleRateSlowMode = updateSampleRateSlowMode;}}
    void setSampleRateMedMode(uint32_t updateSampleRateMedMode)   {if(updateSampleRateMedMode<=2000){sampleRateMedMode = updateSampleRateMedMode;}}
    void setSampleRateFastMode(uint32_t updateSampleRateFastMode) {if(updateSampleRateFastMode<=2000){sampleRateFastMode = updateSampleRateFastMode;}}
    void setAlphaEMA(float alphaEMAIn){if(alphaEMAIn >0 && alphaEMAIn <=1){alphaEMA = alphaEMAIn;}}

    //void setRegressionSamples():???
//void setTargetValue(float targetValueIn){targetValue = targetValueIn;}
    //virtual void setDeenergizeOffset(ADC& adc, bool outputOverrideIn);

    //void resetTimer(){timer = 0;}                // resets timer to zero


    /*     //void setRollingSensorArrayRaw(uint8_t arrayPosition, uint16_t sensorValueToArray)
    void setRollingSensorArrayRaw(uint8_t arrayPosition, uint16_t sensorValueToArray)
      {
        rollingSensorArrayRaw[arrayPosition] = sensorValueToArray;
        arrayPosition++;
      } */

/*     void setCurrentCalibrationValue()
    {
    for (size_t i = 0; i < 10; i++)
    {
      currentRunningSUM = currentRunningSUM + rollingSensorArrayRaw[i];
    }
    currentCalibrationValue = currentRunningSUM / 10;
    } */
    //};
};

// need to add differential read toggle somehow 
// - differential boolean variable that allows second input to be chosen or defaulted to correct option
// need to add a way to set other SENSOR types like the RTD sensors over I2C (we'd probably want multiple classes. ADCsensors, I2C sensors, SPI sensors etc - Mat)
// - maybe not the right call to roll into this? Hmm. Need to establish use of SENSOR class with sample rates and real read/sends to see what is better
// That will set me up for incorporating the external ADCs later

#endif