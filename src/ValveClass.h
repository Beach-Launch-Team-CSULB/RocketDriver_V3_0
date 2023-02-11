#ifndef VALVECLASS_H
#define VALVECLASS_H

#include <Arduino.h>
#include "ValveStates.h"
#include "base_classes/state_machine.hpp"

// This class defines the Valve Object that will be used to represent and actuate the valves
// Run begin to set the pins


// 2023 Feb 10
// Current Authors: 
//     Joseph Kessler (joseph.b.kessler@gmail.com)
// 
////////////////////////////////////////////////////////////////////////////////
//     This is a template state machine class that requires an enum of states.
// It is inherited by a class using class Class : StateMachine<StateType> with a
// generic typename.
// 

enum ValveType
{
    NormalClosed,
    NormalOpen,
};

class Valve : public StateMachine<ValveState>
{

    private:
        const uint32_t valveID = 99;                          // Valve ID number 
        const uint8_t valveNodeID = 99;                      // NodeID the valve is controlled by
        ValveType valveType_Default;                  // sets the valve type, either normal closed or normal open
        ValveType valveType;                  // sets the valve type, either normal closed or normal open
        const u_int8_t ALARA_HP_Channel = 0;
        uint8_t pinPWM = 99;                              // Valve PWM pin for actuation
        uint8_t pinDigital = 99;                          // Valve Digital Out pin for actuation
        uint8_t pinADC = 99;                              // Valve ADC read pin
        uint32_t fullDutyTime_Default = 2000;                // Time PWM needs to be at full duty for actuation, in MICROS
        uint32_t fullDutyTime;                // Time PWM needs to be at full duty for actuation, in MICROS
//ValveState _state;
//ValveState _priorState;                           // Tracks the valve state
        elapsedMicros timer;                        // timer for the valve, used for changing duty cycles, in MICROS
        uint16_t fullDuty_Default{256};                // full duty cycle for servo initial actuation
        uint16_t holdDuty_Default{50};                   // partial duty cycle to hold valve in actuated state
        uint16_t warmDuty_Default{5};                   // partial duty cycle to hold valve in actuated state
        uint16_t fullDuty;                // full duty cycle for servo initial actuation
        uint16_t holdDuty;                   // partial duty cycle to hold valve in actuated state
        uint16_t warmDuty;                   // partial duty cycle to hold valve in actuated state
        bool nodeIDCheck;                           // Whether this object should operate on this node
        bool abortHaltDeviceBool;                    // Whether this valve is set by the abort halt flag override
        uint16_t controlSensor1Value;               // For use in control schemes, really a template placement pending needed number and type of samples
        bool controllerUpdate = false;              // flag for when valve stateOps does a state change to update in the controllers
//int64_t fireSequenceActuation;              // time on autosequence to actuate IF FireCommanded is used
//int64_t currentAutosequenceTime;              // time of autosequence for comparison under FireCommanded

    public:
    
    // constructor, define the valve ID here, and the pin that controls the valve, setFireDelay is only parameter that can be left blank
        Valve(uint32_t setValveID, uint8_t setValveNodeID, ValveType setValveType_Default, uint8_t setALARA_HP_Channel, uint32_t setFullDutyTime_Default,  
        bool setAbortHaltDeviceBool = false, uint16_t setHoldDuty_Default = 64, bool setNodeIDCheck = false);
    // Default constructor with no args    
        Valve(ValveType setValveType_Default, bool setNodeIDCheck = false);
    // a start up method, to set pins from within setup()
        void begin(uint8_t pinArrayIn[][11]);

    // access functions defined in place

    // get functions, return the current value of that variable
        uint32_t getValveID(){return valveID;}
        uint8_t getValveNodeID(){return valveNodeID;}
        ValveType getValveType(){return valveType;}
        uint8_t getHPChannel(){return ALARA_HP_Channel;}
        uint8_t getPinPWM(){return pinPWM;}
        uint8_t getPinDigital(){return pinDigital;}
        uint8_t getPinADC(){return pinADC;}
        uint32_t getFullDutyTime(){return fullDutyTime;}
        uint16_t getHoldDuty(){return holdDuty;}
//ValveState getState(){return _state;}
        ValveState getSyncState()
        {
            if(controllerUpdate)
            {
                controllerUpdate = false;
                return getState();
            }
            else {return ValveState::NullReturn;}
        }

//ValveState getPriorState(){return _priorState;}
        uint32_t getTimer(){return timer;}
//int64_t getCurrentAutoSequenceTime(){return currentAutosequenceTime;}
//int64_t getFireTime(){return fireSequenceActuation;}
        bool getNodeIDCheck(){return nodeIDCheck;}
        bool getAbortHaltDeviceBool(){return abortHaltDeviceBool;}

        //every time a state is set, the timer should reset
        //Is the above still true?
        // set function for current autosequence time
 //       void setCurrentAutoSequenceTime(int64_t timeSetIn)
 //       {
 //           currentAutosequenceTime = timeSetIn;
 //       }
    // set the Node ID Check bool function
        void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}
    //set functions 
        void setValveType(uint8_t typeIn){if (typeIn == 0 || typeIn == 1) {valveType = static_cast<ValveType>(typeIn);}}
        void setFullDutyTime(uint32_t fullDutyTimeIn){if (fullDutyTimeIn <= 10000) {fullDutyTime = fullDutyTimeIn;}}
        void setFullDutyCyclePWM(uint16_t FullDutyCyclePWMIn){if (FullDutyCyclePWMIn >= 50 && FullDutyCyclePWMIn <= 256) {fullDuty = FullDutyCyclePWMIn;}}
        void setHoldDutyCyclePWM(uint16_t HoldDutyCyclePWMIn){if (HoldDutyCyclePWMIn >= 25 && HoldDutyCyclePWMIn <= 256) {holdDuty = HoldDutyCyclePWMIn;}}
        void setWarmDutyCyclePWM(uint16_t WarmDutyCyclePWMIn){if (WarmDutyCyclePWMIn >= 25 && WarmDutyCyclePWMIn <= 256) {warmDuty = WarmDutyCyclePWMIn;}}

    // reset all configurable settings to defaults
        void resetAll();
    // functions with executables defined in ValveClasses.cpp
        void resetTimer();              // resets timer to zero, timer increments automatically in microseconds

    // ----- THIS METHOD TO BE RUN EVERY LOOP ------
    // stateOperations will check the current state of the valve and perform any actions that need to be performed
    // for example, if the valve is commanded to open, this needs to be run so that the valve can start opening
    // and it needs to be run every loop so that once enough time has pass the 
        void ioStateOperations();
        void controllerStateOperations();

    // Sensor pull in function for control
        //void controlSensorFetch(uint16_t updateControlSensor1Value){controlSensor1Value = updateControlSensor1Value;}
};

#endif