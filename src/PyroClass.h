#ifndef PyroClass_H
#define PyroClass_H

#include "states/PyroStates.hpp"
#include <Arduino.h>
#include "base_classes/state_machine.hpp"

class Pyro : public StateMachine<PyroState>
{
    private:
        const uint32_t pyroID = 99;
        const uint32_t pyroNodeID = 99;
        const u_int8_t ALARA_HP_Channel = 0;
        uint8_t pinDigital = 99;
        uint8_t pinPWM = 99;
        uint8_t pinADC = 99;                              // Valve ADC read pin
        elapsedMicros timer;
//PyroState state;
//PyroState priorState;
        uint32_t liveOutTime;
        uint32_t liveOutTime_Default;
        bool nodeIDCheck;                           // Whether this object should operate on this node
        bool controllerUpdate = false;
//int64_t fireSequenceActuation;              // time on autosequence to actuate IF FireCommanded is used
//int64_t currentAutosequenceTime;              // time of autosequence for comparison under FireCommanded

    public:
    
    // constructor, define the valve ID here, and the pin that controls the valve, setFireDelay is only parameter that can be left blank
        Pyro(uint32_t setPyroID, uint32_t setPyroNodeID, uint8_t setALARA_HP_Channel, uint32_t liveOutTime_Default, bool setNodeIDCheck = false); 
    // default constructor
        Pyro(uint32_t setLiveOutTime = 250000);
    // Alternate constructor with future full implementation, needs the clonedpyro features still
    //    Pyro(int setPyroID, int setPyroNodeID, int setFirePin, int setShuntPin, int setContPin, uint32_t setFireDelay = 0);

    // a start up method, to set pins from within setup()
        void begin(uint8_t pinArrayIn[][11]);

    // access functions defined in place

    // get functions, return the current value of that variable
        uint32_t getPyroID(){return pyroID;}
        uint32_t getPyroNodeID(){return pyroNodeID;}
        //uint32_t getFirePin(){return pinDigital;}
        //uint32_t getArmPin(){return pinPWM;}
        uint8_t getHPChannel(){return ALARA_HP_Channel;}
        uint8_t getPinPWM(){return pinPWM;}
        uint8_t getPinDigital(){return pinDigital;}
        uint8_t getPinADC(){return pinADC;}
        //uint32_t getshuntPin(){return shuntPin;}
        //uint32_t getContPin(){return contCheckPin;}        
        uint32_t getLiveOutTime(){return liveOutTime;}
//int64_t getCurrentAutoSequenceTime(){return currentAutosequenceTime;}
//int64_t getFireTime(){return fireSequenceActuation;}
//PyroState getState(){return state;}
        PyroState getSyncState();
        
        uint32_t getTimer(){return timer;}
        bool getNodeIDCheck(){return nodeIDCheck;}

    // set functions, allows the setting of a variable
        // set function for current autosequence time
//void setCurrentAutoSequenceTime(int64_t timeSetIn)
//{
//    currentAutosequenceTime = timeSetIn;
//}
        // Bypasses state logic to reset the Pyro to Off, used to reset after device has been fired
        void resetPyro(){_setInitialValues(PyroState::Off, getPriorState());}

        void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}
        void setLiveOutTime(uint32_t liveOutTimeIn){if (liveOutTimeIn <= 2000000){liveOutTime = liveOutTimeIn;}}
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


};

#endif