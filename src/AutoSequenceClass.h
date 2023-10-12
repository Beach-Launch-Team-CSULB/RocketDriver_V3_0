#ifndef AUTOSEQUENCECLASS_H
#define AUTOSEQUENCECLASS_H

#include <Arduino.h>
#include "AutoSequenceStates.h"
#include "./Base_Classes/Task_Begin.hpp"
#include "./Base_Classes/Timer.hpp"

class AutoSequence : public Timer, public Task_Begin
{

    private:
        const uint32_t autoSequenceID = 99;
        int32_t countdownStart_Default;        //must be set in Micros so it matches the timer
        int32_t countdownStart;        //must be set in Micros so it matches the timer
        int64_t currentCountdown;
        //elapsedMicros timer; // Keep this instance here as a comment for now. - Joe
        int32_t signedTimer;
        AutoSequenceState state;
        AutoSequenceState priorState;
        uint32_t hostNodeID;      // hostNodeID is for assigning the node that is the host of the autosequence, i.e. the node starting the engine should be in charge of ignition autosequence

    public:

    // constructor    
        AutoSequence(uint32_t setAutoSequenceID, int32_t setCountdownStart, uint32_t setHostNodeID);

    // a start up method,
    void begin();

    // get functions, return the current value of that variable
        uint32_t getAutoSequenceID(){return autoSequenceID;}
        int32_t getCountdownStart(){return countdownStart;}
        int64_t getCurrentCountdown(){return currentCountdown;}
        AutoSequenceState getAutoSequenceState(){return state;}
        uint32_t getHostNodeID(){return hostNodeID;}
        //uint32_t getTimer(){return timer;}
        uint32_t getSignedTimer(){return signedTimer;}
        
    // set functions, allows the setting of a variable
        //void setCountdownStart() {countdownStart;} //function to set a given autosequence countdown timer value
        
        void setCurrentCountdown(int64_t updateCountdown) {currentCountdown = updateCountdown;}

        void setSignedTimer(int32_t updateSignedTimer){signedTimer = updateSignedTimer;}

        void setState(AutoSequenceState newState)
            {
                // if state is already Running, ignore if told RunCommanded
                // all other states, check if the newState is actually new, if so shift state to prior state, set state to newState
                if (!(newState == AutoSequenceState::RunCommanded && state == AutoSequenceState::Running))
                {
                    if (newState != state)
                    {
                        priorState = state;
                    }
                    state = newState;
                }
            }

    // reset all configurable settings to defaults
        void resetAll();
        void setCountdownStart(uint32_t countdownStartIn){if(countdownStartIn >= 1000000 && countdownStartIn <= 60000000){countdownStart = (-1)*static_cast<int32_t>(countdownStartIn);}}
        void stateOperations(); //add into this the functions for managing the countdown initialization and holds

};




#endif