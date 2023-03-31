#include "AutoSequenceClass.h"
#include <Arduino.h>

//int64_t maxcountdownint = 36028797018963968; //7 byte (56 bit) signed int max value, this is to stay inside 7 CAN bytes


AutoSequence::AutoSequence(uint32_t setAutoSequenceID, int32_t setCountdownStart_Default, uint32_t setHostNodeID) : autoSequenceID{setAutoSequenceID}, countdownStart_Default{setCountdownStart_Default}, hostNodeID{setHostNodeID}
{
    countdownStart = countdownStart_Default;
}

void AutoSequence::begin()
{
    
}

void AutoSequence::resetAll()
{
    //
    Serial.println("did autosequence restAll run ");
    countdownStart = countdownStart_Default;
}

void AutoSequence::stateOperations()
{
    //Serial.println("does autosequence state ops run?");
    switch (state)
    {
    case AutoSequenceState::Standby:
        
        setCurrentCountdown(countdownStart);
        //setCurrentCountdown(0);
        break;
    
    case AutoSequenceState::RunCommanded:
        
    //if (priorState != AutoSequenceState::Running)
    //{
        state = AutoSequenceState::Running;
        resetTimer();
    //}
        break;
       
    case AutoSequenceState::Running:
        //if (hostNodeID = nodeID)
        //{
            countdownStart = getCountdownStart();
            //Serial.print("CountdownStart from StateOps : ");
            //Serial.print(countdownStart);
            //timer = getTimer(); // Does this line do anything?
            signedTimer = (signed int)getTimer();
            //Serial.print(" timer : ");
            //Serial.print(timer);
            currentCountdown = signedTimer + countdownStart;
            //Serial.print("currentCountdown");
            //Serial.println(currentCountdown);
            setCurrentCountdown(currentCountdown);
        //}
        break;
       
    case AutoSequenceState::Hold:
  
        break;
    
    // All other states require no action
    default:
        break;
    }
}