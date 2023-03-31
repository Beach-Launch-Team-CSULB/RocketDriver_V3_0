
#include "PyroClass.h"
#include <Arduino.h>
#include "extendedIO/extendedIO.h"

Pyro::Pyro(uint32_t setPyroID, uint32_t setPyroNodeID, uint8_t setALARA_HP_Channel, uint32_t setLiveOutTime_Default,  bool setNodeIDCheck)
    : pyroID{setPyroID}, pyroNodeID{setPyroNodeID}, ALARA_HP_Channel{setALARA_HP_Channel}, liveOutTime_Default{setLiveOutTime_Default}, nodeIDCheck{setNodeIDCheck}
{
    liveOutTime = liveOutTime_Default;
    _setInitialValues(PyroState::Off, PyroState::Off);
    //state = PyroState::Off;
}

Pyro::Pyro(uint32_t setLiveOutTime) : liveOutTime{setLiveOutTime}
{

}

void Pyro::begin(uint8_t pinArrayIn[][11])
{
    if (nodeIDCheck)
    {
        pinDigital = pinArrayIn[0][ALARA_HP_Channel];
        pinPWM = pinArrayIn[1][ALARA_HP_Channel];
        pinADC = pinArrayIn[2][ALARA_HP_Channel];
        
        pinModeExtended(pinDigital, OUTPUT);
        pinModeExtended(pinPWM, OUTPUT);
        digitalWriteExtended(pinDigital, 0);
        digitalWriteExtended(pinPWM, 0);
    }
}

void Pyro::resetAll()
{
    //rest all configurable settings to defaults
    setLiveOutTime(liveOutTime_Default);
}

PyroState Pyro::getSyncState()
{
    if(controllerUpdate)
    {
        controllerUpdate = false;
        return getState();
    }
    else {return PyroState::NullReturn;}
}

void Pyro::ioStateOperations()
{
    switch (getState())
    {
    // physical output state actions only, NO LOGIC
    case PyroState::On:
        digitalWriteExtended(pinDigital, 1);
        digitalWriteExtended(pinPWM, 1);
        break;
    case PyroState::Off:
        digitalWriteExtended(pinDigital, 0);
        digitalWriteExtended(pinPWM, 0);
        break;        
    case PyroState::Fired:
        digitalWriteExtended(pinDigital, 0);
        digitalWriteExtended(pinPWM, 0);
        break;        
    // All other states require no action
    default:
        break;
    }
}

void Pyro::controllerStateOperations()
{
    switch (getState())
    {
    // if a valve has been commanded to fire, it will start actuation after appropriate delay, normal closed actuate open, normal open actuate closed
    // every state change should reset the timer
/*     case PyroState::FireCommanded:
        state = PyroState::OnCommanded;
        timer = 0;
        break;
 */
    // if a pyro is commanded on, turns on 
    case PyroState::OnCommanded:
        if (getPriorState() != PyroState::Fired) //only allow OnCommanded to do anything if not Fired
        {
            if (getPriorState() != PyroState::On)
            {
                _setInitialValues(PyroState::On, getPriorState());
//state = PyroState::On;
                resetTimer();
            }
            else {_setInitialValues(PyroState::On, getPriorState());}
        }
        else {_setInitialValues(PyroState::Fired, getPriorState());} //keeps us in Fired state
        break;

    case PyroState::On:
        if(getTimer() >= liveOutTime)
        {
            _setInitialValues(PyroState::Fired, getPriorState());
//state = PyroState::Fired;
            //resetTimer();
        }
        break;

    // if a pyro is commanded off, turns off immediately, not sure I need this at all the way we do on valves
    case PyroState::OffCommanded:
        if (getPriorState() != PyroState::Fired) //only allow OnCommanded to do anything if not Fired
        {
            if (getPriorState() != PyroState::Off)
            {
                _setInitialValues(PyroState::Off, getPriorState());
//state = PyroState::Off;
            //resetTimer();
            }
            else {_setInitialValues(PyroState::Off, getPriorState());}
        }
        else {_setInitialValues(PyroState::Fired, getPriorState());} //keeps us in Fired state
        break;
        
    case PyroState::Off:
        //resetTimer();
        break;        
    case PyroState::FireCommanded:
        if (getCurrentAutoSequenceTime() >= getFireTime())
        {
            //     If AutosequenceTime is greater than fireSequenceActuation, 
            // and the valve has already fired, replace the command.
            _setInitialValues(PyroState::OnCommanded, getPriorState());
//state = PyroState::OnCommanded;
        }
        break;
    case PyroState::Fired:
        //do I need anything for Fired?
        break;        
    
    // All other states require no action
    default:
        break;
    }

/*             Serial.print("pyro fire commanded time check: ");
            Serial.print(currentAutosequenceTime);
            Serial.print(" >= ");
            Serial.print(fireSequenceActuation);
            Serial.println(); */
}


// old shit
/* void Pyro::stateOperations()
{
    switch (state)
    {
    // if a valve has been commanded to fire, it will start actuation after appropriate delay, normal closed actuate open, normal open actuate closed
    // every state change should reset the timer

    // if a pyro is commanded on, turns on 
    case PyroState::OnCommanded:
        if (priorState != PyroState::On)
        {
        state = PyroState::On;
        timer = 0;
        }
        else {state = PyroState::On;}
        break;

    case PyroState::On:
        digitalWriteFast(firePin, 1);
        digitalWriteFast(armPin, 1);
        if(timer >= liveOutTime)
        {
            state = PyroState::Off;
            timer = 0;
            controllerUpdate = true;
        }
        break;

    // if a pyro is commanded off, turns off immediately, not sure I need this at all the way we do on valves
    case PyroState::OffCommanded:
        if (priorState != PyroState::Off)
        {
        state = PyroState::Off;
        timer = 0;
        }
        else {state = PyroState::Off;}
        break;
        
    case PyroState::Off:
        digitalWriteFast(firePin, 0);
        digitalWriteFast(armPin, 0);
        timer = 0;
        break;        
    
    // All other states require no action
    default:
        break;
    }
} */