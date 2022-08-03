#include "PyroClass.h"
#include <Arduino.h>
#include "extendedIO/extendedIO.h"



Pyro::Pyro(uint32_t setPyroID, uint32_t setPyroNodeID, uint8_t setFirePin, uint8_t setArmPin, uint32_t setLiveOutTime,  bool setNodeIDCheck)
                : pyroID{setPyroID}, pyroNodeID{setPyroNodeID}, firePin{setFirePin}, armPin{setArmPin}, liveOutTime{setLiveOutTime}, nodeIDCheck{setNodeIDCheck}
{
    state = PyroState::Off;
    timer = 0;
}

Pyro::Pyro(uint32_t setLiveOutTime) : liveOutTime{setLiveOutTime}
{
    
}

void Pyro::begin()
{
    if (nodeIDCheck)
    {
        pinModeExtended(firePin, OUTPUT);
        pinModeExtended(armPin, OUTPUT);
        digitalWriteExtended(firePin, 0);
        digitalWriteExtended(armPin, 0);
    }
}

void Pyro::resetTimer()
{
    timer = 0;
}

void Pyro::resetAll()
{
    //rest all configurable settings to defaults
}

PyroState Pyro::getSyncState()
{
    if(controllerUpdate)
    {
        controllerUpdate = false;
        return state;
    }
    else {return PyroState::NullReturn;}
}

void Pyro::stateOperations()
{
    switch (state)
    {
    // physical output state actions only, NO LOGIC
    case PyroState::On:
        digitalWriteExtended(firePin, 1);
        digitalWriteExtended(armPin, 1);
        break;
    case PyroState::Off:
        digitalWriteExtended(firePin, 0);
        digitalWriteExtended(armPin, 0);
        break;        
    case PyroState::Fired:
        digitalWriteExtended(firePin, 0);
        digitalWriteExtended(armPin, 0);
        break;        
    // All other states require no action
    default:
        break;
    }
}

void Pyro::controllerStateOperations()
{
    switch (state)
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
        if (priorState != PyroState::Fired) //only allow OnCommanded to do anything if not Fired
        {
            if (priorState != PyroState::On)
            {
            state = PyroState::On;
            timer = 0;
            }
            else {state = PyroState::On;}
        }
        else {state = PyroState::Fired;} //keeps us in Fired state
        break;

    case PyroState::On:
        //digitalWriteFast(firePin, 1);
        //digitalWriteFast(armPin, 1);
        if(timer >= liveOutTime)
        {
            state = PyroState::Fired;
            timer = 0;
        }
        break;

    // if a pyro is commanded off, turns off immediately, not sure I need this at all the way we do on valves
    case PyroState::OffCommanded:
        if (priorState != PyroState::Fired) //only allow OnCommanded to do anything if not Fired
        {
            if (priorState != PyroState::Off)
            {
            state = PyroState::Off;
            //timer = 0;
            }
            else {state = PyroState::Off;}
        }
        else {state = PyroState::Fired;} //keeps us in Fired state
        break;
        
    case PyroState::Off:
        //timer = 0;
        break;        
    case PyroState::Fired:
        //do I need anything for Fired?
        break;        
    
    // All other states require no action
    default:
        break;
    }
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