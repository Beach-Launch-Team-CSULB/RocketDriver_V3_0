
#ifndef BASECLASS_STATEMACHINE_H_
#define BASECLASS_STATEMACHINE_H_

// 2023 Feb 10
// Current Authors: 
//     Joseph Kessler (joseph.b.kessler@gmail.com)
// 
////////////////////////////////////////////////////////////////////////////////
//     This is a template state machine class that requires an enum of states.
// It is inherited by a class using class Class : StateMachine<StateType> with a
// generic typename.
// 
template<typename StateType> // Since this class is templated, it must exist entirely within this file. >:(
class StateMachine
{
private:
    StateType _state;
    StateType _priorState;                           // Tracks the valve state
    int64_t _fireSequenceActuation;              // time on autosequence to actuate IF FireCommanded is used
    int64_t _currentAutoSequenceTime;              // time of autosequence for comparison under FireCommanded

protected:
    // Sets the initial values during a constructor, or overrides both states.
    // Please do not use this outside a constructor.
    void _setInitialValues(StateType state, StateType priorState)
    {
        _state = state;
        _priorState = priorState;
        //_fireSequenceActuation = fireSequenceActuation; // This is unused typically? odd.
    }

    //     Replaces the current state with a new state.  Please prefer setState 
    // over this, unless you have a very good reason.
    void _replaceState(StateType state)
    {
        _state = state;
    }

    // Reverts the current state to the previous state.
    //     Assumes the previous state is legal. Be careful, calling this twice 
    // does nothing!
    void _revertState()
    {
        _state = _priorState;
    }

public:
    // Gets the current state of the state machine.
    StateType getState(){return _state;} 

    // Gets the prior state of the state machine.
    StateType getPriorState(){return _priorState;}

    // Gets the fireSequenceActuation time.  Used with getCurrentAutoSequenceTime().
    // These functions are primarily used with single-use firing events, like launch.
    int64_t getFireTime(){return _fireSequenceActuation;}

    // Gets the current AutoSequence Time.  Used with getFireTime().
    // These functions are primarily used with single-use firing events, like launch.
    int64_t getCurrentAutoSequenceTime(){return _currentAutoSequenceTime;}

    // Sets the state machine to newState.  pushes the current state to priorState.
    // If fireTimeIn is present, performs a set. Maybe I should not have this here? It is not atomic, and setFireTime exists.
    virtual void setState(StateType newState, int64_t fireTimeIn) 
    {
        if (newState != _state)
        {
            _priorState = _state;
            _state = newState;

            _fireSequenceActuation = fireTimeIn; // This is the only fireSequenceActuaion is currently set. 2023 Feb 11 - Joe 
            // Generally, this version of the function is only called in EngineController.
        }
    }

    //     Convinience overloaded function. Should compile away.  If it doesn't,
    // then copy and paste the above code and write it manually without
    // fireSequenceActuation.
    void setState(StateType newState) 
    {
        setState(newState, _fireSequenceActuation);
    }

    void setFireTime(int64_t fireSequenceActuation)
    {
        _fireSequenceActuation = fireSequenceActuation;
    }

    void setCurrentAutoSequenceTime(int64_t timeSetIn)
    {
        _currentAutoSequenceTime = timeSetIn;
    }
};

#endif