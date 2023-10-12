#ifndef ALARASENSORCONTROLLERCLASS_H
#define ALARASENSORCONTROLLERCLASS_H

#include <Arduino.h>
#include "./States/ControllerStates.h"
#include "./Base_Classes/state_machine.hpp"
#include "./States/PyroStates.hpp"
#include "./States/SensorStates.hpp"
#include "./States/ValveStates.hpp"
#include "./ALARASNConfigurations.h"
#include "./Sensors/ALARAHPSensorClass.h"
#include "./Base_Classes/Task_Begin.hpp"

class ALARAV2SensorController : public StateMachine<ALARAV2SensorControllerState>, Task_Begin
{
    private:
        const uint32_t controllerID;                        // Controller ID number - not super useful right now? Maybe for state reporting.
        const uint8_t controllerNodeID;                     // node the controller is running on
        bool nodeIDCheck;                           // Whether this object should operate on this node
// ALARAV2SensorControllerState state;
// ALARAV2SensorControllerState priorState;
        SensorState sensorStateInternal;                    // Use one sensor state inside here to toggle all sensors on controller
        SensorState sensorStateGNC;                    // Use one sensor state inside here to toggle all sensors on controller
        SensorState sensorStateALARAHP;                    // Use one sensor state inside here to toggle all sensors on controller
        
        // bools for if the possible on board sensors are active
        // set default as false and flip to true based on board configuration and use case  
        bool BNO055_active;
        bool BMI085_active;
        bool KX134_1211_active;
        bool SAM_M8Q_GPS_active;
        bool MS5607_active;
        // On board MCU ADC reads for voltage rails
        bool ALARA_VINRail_active;
        bool ALARA_5VRail_active;
        bool ALARA_3V3Rail_active;

/*         ALARAHP_SENSOR &HP1;
        ALARAHP_SENSOR &HP2;
        ALARAHP_SENSOR &HP3;
        ALARAHP_SENSOR &HP4;
        ALARAHP_SENSOR &HP5;
        ALARAHP_SENSOR &HP6;
        ALARAHP_SENSOR &HP7;
        ALARAHP_SENSOR &HP8;
        ALARAHP_SENSOR &HP9;
        ALARAHP_SENSOR &HP10; */
        
    public:
    
    // constructor
        ALARAV2SensorController(uint32_t setControllerID, uint8_t setControllerNodeID, bool setNodeIDCheck = false,
                                bool setBNO055_active = false, bool setBMI085_active = false, bool setKX134_1211_active = false, bool setSAM_M8Q_GPS_active = false, bool setMS5607_active = false,
                                bool setALARA_VINRail_active = false, bool setALARA_5VRail_active = false, bool setALARA_3V3Rail_active = false);
    // a start up method, to set pins from within setup()
    void begin();
        //void setALARAHPSensors(ALARAHP_SENSOR* setHP1, ALARAHP_SENSOR* setHP2, ALARAHP_SENSOR* setHP3,ALARAHP_SENSOR* setHP4, ALARAHP_SENSOR* setHP5, ALARAHP_SENSOR* setHP6,ALARAHP_SENSOR* setHP7, ALARAHP_SENSOR* setHP8, ALARAHP_SENSOR* setHP9, ALARAHP_SENSOR* setHP10);

    // get functions, return the current value of that variable
        uint32_t getControllerID(){return controllerID;}
        uint8_t getControllerNodeID(){return controllerNodeID;}
        bool getNodeIDCheck(){return nodeIDCheck;}
// ALARAV2SensorControllerState getState(){return state;}

    // set functions, allows the setting of a variable
    // set the Node ID Check bool function
        void setNodeIDCheck(bool updatedNodeIDCheck) {nodeIDCheck = updatedNodeIDCheck;}
    // controller state set function
// void setState(ALARAV2SensorControllerState newState)
// {
//     if (newState != state)
//     {
//         priorState = state;
//     }
//     state = newState;
// }
    // sensor bool set functions
        void setBNO055_active(bool ALARAsensorSetIN) {BNO055_active = ALARAsensorSetIN;}
        void setBMI085_active(bool ALARAsensorSetIN) {BMI085_active = ALARAsensorSetIN;}
        void setKX134_1211_active(bool ALARAsensorSetIN) {KX134_1211_active = ALARAsensorSetIN;}
        void setSAM_M8Q_GPS_active(bool ALARAsensorSetIN) {SAM_M8Q_GPS_active = ALARAsensorSetIN;}
        void setMS5607_active(bool ALARAsensorSetIN) {MS5607_active = ALARAsensorSetIN;}
        void setALARA_VINRail_active(bool ALARAsensorSetIN) {ALARA_VINRail_active = ALARAsensorSetIN;}
        void setALARA_5VRail_active(bool ALARAsensorSetIN) {ALARA_5VRail_active = ALARAsensorSetIN;}
        void setALARA_3V3Rail_active(bool ALARAsensorSetIN) {ALARA_3V3Rail_active = ALARAsensorSetIN;}

        void ALARAconfigurationSensorSet(ALARASN& thisALARA);

    // ----- THIS METHOD TO BE RUN EVERY LOOP ------
    // stateOperations will check the current state of the valve and perform any actions that need to be performed
    // for example, if the valve is commanded to open, this needs to be run so that the valve can start opening
    // and it needs to be run every loop so that once enough time has pass the 
        void stateOperations();

};

#endif