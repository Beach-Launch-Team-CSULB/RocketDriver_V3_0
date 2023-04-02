#ifndef CONTROLCOMMANDS_H
#define CONTROLCOMMANDS_H
#pragma once
#include <Arduino.h>

// Parts of this are obsolete, or will be. reset and config messages will go elsewhere/change, need to add more commands for new features possibly

// this enum hold all commands to be sent to the teensy. All commands assumed to be uint8_t type 

// in main file as global declare: Command CurrentCommand{command_NOCOMMAND} to hold the current command

enum Command // seems to be the third byte of the node state frame. Is there a reason to catch this?
{
    // we reserve 0 to be a no command state
    command_NOCOMMAND = 0,
    command_passive = 1,                          //Start of global states
    command_standby = 3,
    command_test_exit = 4,                      //not implemented yet into state machine
    command_test = 5,
    command_abort = 7,
    command_vent = 9,
    command_HiPressArm = 11,
    command_HiPressPressurized = 13,
    command_TankPressArm = 15,
    command_TankPressPressurized = 17,
    command_fireArm = 19,
    command_fire = 21,                          //End of global states
    command_ExitOffNominal = 22,
    command_EnterOffNominal = 23,
    command_closeHiPress = 32,                  //Start of individual device states
    command_openHiPress = 33,
    command_closeHiPressVent = 34,
    command_openHiPressVent = 35,
    command_closeLoxVent = 36,
    command_openLoxVent = 37,
    command_closeLoxPressValve = 38,
    command_openLoxPressValve = 39,
    command_closeLoxPressLineVent = 40,
    command_openLoxPressLineVent = 41,
    command_closeFuelVent = 42,
    command_openFuelVent = 43,
    command_closeFuelPressValve = 44,
    command_openFuelPressValve = 45,
    command_closeFuelPressLineVent = 46,
    command_openFuelPressLineVent = 47,
    command_closeLoxMV = 48,
    command_openLoxMV = 49,    
    command_closeFuelMV = 50,
    command_openFuelMV = 51,
    command_engineIgniterPyro1_Off = 52,            //DO NOT MATCH ID FORMAT
    command_engineIgniterPyro1_On = 53,
    command_engineIgniterPyro2_Off = 54,
    command_engineIgniterPyro2_On = 55,                         //End of individual device states
    command_allSensorsOff = 126,                        //For Calibration Routines/testing
    command_allSensorsSlow = 127,                       //For Calibration Routines/testing
    command_allSensorsMedium = 128,                     //For Calibration Routines/testing
    command_allSensorsFast = 129,                       //For Calibration Routines/testing
    command_allSensorsCalibration = 130,                //For Calibration Routines/testing
    command_propProgSetting = 237,                      // placeholder to use for sending specific program
    command_node1RESET = 238,                            // Use for reset command that writes to internal reset register, needs a following byte to ID the node to be valid
    command_node2RESET = 239,                            // Use for reset command that writes to internal reset register, needs a following byte to ID the node to be valid
    command_node3RESET = 240,                            // Use for reset command that writes to internal reset register, needs a following byte to ID the node to be valid
    command_node4RESET = 241,                            // Use for reset command that writes to internal reset register, needs a following byte to ID the node to be valid
    command_node5RESET = 242,                            // Use for reset command that writes to internal reset register, needs a following byte to ID the node to be valid
    command_node6RESET = 243,                            // Use for reset command that writes to internal reset register, needs a following byte to ID the node to be valid
    command_node7RESET = 244,                            // Use for reset command that writes to internal reset register, needs a following byte to ID the node to be valid
    command_GLOBALRESET = 254,                          // Use for reset command that writes to internal reset register
    command_reseved = 255,                              // 255 Reserved for future use
    command_SIZE, // not a valid command but it is useful for checking if recieved messages are valid commands, see CANRead. Always leave this at the end of the enum listcomm
};


//only using a struct here for potential future command format expansion
struct commandMSG
{
    union
    {
    uint8_t commandByte;
    Command commandEnum;
    };
};

#endif