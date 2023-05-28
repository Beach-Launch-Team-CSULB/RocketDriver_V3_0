// RocketDriver V2.0 Propulsion Control and Data Acquisition - Embedded System Node Program
// Originally by Dan Morgan and Mat Arnold
// For Renegade, Beach Launch Team, Dan Morgan + Brandon Summers' personal machinations and more
//
// Rocket driver developers:
// Lam Khuu - spring 2023 -> present
// Ivan Krat - spring 2023 -> present
// Josheph - spring 2023 -> present  
//
//
//
//
// -------------------------------------------------------------
// Use top level define conditional to determine which system the code is operating
// Maintain definition header sets for a given propulsion system
#include "ControlFunctions.h"
#include "AutoSequenceDefinitions.h"
#include "TankPressControllerDefinitions.h"
#include "EngineControllerDefinitions.h"
#include "ControlFunctions.h"
#include "ALARASensorControllerDefinitions.h"
#include "Definitions/ValveDefinitions.hpp"
#include "Definitions/PyroDefinitions.hpp"
#include "Definitions/RadioDefinitions.hpp"
#include "Definitions/SensorDefinitions.hpp"
// -------------------------------------------------------------

// ----- "COTS" includes ----- //
#include <Arduino.h>
#include <EEPROM.h>
#include <ADC.h>
#include <ADC_util.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <WireKinetis.h>
#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include <InternalTemperature.h>
#include <array>
#include <string>
#include <list>
#include <unordered_map>
using std::string;
#include <IntervalTimer.h>

#include "ALARAUtilityFunctions.h"
#include "ALARABoardControllerClass.h"
#include "ALARABoardControllerDefinitions.h"
//#include "ToMillisTimeTracker.h"
#include "CANRead.h"
#include "CANWrite.h"
#include "OperationFunctionTemplates.h"
#include "ALARApinDefines.h"
#include "fluidSystemSimulation.h"
#include "FlexCAN3Controller.h"
#include "SerialUSBController.h"
#include "extendedIO/extendedIO.h"
#include "ms5607/ms5607.h"
//Trying to figure out RTC stuff with these libs
#include <TimeLib.h>
#include <DS1307RTC.h>

//#define PROPULSIONSYSNODEIDPRESET 2;     //NOT in use normally, for testing with the address IO register inactive
///// ADC /////
ADC* adc = new ADC();

// All ALARA boards need to be set to REF_1V2
ADC_REFERENCE ref0 = ADC_REFERENCE::REF_1V2;
ADC_REFERENCE ref1 = ADC_REFERENCE::REF_1V2;
uint8_t averages0 = 4;
uint8_t averages1 = 4;

MS5607 ALARAbaro;

uint32_t rocketDriverSeconds;
uint32_t rocketDriverMicros;


// Timer for setting main loop debugging print rate
elapsedMillis mainLoopTestingTimer;
elapsedMillis ezModeControllerTimer;
elapsedMillis commandExecuteTimer;
elapsedMillis shittyCANTimer;
elapsedMillis crashTimer;

//For use in doing serial inputs as CAN commands for testing
uint8_t fakeCANmsg; //CAN2.0 byte array, first 4 bytes are ID field for full extended ID compatibility
uint8_t fakeCanIterator = 0;

bool localNodeResetFlag = false; //flag to trigger register reset from commanded reset over CAN
bool abortHaltFlag; //creates halt flag that is a backup override of state machine, am I currently using it?
bool outputOverride = true; // initializes as true to block outputs until changed

///// NODE DECLARATION /////
//default sets to max nodeID intentionally to be bogus until otherwise set

enum nodeID
{
    controlNode = 0,
    globalCall = 1,
    engineLowerProp = 2,
    upperProp = 3 ,
    renegadeAdtlSensor = 4,
    Telemetry = 5,
    LCCLogger_Interpreter = 6,
    PasafireAdtlSensor = 7,
    Pasafire = 8
};

ALARASN thisALARA;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set node ID here to determine which ALARA/ node currently coded for using the nodeID enum
nodeID ALARAnodeID = nodeID::engineLowerProp;                      // ALARA hardware node address
////////////////////////////////////////////// ////////////////////////////////////////////////////////////////////////////////////////////
nodeID ALARAnodeIDfromEEPROM;            //nodeID read out of EEPROM
uint32_t ALARAnodeIDfromEEPROM_errorFlag;            //nodeID read out of EEPROM
bool nodeIDdeterminefromEEPROM;           //boolean flag for if startup is to run the nodeID detect read
uint32_t nodeIDdeterminefromEEPROM_errorFlag;
/*
//uint8_t PropulsionSysNodeID = PROPULSIONSYSNODEIDPRESET;              //engine node = 2, prop node = 3, Pasafire node = 8
uint8_t PropulsionSysNodeID;              //engine node = 2, prop node = 3, Pasafire node = 8
uint8_t PropulsionSysNodeIDfromEEPROM;    //PropulsionSysNodeID read out of EEPROM
uint32_t PropulsionSysNodeIDfromEEPROM_errorFlag;    //PropulsionSysNodeID read out of EEPROM
*/
uint32_t vehicleStatefromEEPROM_errorFlag;
uint32_t missionStatefromEEPROM_errorFlag;

const uint8_t configVerificationKey = 166; //upgrade to a map later

bool NRF24TransmitterFromEEPROM;
uint32_t NRF24TransmitterFromEEPROM_errorFlag;
bool LoRaTransmitterFromEEPROM;
uint32_t LoRaTransmitterFromEEPROM_errorFlag;


///// WATCHDOG SYSTEM /////
elapsedMillis propulsionControlWatchdog;                  // Watchdog timer that must be reset by ground control over bus to prevent an autovent
uint32_t propulsionControlWatchdogVentTime = 120000;   // 120 seconds in millis gives two minutes to reestablish control before autovent, DISABLE IN FLIGHT

/* ///// ADC /////
ADC* adc = new ADC();
 */
///// LED /////
elapsedMillis sinceLED;

///// CAN /////
CAN_message_t message;
CAN_message_t rxmsg;
CAN_message_t extended;
CAN_stats_t Can0stats;
bool CANSensorReportConverted = false;
bool NewCommandMessage{false};
bool NewConfigMessage{false};

CAN_filter_t wtf;

FlexCan3Controller Can2msgController;
SerialUSBController SerialUSBdataController;

const int CAN2busSpeed = 500000; // CAN2.0 baudrate - do not set above 500000 for full distance run bunker to pad

bool startup{true}; // bool for storing if this is the first loop on startup, ESSENTIAL FOR STATE MACHINE OPERATION (maybe not anymore?)

uint32_t loopCount {0};// for debugging

// Set the global command, and global state
Command currentCommand{command_NOCOMMAND}; 
VehicleState currentVehicleState{VehicleState::passive};
VehicleState priorVehicleState{VehicleState::passive};
MissionState currentMissionState{MissionState::passive};
MissionState priorMissionState{MissionState::passive};
// SET "staticTest = true" FOR GROUND TESTING, "false" FOR FLIGHT!!!!!
bool staticTest = true;


commandMSG currentCommandMSG{};
configMSG currentConfigMSG{};

uint32_t vehicleStateAddressfromEEPROM_errorFlag;
uint32_t missionStateAddressfromEEPROM_errorFlag;
//AutoSequence stuff for main
int64_t currentCountdownForMain;


////// Set EEPROM addresses
// Change these up occasionally to reduce write cycle wear on the same bytes
// I could use EEPROM itself to store current start byte of my data and automate iterating this. Good idea for future upgrade.
// if there is a hiccup, grab the old addresses
uint16_t vehicleStateAddress1{30};
uint16_t vehicleStateAddress2{31};
uint16_t vehicleStateAddress3{32};
uint16_t missionStateAddress1{33};
uint16_t missionStateAddress2{34};
uint16_t missionStateAddress3{35};
uint16_t nodeIDAddress1{40}; // nodeID  enum
uint16_t nodeIDAddress2{41}; // nodeID  enum
uint16_t nodeIDAddress3{42}; // nodeID  enum

uint16_t NRF24RadioAddress1{80};    // Store boolean transmitter value from radio definition
uint16_t NRF24RadioAddress2{81};    // Store boolean transmitter value from radio definition 
uint16_t NRF24RadioAddress3{82};    // Store boolean transmitter value from radio definition
uint16_t LoRaRadioAddress1{83};     // Store boolean transmitter value from radio definition
uint16_t LoRaRadioAddress2{84};     // Store boolean transmitter value from radio definition
uint16_t LoRaRadioAddress3{85};     // Store boolean transmitter value from radio definition

//
uint16_t PropulsionSysNodeIDAddress1{16}; // legacy?
uint16_t PropulsionSysNodeIDAddress2{17}; // legacy?
uint16_t PropulsionSysNodeIDAddress3{18}; // legacy?
uint16_t nodeIDDetermineAddress1{19};     // legacy?
uint16_t nodeIDDetermineAddress2{20};     // legacy?
uint16_t nodeIDDetermineAddress3{21};     // legacy?


//-------------------------------------------------------//
void setup() {

#define RESET_ALL_EEPROM_ADDRESSES_AND_SET_NEW_ADDRESSES 1  // only set to 1 if we want to change up the eeprom addresses
                                                            // and set the initial states again.
                                                            // reminder to set this back to 0 after this is done until another EEPROM address change
                                                            // especially during operation
#if RESET_ALL_EEPROM_ADDRESSES_AND_SET_NEW_ADDRESSES
    resetAllEEPROMAddresses();
    tripleEEPROMwrite(ALARAnodeID, nodeIDAddress1, nodeIDAddress2, nodeIDAddress3);
    tripleEEPROMwrite(static_cast<uint8_t>(currentVehicleState), vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3);
    tripleEEPROMwrite(static_cast<uint8_t>(currentMissionState), missionStateAddress1, missionStateAddress2, missionStateAddress3);
    if (ALARAnodeID == nodeID::Telemetry)
    {
        tripleEEPROMwrite(NRF24_transmitter, NRF24RadioAddress1, NRF24RadioAddress2, NRF24RadioAddress3);
        tripleEEPROMwrite(LoRa_transmitter, LoRaRadioAddress1, LoRaRadioAddress2, LoRaRadioAddress3);
    }
#endif
    /////////////////////////////////////////////////////
    // -- Initialize hardware functions and globals -- //
    /////////////////////////////////////////////////////
    startup = true;   // Necessary to set startup to true for the code loop so it does one startup loop for the state machine before entering regular loop behavior
    Serial.begin(9600); // Value is arbitrary on Teensy, it will initialize at the MCU dictate baud rate regardless what you feed this
    Wire.begin();
    SPI.begin();

    ///// ----- Insert a board rev check to pin defines here, if it fails disable our GPIO? ------ //

    /////////////////////////////////
    // -- EEPROM initialization -- //
    /////////////////////////////////
    // I'm not sure why this is done at all..?

    // -----Read Last State off eeprom and update -----
    // I think the only things that should be stored in the flash are the vehicle and mission states. The rest seem redundant 
    ALARAnodeIDfromEEPROM = static_cast<nodeID>(tripleEEPROMread(nodeIDAddress1, nodeIDAddress2, nodeIDAddress3, ALARAnodeIDfromEEPROM_errorFlag));
    currentVehicleState = static_cast<VehicleState>(tripleEEPROMread(vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3, vehicleStateAddressfromEEPROM_errorFlag));
    currentMissionState = static_cast<MissionState>(tripleEEPROMread(missionStateAddress1, missionStateAddress2, missionStateAddress3, missionStateAddressfromEEPROM_errorFlag));
    NRF24TransmitterFromEEPROM = tripleEEPROMread(NRF24RadioAddress1, NRF24RadioAddress2, NRF24RadioAddress3, NRF24TransmitterFromEEPROM_errorFlag);
    LoRaTransmitterFromEEPROM = tripleEEPROMread(LoRaRadioAddress1, LoRaRadioAddress2, LoRaRadioAddress3, LoRaTransmitterFromEEPROM_errorFlag);

    // Only write to EEPROM the node ID if manual ID define is present at top of Main
    //#ifdef PROPULSIONSYSNODEIDPRESET
    //tripleEEPROMupdate(static_cast<uint8_t>(2), PropulsionSysNodeIDAddress1, PropulsionSysNodeIDAddress2, PropulsionSysNodeIDAddress3);
    //#endif
    //PropulsionSysNodeIDfromEEPROM = tripleEEPROMread(PropulsionSysNodeIDAddress1, PropulsionSysNodeIDAddress2, PropulsionSysNodeIDAddress3, PropulsionSysNodeIDfromEEPROM_errorFlag);
    //PropulsionSysNodeID = 2; //tripleEEPROMread(PropulsionSysNodeIDAddress1, PropulsionSysNodeIDAddress2, PropulsionSysNodeIDAddress3, PropulsionSysNodeIDfromEEPROM_errorFlag);
    nodeIDdeterminefromEEPROM = tripleEEPROMread(nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3, nodeIDdeterminefromEEPROM_errorFlag);
    startupStateCheck(currentVehicleState, currentCommand);
    // Set NewCommandMessage true so the command from startupStateCheck gets read by commandExecute
    NewCommandMessage = true;

    #ifdef ALARAV2_1
    // ----- MUX Setups for ALARA -----
    // Board Addressing MUX
    MUXSetup(true, ALARA_DIGITAL_ADDRESS_1, ALARA_DIGITAL_ADDRESS_2, ALARA_DIGITAL_ADDRESS_3, ALARA_DIGITAL_ADDRESS_4);
    
    // MOVE NODEDETECTSHITHERE!!!
    // Check map for ALARASN configutation
    lookupALARASNmap(thisALARA, ALARAnodeIDfromEEPROM);

    // NOR Flash CS pin MUX
    MUXSetup(false, ALARA_NOR_S0, ALARA_NOR_S1, ALARA_NOR_S2);
    #endif

    // ----- Run the Node ID Detection Function -----
    //PropulsionSysNodeID = NodeIDDetect(nodeID, nodeIDdeterminefromEEPROM, nodeIDfromEEPROM); // - OVERHAUL WITH NEW FUNCTION AND SYSTEM
    //PropulsionSysNodeID = PROPULSIONSYSNODEIDPRESET;       //For manually assigning NodeID isntead of the address read, make sure to comment out for operational use
    //PropulsionSysNodeID = thisALARA.propulsionSysNodeID;
    // Write 0 to byte for nodeIDDetermineAddress after reading it after a reset
    //tripleEEPROMupdate(0, nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3);
    //CHEATER OVERRIDE!!!!!
    //PropulsionSysNodeID = 8;

    //////////////////////////////
    // -- Initialize CAN Bus -- //
    //////////////////////////////
    //     This seems almost strange being here.  It should be with the rest of 
    // the CAN stuff near the bottom for clarity.

    // -----Initialize CAN-----
    vectorCommandBufferInitialize(32);  //number of vector elements memory is reserved for
    vectorConfigBufferInitialize(32); //number of vector elements memory is reserved for
    // CAN0 - FlexCAN 2.0 bus
    Can0.begin(CAN2busSpeed);

    // -----Initialize ADCs-----
    // Quick and dirty way to setup unique ADC use case on the LC/TC Teensy node
    #ifdef TEENSY3_X
        // Setting alt I2C pins because I used default I2C pins
        Wire.setSDA(8);
        Wire.setSCL(7);
        ref0 = ADC_REFERENCE::REF_3V3;  // ask what the purpose of ADC reference is 
        ref1 = ADC_REFERENCE::REF_1V2;  //
        averages0 = 32;
        averages1 = 32;
    #endif
    MCUADCSetup(*adc, ref0, ref1, averages0, averages1);

    //////////////////////////////////
    // -- Initialize Sensors Bus -- //
    //////////////////////////////////
    //     Finally!! Functional programming!!! Kinda.  This uses some very well 
    // made atomic functions to verify and initialize all the valves and 
    // sensors.
////////////////////////////////////////////////////////////////////////////////

    // -----Run Valve PropulsionSysNodeID Check-----
    // ID Check verifies that the right devices are attached to the right ALARA.
    ValveNodeIDCheck(valveArray, ALARAnodeIDfromEEPROM);
    PyroNodeIDCheck(pyroArray, ALARAnodeIDfromEEPROM);
    SensorNodeIDCheck(sensorArray, ALARAnodeIDfromEEPROM);
    SensorNodeIDCheck(HPsensorArray, ALARAnodeIDfromEEPROM);
    //SensorNodeIDCheck(TCsensorArray, ALARAnodeIDfromEEPROM);

    // -----Run Valve Setup-----
    valveSetUp(valveArray, ALARA_HP_Array);
    pyroSetUp(pyroArray, ALARA_HP_Array);
    engineControllerSetup(engineControllerArray);
    tankPressControllerSetup(tankPressControllerArray);

    // -----Run AutoSequence Setup-----
    autoSequenceSetUp(autoSequenceArray);
    
    // -----Run Sensor Setup -----
    sensorSetUp(sensorArray);
    sensorSetUp(HPsensorArray);
    sensorSetUp(TCsensorArray);
    #ifdef TEENSY3_X
    coldJunctionRenegade.begin();
    #endif

    // ----- Set Controller Dependent Sensor Settings -----
    controllerSensorSetup(valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);

    #ifdef ALARAV2_1
    pinModeExtended(ALARA_DIGITAL_ADDRESS_OE, OUTPUT);
    ALARAbaro.init(ALARA_BPS_CSN,OSR_1024);
    boardController.begin();
    #endif
    
    /////////////////////////
    // -- Start Can Bus -- //
    /////////////////////////
    //     The other half of the CAN bus initialization.  I'm not sure why this
    // is seperated from the first chunk.
    // 
    SerialUSBdataController.setPropStatusPrints(true);
    SerialUSBdataController.setPropCSVStreamPrints(false);
    Can0.startStats();
    Can0.setTxBufferSize(64);
}


/////////////////////
// -- Main Loop -- //
/////////////////////
//     This is the main loop of the driver.  Typically, these run on a while 
// loop forever.  I'm not sure if this is how it works under the hood here. -Joe
// 
// This function was filled with functions such as:
//Serial.println("Do I get past command execute?");
//Serial.println("Do I get past new config msg?");
//     In the event this is needed again, the following macro should be used 
// instead:
// 
//#define VERBOSE_PRINTS
#ifdef VERBOSE_PRINTS
#define DEBUG_SPRINT_PASS(_NAME) Serial.println("Do I get past" _NAME '?');
#else
#define DEBUG_SPRINT_PASS(_NAME)
#endif
// Debug Serial Print Pass. Duh what else could it mean ha ha ha ඞ


////////////////////////////////////////////////////////////////////////////////

void loop() 
{
    // Lazy "SensorTasks" for the RTD sensor
    //if (coldJunctionRenegade.getSensorNodeID() == PropulsionSysNodeID) -> Lam
    if (coldJunctionRenegade.getSensorNodeID() == ALARAnodeIDfromEEPROM)
    {
        coldJunctionRenegade.read();
    }

    //fakesensorShit(rocketDriverSeconds, rocketDriverMicros, &myTimeTrackingFunction);

    ///// Custom function for tracking miliseconds and seconds level system time for timestamping /////
    // let it run each loop in addition to when called to keep it synced up
    myTimeTrackingFunction(rocketDriverSeconds, rocketDriverMicros);

    // --- Read CAN bus and update current command ---
    if(CANread(Can0, configVerificationKey, NewConfigMessage, currentCommand, currentConfigMSG, ALARAnodeIDfromEEPROM) && !startup) // do not execute on the first loop
    {
        //Serial.print("CAN Message Recieved: ");
        //Serial.println(currentCommand); //currently only does the command not any message
    }
    if(SerialAsCANread())
    {
        //Serial.println("Command Entered");
    }
    //Serial.println("Do I get to live?");
    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// ------ MESSAGE PROCESSING BLOCK ----- /////  
    //                 CAN MESSAGES                //
    //pull next command message from buffer, if there is one
    if (!NewCommandMessage)
    {
        NewCommandMessage = readRemoveVectorBuffer(currentCommandMSG);
        currentCommand = currentCommandMSG.commandEnum;
    }
    //Serial.println("Do I get past new command message?");
    //process command
    //if (commandExecuteTimer >= 2000)
    //{
    commandExecute(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, NewCommandMessage, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);
    //Serial.println("Do I get past command execute?");
    //}
    //pull next config message from buffer, if there is one
    NewConfigMessage = readRemoveVectorBuffer(currentConfigMSG);
    //Serial.println("Do I get past new config msg?");
    //process config message
    configMSGread(currentConfigMSG, NewConfigMessage, valveArray, pyroArray, sensorArray, autoSequenceArray, tankPressControllerArray, engineControllerArray, waterGoesVroom);
    //Serial.println("Do I get past config msg read?");
    ///// ------------------------------------ /////  

    //                 RADIO MESSAGES              //

    if (ALARAnodeIDfromEEPROM == nodeID::Telemetry)
    {

    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////
    // -- Controller tasks -- //
    ////////////////////////////
    // State machine and controller think operations.
    // 
    if (ezModeControllerTimer >= 20) // 5 = 200Hz controller rate, 20 = 50Hz rate
    {
        // -----Process Commands Here-----
        vehicleStateMachine(currentVehicleState, priorVehicleState, currentCommand, boardController, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray, waterGoesVroom, abortHaltFlag, outputOverride);
        missionStateMachine(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, boardController, autoSequenceArray, staticTest, abortHaltFlag);
        
        #ifdef ALARAV2_1
        controllerDataSync(valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray);
        #endif
        autoSequenceTasks(autoSequenceArray, ALARAnodeIDfromEEPROM);
        tankPressControllerTasks(tankPressControllerArray, ALARAnodeIDfromEEPROM, IgnitionAutoSequence);
        engineControllerTasks(engineControllerArray, ALARAnodeIDfromEEPROM, IgnitionAutoSequence);
        controllerDeviceSync(currentVehicleState, priorVehicleState, currentCommand, valveArray, pyroArray, autoSequenceArray, sensorArray, tankPressControllerArray, engineControllerArray, waterGoesVroom, abortHaltFlag);
        //fluid sim run
        waterGoesVroom.fluidSystemUpdate();
        ezModeControllerTimer = 0;

        //ALARAbaro.update();
        //ALARAbaro.print_all();
    }
    DEBUG_SPRINT_PASS("Controller Tasks");
    
    // -----Advance needed controller system tasks (tank press controllers, ignition autosequence, . ..) ----- //
    // -----Advance needed propulsion system tasks (valve, pyro, sensors, . ..) ----- //

    ////////////////////////////////
    // -- Valve and Pyro tasks -- //
    ////////////////////////////////
    //     Updates the Valves and Pyros.  Interrupts are disabled for this
    // operation.
    // 

    cli(); // disables interrupts to ensure complete propulsion output state is driven

    valveTasks(valveArray, ALARAnodeIDfromEEPROM, outputOverride, *autoSequenceArray.at(0));
    pyroTasks(pyroArray, ALARAnodeIDfromEEPROM, outputOverride, *autoSequenceArray.at(0));
    //MUST KEEP HP OVERRIDE AFTER VALVE/PYRO TASKS
    #ifdef ALARAV2_1
    ALARAHPOverride(ALARA_HP_Array, outputOverride);
    #endif
    sei(); // reenables interrupts after propulsion output state set is completed
    DEBUG_SPRINT_PASS("Valve and Pyro Tasks");

    ////////////////////////////////
    // -- Sensor tasks -- //
    ////////////////////////////////
    // Updates sensors.
    // 

    sensorTasks(sensorArray, *adc, ALARAnodeIDfromEEPROM, rocketDriverSeconds, rocketDriverMicros);
    ALARAHPsensorTasks(HPsensorArray, *adc, ALARAnodeIDfromEEPROM, rocketDriverSeconds, rocketDriverMicros, outputOverride);
    TCsensorTasks(TCsensorArray, *adc, ALARAnodeIDfromEEPROM, rocketDriverSeconds, rocketDriverMicros);
    DEBUG_SPRINT_PASS("Sensor Tasks");
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////
    // -- EEPROM Update -- //
    /////////////////////////
    // Updates the EEPROM?? Why?
    // 
    // -----Update States on EEPROM -----
    // ONLY write if something new to write!!! Don't spam EEMPROM it will kill the memory bytes physically if overused
    //if ((static_cast<uint8_t>(currentVehicleState)) != (tripleEEPROMread(vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3, vehicleStatefromEEPROM_errorFlag)))
    //{
    tripleEEPROMupdate(static_cast<uint8_t>(currentVehicleState), vehicleStateAddress1, vehicleStateAddress2, vehicleStateAddress3);
    //}
    //if ((static_cast<uint8_t>(currentMissionState)) != (tripleEEPROMread(missionStateAddress1, missionStateAddress2, missionStateAddress3, missionStatefromEEPROM_errorFlag)))
    //{
    tripleEEPROMupdate(static_cast<uint8_t>(currentMissionState), missionStateAddress1, missionStateAddress2, missionStateAddress3);
        /*   Serial.println("Does current vs prior MISSION state EEPROM protect work as expected? ");
        Serial.print(" priorMissionState : ");
        Serial.print(static_cast<uint8_t>(priorMissionState));
        Serial.print(" currentMissionState : ");
        Serial.println(static_cast<uint8_t>(currentMissionState)); */
    //}

    // When the radio switches between RX and TX mode, use EEPROM update everytime
    // TripleEEPROMupdate(RADDDDDDDDDDDDIIIIIIIIIIIIIOOOOOOOOOOOOOOOO)


    // Reset function to reboot Teensy with internal reset register
    // Need to figure out how to rework using this feature with reworked ID system
    //TeensyInternalReset(localNodeResetFlag, nodeIDDetermineAddress1, nodeIDDetermineAddress2, nodeIDDetermineAddress3);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// ----- All outgoing messages managed here ----- /////
//                      CAN MESSAGES                    //
    // Run every loop
    if (shittyCANTimer >= 1000)
    {
        Can2msgController.setExternalStateChange(true); //cheater force for quasi messages, AM I USING THIS AT ALL RIGHT NOW???
        shittyCANTimer = 0;
    }
    Can2msgController.controllerTasks(Can0, currentVehicleState, currentMissionState, currentCommand, engineControllerArray, tankPressControllerArray, valveArray, pyroArray, sensorArray, HPsensorArray, autoSequenceArray, waterGoesVroom, ALARAnodeIDfromEEPROM);
    /*   Serial.println("Do I get past Can2 controllerTasks?");
    Can0stats = Can0.getStats();
    Serial.print("Can0stats.ringRxMax? ");
    Serial.println(Can0stats.ringRxMax);
    Serial.print("Can0stats.ringTxHighWater? ");
    Serial.println(Can0stats.ringTxHighWater); */

//                      RADIO MESSAGES                    //

    if (ALARAnodeID == nodeID::Telemetry)
    {

    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// ----- Serial Print Functions ----- /////
    if (mainLoopTestingTimer >= 250)
    {
        SerialUSBdataController.propulsionNodeStatusPrints(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, currentCommandMSG, currentConfigMSG, autoSequenceArray, engineControllerArray, waterGoesVroom, tankPressControllerArray, valveArray, pyroArray, sensorArray, HPsensorArray, ALARAnodeIDfromEEPROM);
        SerialUSBdataController.propulsionNodeCSVStreamPrints(currentVehicleState, priorVehicleState, currentMissionState, priorMissionState, currentCommand, currentCommandMSG, currentConfigMSG, autoSequenceArray, engineControllerArray, waterGoesVroom, tankPressControllerArray, valveArray, pyroArray, sensorArray, ALARAnodeIDfromEEPROM);
        mainLoopTestingTimer = 0; //resets timer to zero each time the loop prints
        Serial.print(" Crash Timer Millis: ");
        Serial.println(crashTimer);
    }

    // Resets the startup bool, DO NOT REMOVE
    startup = false;

}