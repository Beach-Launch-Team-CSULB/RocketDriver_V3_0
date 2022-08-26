#ifndef SENSORDEFINITIONSPASABANG_H
#define SENSORDEFINITIONSPASABANG_H

#include "EXTSensorClass.h"
#include <array>
#include "ALARApinDefines.h"
#include "fluidSimulationDefinitions.h"

// define number of sensors here
// Renegade SF
#define NUM_SENSORS 13

// initialize all sensor objects here
// PasaFire BangBang Config
//EXT_SENSOR ThrustMountLoadCell1pos{50, 2, A3, 2, 10, 100, false};                       // 0
//EXT_SENSOR ThrustMountLoadCell1neg{50, 5, A2, 2, 10, 100, false};                       // 1
//EXT_SENSOR ChamberPT1{56, 8, ALARA_ANALOG_IN4, &waterGoesVroom, 10, 100, 800, 0.0185, -128.88};                 // 7
EXT_SENSOR MVPneumaticsPT{56, 8, ALARA_ANALOG_IN4, &waterGoesVroom, 2, 10, 50, 0.0185, -128.88};                // 11
EXT_SENSOR FuelLinePT{58, 8, ALARA_ANALOG_IN5, &waterGoesVroom, 2, 10, 100, 0.0190, -124.86};          // 8
EXT_SENSOR LoxLinePT{60, 8, ALARA_ANALOG_IN6, &waterGoesVroom, 2, 10, 100, 0.0190, -122.76};           // 10
EXT_SENSOR FuelTankPT1{62, 8, ALARA_ANALOG_IN7, &waterGoesVroom, 2, 10, 600, 0.0192, -123.62};                    // 14
EXT_SENSOR FuelTankPT2{64, 8, ALARA_ANALOG_IN8, &waterGoesVroom, 2, 10, 600, 0.0192, -124.96};                    // 14
EXT_SENSOR LoxTankPT1{66, 8, ALARA_ANALOG_IN2, &waterGoesVroom, 2, 10, 600, 0.0193, -122.2};                    // 15
EXT_SENSOR LoxTankPT2{68, 8, ALARA_ANALOG_IN3, &waterGoesVroom, 2, 10, 600, 0.0192, -123.84};                    // 15
EXT_SENSOR HiPressPT{70, 8, ALARA_ANALOG_IN1, &waterGoesVroom, 2, 10, 50, 0.0959, -623.08};                 // 16
//EXT_SENSOR MCUtempNode8{200, 8, 70, 2, 5, 10};                                    // 18

//FAKESHIT
//EXT_SENSOR FakeChamberPT1{156, 8, 41, &waterGoesVroom, simulatedInput};                 // 7
EXT_SENSOR FakeFuelLinePT{158, 8, 32, &waterGoesVroom, simulatedInput};          // 8
EXT_SENSOR FakeLoxLinePT{160, 8, 22, &waterGoesVroom, simulatedInput};           // 10
EXT_SENSOR FakeFuelTankPT{162, 8, 31, &waterGoesVroom, simulatedInput};                    // 14
EXT_SENSOR FakeLoxTankPT{166, 8, 21, &waterGoesVroom, simulatedInput};                    // 15
EXT_SENSOR FakeHiPressPT{170, 8, 11, &waterGoesVroom, simulatedInput};                 // 16

/* // Pasafire config
EXT_SENSOR PasafireChamberPT1{189, 8, A14, 10, 100, 1000, false, 0.0125, -123.5};       // 20
EXT_SENSOR PasafireFuelTankPT{190, 8, A20, 2, 10, 100, false, 0.0125, -124.90};        // 21
EXT_SENSOR PasafireLOXTankPT{191, 8, A18, 2, 10, 100, false, 0.0125, -123.57};         // 22
EXT_SENSOR PasafireFuelRegPT{192, 8, A17, 2, 10, 100, false, 0.0124, -123.43};         // 23
EXT_SENSOR PasafireLOXRegPT{193, 8, A16, 2, 10, 100, false, 0.0123, -123.11};          // 24
EXT_SENSOR PasafireFuelLinePT{194, 8, A21, 2, 10, 100, false, 0.0124, -123.17};        // 25
EXT_SENSOR PasafireLOXLinePT{195, 8, A22, 2, 10, 100, false, 0.0124, -126.80};         // 26
EXT_SENSOR PasafirePneumaticPT{196, 8, A15, 2, 10, 100, false, 0.0672, -823.58};       // 27
EXT_SENSOR PasafireThrustMountLoadCell1pos{177, 8, A3, 2, 10, 150, false};            // 28
EXT_SENSOR PasafireThrustMountLoadCell1neg{178, 8, A2, 2, 10, 150, false};            // 29
EXT_SENSOR MCUtempNode8{800, 8, 70, 2, 5, 10, true};                                   // 30 */


// Sensor Array including PasaBang only
//std::array<MCU_SENSOR*, NUM_SENSORS> sensorArray{&ChamberPT1, &FuelLinePT, &LoxLinePT, &FuelTankPT1, &FuelTankPT2, &LoxTankPT1, &LoxTankPT2, &HiPressPT};
std::array<SENSORBASE*, NUM_SENSORS> sensorArray{&MVPneumaticsPT, &FuelLinePT, &LoxLinePT, &FuelTankPT1, &FuelTankPT2, &LoxTankPT1, &LoxTankPT2, &HiPressPT, &FakeFuelLinePT, &FakeLoxLinePT, &FakeFuelTankPT, &FakeLoxTankPT, &FakeHiPressPT};

#endif