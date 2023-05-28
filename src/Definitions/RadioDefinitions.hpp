#ifndef RADIODEFINITIONS_HPP
#define RADIODEFINITIONS_HPP

#include <Arduino.h>
#include <SPI.h>
#include <RH_NRF24.h>
#include <RH_RF95.h> // Lora SX1276/77/78/79
#include <RHReliableDatagram.h>

#include "ALARApinDefines.h"

#define TRANSMITTER_NETWORK_ADDR 1
#define RECEIVER_NETWORK_ADDR 2

// store in EEPROM but probably not needed, we'll see 
bool NRF24_transmitter = true;  // can switch between trasmitter and receiver, but there has to be a main mode
bool LoRa_transmitter = true;   // can switch between trasmitter and receiver, but there has to be a main mode
RH_NRF24 NRF24Driver{ALARA_NRF_CE, ALARA_NRF_CSN};
RH_RF95 LoRaDriver{ALARA_LORA_CS, ALARA_LORA_INT};

// Manager
// Manager
// Manager
// Manager
// Manager
// Manager
#endif