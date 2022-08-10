#ifndef ALARAPINDEFINES_H
#define ALARAPINDEFINES_H
#pragma once

// currently no quick way to automate this, need to choose the appropriate define for the board in use
#define TEENSY3_X

#ifdef TEENSY3_X
// ALARA pin mapping definitions
// Analog Inputs
#define ALARA_ANALOG_IN1 A2
#define ALARA_ANALOG_IN2 A3
#define ALARA_ANALOG_IN3 A15
#define ALARA_ANALOG_IN4 A14
#define ALARA_ANALOG_IN5 A21
#define ALARA_ANALOG_IN6 A22
#define ALARA_ANALOG_IN7 A11
#define ALARA_ANALOG_IN8 A10
#define ALARA_ANALOG_3_3RAIL A23
#define ALARA_ANALOG_5RAIL A20

// Digital I/O for MUXed addressing system (or V2 plain digital inputs?)
#define ALARA_DIGITAL_ADDRESS_1 26
#define ALARA_DIGITAL_ADDRESS_2 27
#define ALARA_DIGITAL_ADDRESS_3 28
#define ALARA_DIGITAL_ADDRESS_4 29
#define ALARA_DIGITAL_ADDRESS_OE 30

// Digital Outputs
/* #define ALARA_DIGITAL_EXTERNAL_1 63
#define ALARA_DIGITAL_EXTERNAL_2 64 */

//High Power Outputs
// Digital Outs other than 9,10 intentionally using bottom Teensy pads, leave floating and drive with only PWM
#define ALARA_HIGHPOWER_DIGITALOUT1 40
#define ALARA_HIGHPOWER_DIGITALOUT2 41
#define ALARA_HIGHPOWER_DIGITALOUT3 42
#define ALARA_HIGHPOWER_DIGITALOUT4 43
#define ALARA_HIGHPOWER_DIGITALOUT5 44
#define ALARA_HIGHPOWER_DIGITALOUT6 45
#define ALARA_HIGHPOWER_DIGITALOUT7 46
#define ALARA_HIGHPOWER_DIGITALOUT8 47
#define ALARA_HIGHPOWER_DIGITALOUT9 24
#define ALARA_HIGHPOWER_DIGITALOUT10 25
#define ALARA_HIGHPOWER_PWMOUT1 5
#define ALARA_HIGHPOWER_PWMOUT2 6
#define ALARA_HIGHPOWER_PWMOUT3 8
#define ALARA_HIGHPOWER_PWMOUT4 7
#define ALARA_HIGHPOWER_PWMOUT5 2
#define ALARA_HIGHPOWER_PWMOUT6 35
#define ALARA_HIGHPOWER_PWMOUT7 10
#define ALARA_HIGHPOWER_PWMOUT8 9
#define ALARA_HIGHPOWER_PWMOUT9 23
#define ALARA_HIGHPOWER_PWMOUT10 22
#define ALARA_HIGHPOWER_ANALOGREAD1 A7
#define ALARA_HIGHPOWER_ANALOGREAD2 A6
#define ALARA_HIGHPOWER_ANALOGREAD3 A0
#define ALARA_HIGHPOWER_ANALOGREAD4 A19
#define ALARA_HIGHPOWER_ANALOGREAD5 A18
#define ALARA_HIGHPOWER_ANALOGREAD6 A17
#define ALARA_HIGHPOWER_ANALOGREAD7 A1
#define ALARA_HIGHPOWER_ANALOGREAD8 A13
#define ALARA_HIGHPOWER_ANALOGREAD9 A12
#define ALARA_HIGHPOWER_ANALOGREAD10 A24

// Bus Pins
#define ALARA_PRIMARY_I2C_SCL 19
#define ALARA_PRIMARY_I2C_SDA 18
//#define ALARA_PRIMARY_UART_MOSI 0
//#define ALARA_PRIMARY_UART_MISO 1
#define ALARA_PRIMARY_SPI_SCK 13
#define ALARA_PRIMARY_SPI_MOSI 11
#define ALARA_PRIMARY_SPI_MISO 12
/* #define ALARA_CAN_CS 28
#define ALARA_CAN_INTN0 27
#define ALARA_CAN_INTN1 25
#define ALARA_CAN_INTN 73
#define ALARA_CAN_CLK0 72 */
#define ALARA_RS485_D 1
#define ALARA_RS485_R 0

// NOR - FAKED WITH BOTTOM PINS TO BE DUMMIES
#define ALARA_NOR_S0 52
#define ALARA_NOR_S1 53
#define ALARA_NOR_S2 54
#define ALARA_NOR_OE 55

/* // SMD Sensors
#define ALARA_IMU1_RST 24
#define ALARA_IMU2_CSBG 52
#define ALARA_IMU2_CSBA 51
#define ALARA_ACC_CSN 47
#define ALARA_ACC_TRIG 48
#define ALARA_BPS_CSN 53

// On board indicators - LEDs and Buzzer
#define ALARA_PWM_EXPANDER_OE 65
#define ALARA_BUZZ 30

// NRF and LORA external jumper connectors
#define ALARA_NRF_CSN 55
#define ALARA_NRF_CE 54
#define ALARA_NRF_IRQ 26

#define ALARA_LORA_CS 42
#define ALARA_LORA_INT 40
#define ALARA_LORA_RST 41

// GPS
#define ALARA_GPS_INT 56
#define ALARA_GPS_RST 57 */


#endif
///////////////////////////////////////////////////////////////////////////
#ifdef ALARAV2_1
// ALARA pin mapping definitions
// Analog Inputs
#define ALARA_ANALOG_IN1 A2
#define ALARA_ANALOG_IN2 A3
#define ALARA_ANALOG_IN3 A15
#define ALARA_ANALOG_IN4 A14
#define ALARA_ANALOG_IN5 A21
#define ALARA_ANALOG_IN6 A22
#define ALARA_ANALOG_IN7 A11
#define ALARA_ANALOG_IN8 A10
#define ALARA_ANALOG_3_3RAIL A23
#define ALARA_ANALOG_5RAIL A20

// Digital I/O for MUXed addressing system (or V2 plain digital inputs?)
#define ALARA_DIGITAL_ADDRESS_1 61
#define ALARA_DIGITAL_ADDRESS_2 60
#define ALARA_DIGITAL_ADDRESS_3 59
#define ALARA_DIGITAL_ADDRESS_4 62
#define ALARA_DIGITAL_ADDRESS_OE 58

// Digital Outputs
#define ALARA_DIGITAL_EXTERNAL_1 63
#define ALARA_DIGITAL_EXTERNAL_2 64

//High Power Outputs
#define ALARA_HIGHPOWER_DIGITALOUT1 87
#define ALARA_HIGHPOWER_DIGITALOUT2 86
#define ALARA_HIGHPOWER_DIGITALOUT3 85
#define ALARA_HIGHPOWER_DIGITALOUT4 84
#define ALARA_HIGHPOWER_DIGITALOUT5 83
#define ALARA_HIGHPOWER_DIGITALOUT6 82
#define ALARA_HIGHPOWER_DIGITALOUT7 81
#define ALARA_HIGHPOWER_DIGITALOUT8 80
#define ALARA_HIGHPOWER_DIGITALOUT9 79
#define ALARA_HIGHPOWER_DIGITALOUT10 78
#define ALARA_HIGHPOWER_PWMOUT1 5
#define ALARA_HIGHPOWER_PWMOUT2 6
#define ALARA_HIGHPOWER_PWMOUT3 8
#define ALARA_HIGHPOWER_PWMOUT4 7
#define ALARA_HIGHPOWER_PWMOUT5 2
#define ALARA_HIGHPOWER_PWMOUT6 35
#define ALARA_HIGHPOWER_PWMOUT7 10
#define ALARA_HIGHPOWER_PWMOUT8 9
#define ALARA_HIGHPOWER_PWMOUT9 23
#define ALARA_HIGHPOWER_PWMOUT10 22
#define ALARA_HIGHPOWER_ANALOGREAD1 A7
#define ALARA_HIGHPOWER_ANALOGREAD2 A6
#define ALARA_HIGHPOWER_ANALOGREAD3 A0
#define ALARA_HIGHPOWER_ANALOGREAD4 A19
#define ALARA_HIGHPOWER_ANALOGREAD5 A18
#define ALARA_HIGHPOWER_ANALOGREAD6 A17
#define ALARA_HIGHPOWER_ANALOGREAD7 A1
#define ALARA_HIGHPOWER_ANALOGREAD8 A13
#define ALARA_HIGHPOWER_ANALOGREAD9 A12
#define ALARA_HIGHPOWER_ANALOGREAD10 A24

// Bus Pins
#define ALARA_PRIMARY_I2C_SCL 19
#define ALARA_PRIMARY_I2C_SDA 18
//#define ALARA_PRIMARY_UART_MOSI 0
//#define ALARA_PRIMARY_UART_MISO 1
#define ALARA_PRIMARY_SPI_SCK 13
#define ALARA_PRIMARY_SPI_MOSI 11
#define ALARA_PRIMARY_SPI_MISO 12
#define ALARA_CAN_CS 28
#define ALARA_CAN_INTN0 27
#define ALARA_CAN_INTN1 25
#define ALARA_CAN_INTN 73
#define ALARA_CAN_CLK0 72
#define ALARA_RS485_D 1
#define ALARA_RS485_R 0

// NOR
#define ALARA_NOR_S0 43
#define ALARA_NOR_S1 46
#define ALARA_NOR_S2 44
#define ALARA_NOR_OE 45

// SMD Sensors
#define ALARA_IMU1_RST 24
#define ALARA_IMU2_CSBG 52
#define ALARA_IMU2_CSBA 51
#define ALARA_ACC_CSN 47
#define ALARA_ACC_TRIG 48
#define ALARA_BPS_CSN 53

// On board indicators - LEDs and Buzzer
#define ALARA_PWM_EXPANDER_OE 65
#define ALARA_BUZZ 30

// NRF and LORA external jumper connectors
#define ALARA_NRF_CSN 55
#define ALARA_NRF_CE 54
#define ALARA_NRF_IRQ 26

#define ALARA_LORA_CS 42
#define ALARA_LORA_INT 40
#define ALARA_LORA_RST 41

// GPS
#define ALARA_GPS_INT 56
#define ALARA_GPS_RST 57


#endif
///////////////////////////////////////////////////////////////////////////
#ifdef ALARAV2
// ALARA pin mapping definitions
// Analog Inputs
#define ALARA_ANALOG_IN1 A13
#define ALARA_ANALOG_IN2 A12
#define ALARA_ANALOG_IN3 A15
#define ALARA_ANALOG_IN4 A14
#define ALARA_ANALOG_IN5 A21
#define ALARA_ANALOG_IN6 A22
#define ALARA_ANALOG_IN7 A11
#define ALARA_ANALOG_IN8 A10
#define ALARA_ANALOG_3_3RAIL A20
#define ALARA_ANALOG_5RAIL A23

// Digital I/O for MUXed addressing system (or V2 plain digital inputs?)
#define ALARA_DIGITAL_ADDRESS_1 79
#define ALARA_DIGITAL_ADDRESS_2 80
#define ALARA_DIGITAL_ADDRESS_3 81
#define ALARA_DIGITAL_ADDRESS_4 82

// Digital Outputs
#define ALARA_DIGITAL_EXTERNAL_1 83
#define ALARA_DIGITAL_EXTERNAL_2 84
#define ALARA_DIGITAL_EXTERNAL_3 63
#define ALARA_DIGITAL_EXTERNAL_4 64

//High Power Outputs
#define ALARA_HIGHPOWER_DIGITALOUT1 54
#define ALARA_HIGHPOWER_DIGITALOUT2 51
#define ALARA_HIGHPOWER_DIGITALOUT3 52
#define ALARA_HIGHPOWER_DIGITALOUT4 53
#define ALARA_HIGHPOWER_DIGITALOUT5 55
#define ALARA_HIGHPOWER_DIGITALOUT6 87
#define ALARA_HIGHPOWER_DIGITALOUT7 48
#define ALARA_HIGHPOWER_DIGITALOUT8 47
#define ALARA_HIGHPOWER_DIGITALOUT9 86
#define ALARA_HIGHPOWER_DIGITALOUT10 85
#define ALARA_HIGHPOWER_PWMOUT1 5
#define ALARA_HIGHPOWER_PWMOUT2 6
#define ALARA_HIGHPOWER_PWMOUT3 8
#define ALARA_HIGHPOWER_PWMOUT4 7
#define ALARA_HIGHPOWER_PWMOUT5 2
#define ALARA_HIGHPOWER_PWMOUT6 10
#define ALARA_HIGHPOWER_PWMOUT7 9
#define ALARA_HIGHPOWER_PWMOUT8 22
#define ALARA_HIGHPOWER_PWMOUT9 30
#define ALARA_HIGHPOWER_PWMOUT10 29
#define ALARA_HIGHPOWER_ANALOGREAD1 A7
#define ALARA_HIGHPOWER_ANALOGREAD2 A6
#define ALARA_HIGHPOWER_ANALOGREAD3 A0
#define ALARA_HIGHPOWER_ANALOGREAD4 A19
#define ALARA_HIGHPOWER_ANALOGREAD5 A18
#define ALARA_HIGHPOWER_ANALOGREAD6 A17
#define ALARA_HIGHPOWER_ANALOGREAD7 A16
#define ALARA_HIGHPOWER_ANALOGREAD8 A9
#define ALARA_HIGHPOWER_ANALOGREAD9 A1
#define ALARA_HIGHPOWER_ANALOGREAD10 A24

// Bus Pins
#define ALARA_PRIMARY_I2C_SCL 19
#define ALARA_PRIMARY_I2C_SDA 18
#define ALARA_PRIMARY_UART_MOSI 0
#define ALARA_PRIMARY_UART_MISO 1
#define ALARA_PRIMARY_SPI_SCK 13
#define ALARA_PRIMARY_SPI_MOSI 11
#define ALARA_PRIMARY_SPI_MISO 12
#define ALARA_CAN_CS 74
#define ALARA_CAN_INTN0 42
#define ALARA_CAN_INTN1 40
#define ALARA_CAN_INTN 41
#define ALARA_CAN_CLK0 75
#define ALARA_RS485_D 72
#define ALARA_RS485_R 73

// NOR
#define ALARA_NOR_S0 43
#define ALARA_NOR_S1 46
#define ALARA_NOR_S2 44
#define ALARA_NOR_OE 45

// SMD Sensors
#define ALARA_IMU1_RST 68
#define ALARA_IMU2_CSBG 77
#define ALARA_IMU2_CSBA 78
#define ALARA_ACC_CSN 66
#define ALARA_ACC_TRIG 67
#define ALARA_BPS_CSN 76

// On board indicators - LEDs and Buzzer
#define ALARA_PROG_LED_BLU1 69
#define ALARA_PROG_LED_GRN1 70
#define ALARA_PROG_LED_RED1 71
#define ALARA_PROG_LED_BLU2 58
#define ALARA_PROG_LED_GRN2 59
#define ALARA_PROG_LED_RED2 60
#define ALARA_BUZZ 16

// NRF and LORA external jumper connectors
#define ALARA_NRF_CSN 28
#define ALARA_NRF_CE 27
#define ALARA_NRF_IRQ 65

#define ALARA_LORA_CS 24
#define ALARA_LORA_INT 26
#define ALARA_LORA_RST 25

// GPS
#define ALARA_GPS_INT 56
#define ALARA_GPS_RST 57

#endif

//ALARA HP channel array
//position zero is a null channel don't use
uint8_t ALARA_HP_Array[3][11] = {{0,ALARA_HIGHPOWER_DIGITALOUT1, ALARA_HIGHPOWER_DIGITALOUT2, ALARA_HIGHPOWER_DIGITALOUT3, ALARA_HIGHPOWER_DIGITALOUT4, ALARA_HIGHPOWER_DIGITALOUT5, ALARA_HIGHPOWER_DIGITALOUT6, ALARA_HIGHPOWER_DIGITALOUT7, ALARA_HIGHPOWER_DIGITALOUT8, ALARA_HIGHPOWER_DIGITALOUT9, ALARA_HIGHPOWER_DIGITALOUT10},
                                    {0,ALARA_HIGHPOWER_PWMOUT1, ALARA_HIGHPOWER_PWMOUT2, ALARA_HIGHPOWER_PWMOUT3, ALARA_HIGHPOWER_PWMOUT4, ALARA_HIGHPOWER_PWMOUT5, ALARA_HIGHPOWER_PWMOUT6, ALARA_HIGHPOWER_PWMOUT7, ALARA_HIGHPOWER_PWMOUT8, ALARA_HIGHPOWER_PWMOUT9, ALARA_HIGHPOWER_PWMOUT10},
                                    {0,ALARA_HIGHPOWER_ANALOGREAD1, ALARA_HIGHPOWER_ANALOGREAD2, ALARA_HIGHPOWER_ANALOGREAD3, ALARA_HIGHPOWER_ANALOGREAD4, ALARA_HIGHPOWER_ANALOGREAD5, ALARA_HIGHPOWER_ANALOGREAD6, ALARA_HIGHPOWER_ANALOGREAD7, ALARA_HIGHPOWER_ANALOGREAD8, ALARA_HIGHPOWER_ANALOGREAD9, ALARA_HIGHPOWER_ANALOGREAD10}};

#endif