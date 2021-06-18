#pragma once
#include <stdint.h>
#include <macros.h>

#ifndef __PINS_ARDUINO__
#define __PINS_ARDUINO__

#define ANALOG_CONFIG

/* Analog reference options 
 * Different possibilities available combining Reference and Gain
 */
enum _AnalogReferenceMode
{
  AR_VDD,         // 3.3 V
  AR_INTERNAL,    // 0.6 V
  AR_INTERNAL1V2, // 1.2 V
  AR_INTERNAL2V4  // 2.4 V
};

/* Analog acquisition time options */
enum _AnalogAcquisitionTime
{
  AT_3_US,
  AT_5_US,
  AT_10_US, // Default value
  AT_15_US,
  AT_20_US,
  AT_40_US
};

// Frequency of the board main oscillator
#define VARIANT_MAINOSC (32768ul)

// Master clock frequency
#define VARIANT_MCK (64000000ul)

// Pins
// ----
// Number of pins defined in PinDescription array
#ifdef __cplusplus
extern "C" unsigned int PINCOUNT_fn();
#endif
#define PINS_COUNT (PINCOUNT_fn())
#define NUM_DIGITAL_PINS (48u)
#define NUM_ANALOG_INPUTS (6u)
#define NUM_ANALOG_OUTPUTS (0u)

// LEDs
// ----
#define PIN_LED (13u)
#define LED_BUILTIN PIN_LED

// Analog pins
// -----------
#define PIN_A0 (4u)
#define PIN_A1 (5u)
#define PIN_A2 (29u)
#define PIN_A3 (28u)
#define PIN_A4 (30u)
#define PIN_A5 (31u)
static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
#define PIN_PWM0 (6u)
#define PIN_PWM1 (16u)
static const uint8_t PWM0 = PIN_PWM0;
static const uint8_t PWM1 = PIN_PWM1;
#define PIN_BATTVIN3 (30u)
static const uint8_t BATTVIN3 = PIN_BATTVIN3;
#define ADC_RESOLUTION 12

// Digital pins
// -----------
#define D0 27
#define D1 40
#define G0 29
#define G1 3
#define G2 45
#define G3 44
#define G4 43
#define G5 17
#define G6 38
#define G7 36
#define G8 46
#define G9 9
#define G10 10

/*
 * Serial interfaces
 */
// Serial (EDBG)
#define PIN_SERIAL_RX1 (42ul)
#define PIN_SERIAL_TX1 (35ul)
//For serial flow control, uncomment lines 189 and 190
#define PIN_SERIAL_RTS1 (34ul)
#define PIN_SERIAL_CTS1 (41ul)

#define PIN_SERIAL_RX2 (37ul)
#define PIN_SERIAL_TX2 (39ul)

// SPI
#define PIN_SPI_CIPO (2u)
#define PIN_SPI_COPI (31u)
#define PIN_SPI_SCK (28u)
#define PIN_SPI_SS (20u)

// //NOTE: this is for onboard flash IC
// #define PIN_SPI_CIPO (21u)
// #define PIN_SPI_COPI (14u)
// #define PIN_SPI_SCK (19u)
// #define PIN_SPI_SS (12u)

static const uint8_t SS = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_COPI;
static const uint8_t MISO = PIN_SPI_CIPO;
static const uint8_t SCK = PIN_SPI_SCK;

#define PIN_SPI_CIPO1 (21u)
#define PIN_SPI_COPI1 (14u)
#define PIN_SPI_SCK1 (19u)
#define PIN_SPI_SS1 (32u)

static const uint8_t SS1 = PIN_SPI_SS1;
static const uint8_t MOSI1 = PIN_SPI_COPI1;
static const uint8_t MISO1 = PIN_SPI_CIPO1;
static const uint8_t SCK1 = PIN_SPI_SCK1;

#define PIN_QSPI_SCK 19
#define PIN_QSPI_CS 14
#define PIN_QSPI_IO0 21
#define PIN_QSPI_IO1 22
#define PIN_QSPI_IO2 23
#define PIN_QSPI_IO3 32

// Wire
#define PIN_WIRE_SDA (8u)
#define PIN_WIRE_SCL (11u)
#define PIN_WIRE_INT (15u)

#define PIN_WIRE_SDA1 (33u)
#define PIN_WIRE_SCL1 (24u)

// PDM Interfaces
// ---------------
#define PIN_PDM_PWR (46)
#define PIN_PDM_CLK (25)
#define PIN_PDM_DIN (26)

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL SerialUSB
#define SERIAL_PORT_MONITOR SerialUSB
#define SERIAL_PORT_HARDWARE Serial1
#define SERIAL_PORT_HARDWARE_OPEN Serial1

// Mbed specific defines
#define SERIAL_HOWMANY 2
#define SERIAL1_TX (digitalPinToPinName(PIN_SERIAL_TX1))
#define SERIAL1_RX (digitalPinToPinName(PIN_SERIAL_RX1))
//Uncomment the following two lines if you want to implement serial flow control
// #define SERIAL1_RTS (digitalPinToPinName(PIN_SERIAL_RTS1))
// #define SERIAL1_CTS (digitalPinToPinName(PIN_SERIAL_CTS1))
#define SERIAL2_TX (digitalPinToPinName(PIN_SERIAL_TX2))
#define SERIAL2_RX (digitalPinToPinName(PIN_SERIAL_RX2))

#define SERIAL_CDC 1
#define HAS_UNIQUE_ISERIAL_DESCRIPTOR
#define BOARD_VENDORID 0x2341
#define BOARD_PRODUCTID 0x805a
#define BOARD_NAME "Nano 33 BLE"

#define DFU_MAGIC_SERIAL_ONLY_RESET 0xb0

#define WIRE_HOWMANY 2

#define I2C_SDA (digitalPinToPinName(PIN_WIRE_SDA))
#define I2C_SCL (digitalPinToPinName(PIN_WIRE_SCL))
#define I2C_SDA1 (digitalPinToPinName(PIN_WIRE_SDA1))
#define I2C_SCL1 (digitalPinToPinName(PIN_WIRE_SCL1))

#define SPI_HOWMANY 2

#define SPI_MISO (digitalPinToPinName(PIN_SPI_CIPO))
#define SPI_MOSI (digitalPinToPinName(PIN_SPI_COPI))
#define SPI_SCK (digitalPinToPinName(PIN_SPI_SCK))

#define SPI_MISO1 (digitalPinToPinName(PIN_SPI_CIPO1))
#define SPI_MOSI1 (digitalPinToPinName(PIN_SPI_COPI1))
#define SPI_SCK1 (digitalPinToPinName(PIN_SPI_SCK1))

#define digitalPinToPort(P) (digitalPinToPinName(P) / 32)

uint8_t getUniqueSerialNumber(uint8_t *name);
void _ontouch1200bps_();

#endif //__PINS_ARDUINO__
