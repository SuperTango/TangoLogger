#ifndef	DEFAULTS_H
#define	DEFAULTS_H

#ifdef __AVR_ATmega2560__
// Pins remapped to Arduino Mega 2560
#define P_MOSI          B,2 // pin 51
#define P_MISO          B,3 // pin 50
#define P_SCK           B,1 // pin 52
#define MCP2515_CS      B,0 // pin 53
#define MCP2515_INT     E,4 // pin 2
#define LED2_HIGH       H,5 // pin 8
#define LED2_LOW        H,5 // pin 8

#else

// Atmega 168/328 pin mapping
#define   P_MOSI      B,3 // pin 11 // REV 
#define   P_MISO      B,4 // pin 12
#define   P_SCK       B,5 // pin 13
//#define MCP2515_CS	D,3 // Rev A
#define   MCP2515_CS  B,2 // pin 10, Rev B
#define   MCP2515_INT D,2 // pin 2
#define   LED2_HIGH   B,0 // pin 8
#define   LED2_LOW    B,0 // pin 8

#endif __AVR_ATmega2560__

#endif	// DEFAULTS_H



