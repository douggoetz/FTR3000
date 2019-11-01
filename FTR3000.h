/*
 *  FTR3000.h
 *  Communication functions and macros for the FTR3000
 *  Author: Marika Schubert
 *  November 2017
 *  Updated for new firmware September 2018
 *  
 *  This library is intended for use ONLY the FLOATS
 *  FSW for the Zephyr Balloon Module. It will enable the 
 * 	user to cleanly communicate with the FTR3000 module
 *
 *	The top level module using this file also needs to include
 *	<Ethernet.h> because of how Arduino handles global
 *	and class variables.
 *f
 *  This code has the following dependencies:
 *  Ethernet.h
 *	SPI.h
 *  
 */
 
 
 #ifndef FTR3000_H_
 #define FTR3000_H_
  
 // Dependencies
 #include <Ethernet.h>
 #include <SPI.h>
 #include "Arduino.h"
  
 // Defines
 //Bit masks for status registers
#define SPI_DISABLE (0x1<<30) 

// // Alarms
// #define RTC_ERR (1<<12)
// #define RST_ERR (1<<11)
// #define SD_ERR  (1<<8)
// #define PW_ERR (1<<5)
// #define LD_ERR (1<<4)
// #define APD_ERR (1<<3)
// #define TEMP_ERR (1<<2)
// #define FIBER_ERR (1)

// //Status
// #define DATA_ERR (1<<8)
// #define DATA_READY (1<<6)
// #define MEASURING (1<<5)
// #define CALCULATING (1<<4)
// #define SD_BUSY (1<<3)
// #define PS_WARN (5)
 
// //DIB Pins
// //Ethernet WizIO
#define WizCS 14
#define WizReset 18 //LOW = reset, pulse for 2 us
#define WizPWD 17 // power down for WizIO, Low = normal mode, HIGH = power down
#define WizINT 19 //interrupt 

#define FTR_PWR 26 
 
//Global Variables
 
class FTR
{
public:
	// Call to declare
	FTR(EthernetClient * client, Stream * serial);

	
    void FTRConfigure();
	void start();
	bool EthernetConnect();
	uint8_t status();
	void readRaman(byte * bin);
    // Run to set up SPI port with the correct settings
    void resetSpi();
    
    // Run to disconnect the FTR
    void disconnect();
    
    // Make sure FTR is powered
    void SwitchFTRon();
    
    // Make sure FTR is off
    void SwitchFTRoff();

	void resetWIZ820io(); 

	void resetFtrSpi();

	//void BintoArray(byte * bin, uint16_t * stokes, uint16_t * astokes);

	void BintoArray(byte * bin, uint16_t * stokes, uint16_t * astokes, int arraylength);

	//uint8_t RamanCoAdd(uint16_t * ElementsArray, uint16_t * RamanArray, uint16_t * AveArray); 

	uint8_t RamanCoAdd(uint16_t * ElementsArray, uint16_t * RamanArray, float * AveArray, int NumElements);
	
	//void RamanAverage(uint16_t * ElementsArray, uint16_t * AveArray);
	
	void RamanAverage(uint16_t * ElementsArray, float * AveArray, uint16_t * RamanArray, int NumElements);
	
	void ClearArray(byte * bin, int NumElements);
	
	void ClearArray(uint16_t * bin, int NumElements);

	void ClearArray(float * bin, int NumElements);
private: 
	EthernetClient * _client;
    
    Stream * _serial;
	
	uint8_t config_err; 
};
  
  
// Public functions

// Reset WIZ IO chip  
void resetWIZ820io(); 

 #endif