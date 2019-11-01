/*
 *  FTR3000.c
 *  Communication functions and macros for the FTR3000
 *  Author: Marika Schubert
 *  November 2017
 *  
 *  This library is intended for use on the FLOATS
 *  FSW for the Zephyr Balloon Module. It will enable the 
 * 	user to cleanly communicate with the FTR3000 module
 *
 *  This code has the following dependencies:
 *  Ethernet.h
 *	SPI.h
 *  
 */
 
// Includes
#include "FTR3000.h"

 
// Defines
#define DEBUG
  
// Global Variables

String RamanHex;
byte RamanBytes;
int RamanInt;


//Initialization Commands:
byte GetStatus[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};  
byte StopMeasure[] = { 0x00, 0x00, 0x00, 0x06, 0x00, 0x05, 0x04, 0x01, 0x00, 0x00 };
byte FirmVersion[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x09, 0x09, 0x00}; //
byte SerialNum[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x1F, 0x1F, 0x00}; //
byte GetWaveNum[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x2b, 0x2b, 0x00}; //
byte GetCalConst[] = {0x00, 0x00, 0x00,0x04,0x00,0x2C, 0x2C, 0x00}; //
byte GetPulse[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x06, 0x06, 0x00}; //
byte SetBrokeThres[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0xF0, 0xEA, 0x01, 0x00, 0x05}; //arbitrarily set to 5 db

//Initialize Fiber:
byte SetFiberLen[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0xF4, 0x1C, 0x01, 0x07, 0xD0}; //set to 2000m
byte SetStartPos[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0xEF, 0xEE, 0x01, 0x00, 0x00};
byte SetLossCal[] = {0x00, 0x00, 0x00, 0x08, 0x00, 0x2c, 0x2a, 0x01, 0x00, 0x00, 0x00, 0x01}; //set to 1 db/km
byte SetPulse[] = {0x00, 0x00, 0x00, 0x08, 0x00, 0x0D, 0x06, 0x01, 0x00, 0x03, 0x00, 0x03}; //set to 40us
byte SetFibAlarmRange[] = {0x00, 0x00, 0x00, 0x08, 0x00, 0xE3, 0x0B, 0x01, 0x00, 0x00, 0x07, 0xD0}; // set alarm range from 0 to 2000m
//byte SetAvgTimeHW[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0x13, 0x12, 0x01, 0x00, 0x01}; // set averaging time to hardware avg
byte SetAvgTimeSW[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0x19, 0x12, 0x01, 0x00, 0x06}; //Set averaging time to software avg (2^21)

// Start Measurement Commands:
byte SetContMeasure[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0x07, 0x04, 0x01, 0x00, 0x02}; // set measurement type to continious
byte ReadRaman[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x02, 0x00}; //read raman data with total buffer size = 16390 byte


byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = { 10, 10, 10, 03 };
byte server[] = { 10, 10, 10, 02 };



// Function Implementations
	FTR::FTR(EthernetClient * client, Stream * serial)
	{
		_client = client;
        _serial = serial;
        //Pin macros are caller's responsibility
        ////FTR Power
        pinMode(FTR_PWR, OUTPUT); //Set FTR power pin to output
        //SwitchFTRon(); 
  
        ////Ethernet module power on reset
        pinMode(WizReset, OUTPUT);
        resetWIZ820io();
	}

	void FTR::FTRConfigure(){
		int i;
		char d[16];
		char x[20];

		_client->write(GetStatus,8); //Get instrument status
		_client->flush();
		//char d[16];
		_client->readBytes(d,14);

		Serial.print("Status string:");
	
		for (i = 0; i < 14; i = i + 1) {
		if(i==13){
			Serial.println(d[i], HEX);
		}
		else{
			Serial.print(d[i], HEX);
			Serial.print(",");
			}   
		}
		
	
		_client->write(StopMeasure,10); //send stop measure command
		_client->flush();
		_client->readBytes(x,10);
		Serial.print("Stop Measure String:");
		for (i = 0; i < 10; i = i + 1) {
			if(i==9){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				}   
			}
		
		delay(100);

			_client->write(GetStatus,8); //Get instrument status
			_client->flush();
			//char d[16];
			_client->readBytes(d,14);

			Serial.print("Status string:");
		
			for (i = 0; i < 14; i = i + 1) {
			if(i==13){
				Serial.println(d[i], HEX);
			}
			else{
				Serial.print(d[i], HEX);
				Serial.print(",");
				}   
			}

		_client->write(FirmVersion,8); //request firmware num
		_client->flush();
		_client->readBytes(x,20);

		Serial.print("Firmware:");
			for (i = 0; i < 20; i = i + 1) {
			if(i==19){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				}   
			}
		delay(100);

		_client->write(SerialNum,8); //request Serial num
		_client->flush();
		_client->readBytes(x,14);
		Serial.print("Serial Number:");
			for (i = 0; i < 14; i = i + 1) {
			if(i==13){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				}   
			}
		delay(100);

		_client->write(GetWaveNum,8); //Read shift wave num
		_client->flush();
		_client->readBytes(x,14);
		Serial.print("Wave Shift Number:");
			for (i = 0; i < 14; i = i + 1) {
			if(i==13){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				}   
			}
		delay(100);  

		_client->write(GetCalConst,8); //read CD
		_client->flush();
		_client->readBytes(x,14);
		Serial.print("Cal Constant:");
			for (i = 0; i < 14; i = i + 1) {
			if(i==13){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				}   
			}
		delay(100);  

		_client->write(GetPulse,8); //read Pulse Parameter
		_client->flush();
		_client->readBytes(x,14);
		Serial.print("Pulse Parameter:");
			for (i = 0; i < 14; i = i + 1) {
			if(i==13){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				}   
			}
		delay(100);

		_client->write(SetBrokeThres,10); //send break threshold parameter
		_client->flush();
		_client->readBytes(x,10);
		Serial.print("Fiber Broken Threshold:");
			for (i = 0; i < 10; i = i + 1) {
			if(i==9){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				}   
			}

		delay(100);    

		_client->write(SetFiberLen,10); //send fiber len parameter
		_client->flush();
		_client->readBytes(x,10);
		Serial.print("Fiber Len Response:");
			for (i = 0; i < 10; i = i + 1) {
			if(i==9){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				}   
			}
		
		delay(100);


		_client->write(SetStartPos,10); //send fiber len parameter
		_client->flush();
		_client->readBytes(x,10);
		Serial.print("Start Position:");
			for (i = 0; i < 10; i = i + 1) {
			if(i==9){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				}   
			}
		
		delay(100);

		_client->write(SetLossCal,12);
		_client->flush();
		_client->readBytes(x,10);
		Serial.print("Loss Cal Response:");
			for (i = 0; i < 10; i = i + 1) {
			if(i==9){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				}   
			}
		
		delay(100);

		_client->write(SetPulse,12);
		_client->flush();
		_client->readBytes(x,10);
		Serial.print("Set Pulse Response:");
			for (i = 0; i < 10; i = i + 1) {
			if(i==9){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				}   
			}
		delay(100);

		_client->write(SetFibAlarmRange,12);
		_client->flush();
		_client->readBytes(x,10);
		Serial.print("Set Alarm Range:");
			for (i = 0; i < 10; i = i + 1) {
			if(i==9){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				} 
			}   
		delay(100);

		_client->write(SetAvgTimeSW, 10);
		_client->flush();
		_client->readBytes(x,10);
		Serial.print("Set Avg Time Response:");
			for (i = 0; i < 10; i = i + 1) {
			if(i==9){
				Serial.println(x[i], HEX);
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				} 
			}   
		
		delay(100);

		_client->write(SetContMeasure,10);
		_client->flush();
		_client->readBytes(x,10);
		Serial.print("Set Cont. Measure Response:");
		for (i = 0; i < 10; i = i + 1) {
			if(i==9){
				Serial.println(x[i], HEX);
				x[i] = (char) 0;
			}
			else{
				Serial.print(x[i], HEX);
				Serial.print(",");
				x[i] = (char) 0;
				} 
			}   
		delay(100);

		_client->write(GetStatus,8); //Get instrument status
		_client->flush();
		_client->readBytes(d,14);
		Serial.print("Status string:");
		for (i = 0; i < 14; i = i + 1) {
			if(i==13){
				Serial.println(d[i], HEX);
			// d[i] = (char) 0;
			}
			else{
				Serial.print(d[i], HEX);
				Serial.print(",");
				//d[i] = (char)0;
				}   
			}



}   
	
	void FTR::start(){
        ////Initialize ethernet port 
        SPI.begin(); //SPI0 for DIB rev B and C
		resetWIZ820io();
		resetFtrSpi();

        Ethernet.init(WizCS); //sets CS to pin 14 
		Ethernet.begin(mac, ip);

  		pinMode(WizCS, OUTPUT); 
  		digitalWrite(WizCS, LOW); //deselects Wiz820io from SPI
		
	}

	bool FTR::EthernetConnect(){

		uint8_t timeout = 0;

		while(!_client->connect(server,10001) & (timeout <= 60)){
             timeout++;
             _serial->print("Ethernet not connecting: ");
             _serial->println(timeout);
			 delay(100);
        }  

		if(_client->connect(server,10001)) {
    		_serial->println("Ethernet Connected");
			FTRConfigure();
			return true;
    		
  		}
		else{
    		_serial->println("Ethernet Connection Failed - Reboot");
			return false;
  		}

	}

	uint8_t FTR::status(){
		// Returns the alarm and status bytes (lower 4 bytes)
		uint8_t k;
		uint8_t data[14];
		uint8_t ret;
		_client->write(GetStatus,8); //Get instrument status
		_client->flush();
		k = _client->readBytes(data,14);
		if (k != 14){
			return 1;
		}

		ret = data[13];
		// for (k = 13; k > 12; k--){
		// 	ret |= data[]<<((k-10)*4);		 
		// }
		_serial->print("Stat:");
		_serial->println(ret,HEX);
		return ret;
	}
	
	void FTR::readRaman(byte * bin){
		//if (!bin){
		//	return 1;
		//}
		//if (isDataReady()){
		//uint32_t ret;
		_client->write(ReadRaman,8);
		_client->flush();
			//ret = _client->readBytes(bin,16400);
		_client->readBytes(bin,16400);
			//if(ret != 17000){
			//	return 1;
			//}
			//return ret;
		//}
		//return 1;
	}
	
    void FTR::resetSpi(){
        SPI0_SR |= SPI_DISABLE;
        SPI0_CTAR0 = 0xB8020000;
        SPI0_SR &= ~(SPI_DISABLE);
    }
    
    void FTR::disconnect(){
        _client->flush();
        _client->stop();
        #ifdef DEBUG
        if(!_client->connected()){
            Serial.println("Disconnected from FTR");
        } else {
            Serial.println("Disconnect failed");
        }
        #endif
        digitalWrite(WizCS, HIGH);
    }
    
    void FTR::SwitchFTRon(){
        digitalWrite(FTR_PWR, LOW); 
    }
    
    void FTR::SwitchFTRoff(){
        digitalWrite(FTR_PWR, HIGH); 
    }
    
    void FTR::resetWIZ820io(){
        digitalWrite(WizReset, LOW);
        delay(1000);
        digitalWrite(WizReset,HIGH);
        delay(1000);
    }

	void FTR::resetFtrSpi() {
        SPI0_SR |= SPI_DISABLE;
        SPI0_CTAR0 = 0xB8020000;
        SPI0_SR &= ~(SPI_DISABLE);
	} 


void FTR::BintoArray(byte * bin, uint16_t * stokes, uint16_t * astokes, int arraylength){
  
	_serial->println("Enter Bin to Array");

	int BinLength = 17000;// (sizeof(bin)/sizeof(bin[0]));
	_serial->println(BinLength);
	int indexCounter = 0;

	for (int i = 20; i < (arraylength*4); i = i + 4) {
      //if(indexCounter<= sizeof(stokes)/sizeof(stokes[0])){
      stokes[indexCounter] = ((uint16_t)bin[i]*256) + (uint16_t)bin[i + 1]; //convert binary to int for math operations **typical value ~59000 dBm*1000**
      astokes[indexCounter] = ((uint16_t)bin[i + 2]*256) + (uint16_t)bin[i + 3];
      indexCounter++;  
    }


  _serial->println("exit Bin to Array");

}


uint8_t FTR::RamanCoAdd(uint16_t * ElementsArray, uint16_t * RamanArray, float * AveArray, int NumElements){

	_serial->println("Enter Coadd");
	
	for (int i = 0; i < NumElements; i++){
		
		if(RamanArray[i]>10000 && RamanArray[i]<100000){	
			AveArray[i]+=RamanArray[i];
			ElementsArray[i]++;
		}
	}

	return 1;
}

void FTR::RamanAverage(uint16_t * ElementsArray, float * AveArray, uint16_t * RamanArray, int NumElements){

	_serial->println("Enter Average");

	for(int i = 0; i < NumElements; i++){

		if(ElementsArray[i] != 0){
			AveArray[i]/=ElementsArray[i];
		}		
	}

	for(int i = 0; i < NumElements; i++){

		RamanArray[i] = (uint16_t)AveArray[i];

	}

}

void FTR::ClearArray(byte * bin, int NumElements){

	_serial->println("clearing bin array");
	
	for(int i = 0; i < NumElements; i++){	
		bin[i] = 0;
	}

}

void FTR::ClearArray(uint16_t * bin, int NumElements){

	_serial->println("clearing bin array");
	
	for(int i = 0; i < NumElements; i++){	
		bin[i] = 0;
	}
}

void FTR::ClearArray(float * bin, int NumElements){

	_serial->println("clearing bin array");
	
	for(int i = 0; i < NumElements; i++){	
		bin[i] = 0;
	}
}
 