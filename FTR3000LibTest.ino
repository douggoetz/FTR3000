
//Libraries to include in StratoDIB or Main
 #include <Ethernet.h>
#include <FTR3000.h>



#define FOTSSwitch 2
#define EFUSwitch 3

////Initialization Commands:
//byte GetStatus[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};  
//byte StopMeasure[] = { 0x00, 0x00, 0x00, 0x06, 0x00, 0x05, 0x04, 0x01, 0x00, 0x00 };
//byte FirmVersion[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x09, 0x09, 0x00}; //
//byte SerialNum[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x1F, 0x1F, 0x00}; //
//byte GetWaveNum[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x2b, 0x2b, 0x00}; //
//byte GetCalConst[] = {0x00, 0x00, 0x00,0x04,0x00,0x2C, 0x2C, 0x00}; //
//byte GetPulse[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x06, 0x06, 0x00}; //
//byte SetBrokeThres[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0xF0, 0xEA, 0x01, 0x00, 0x05}; //arbitrarily set to 5 db
//
////Initialize Fiber:
//byte SetFiberLen[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0xF4, 0x1C, 0x01, 0x07, 0xD0}; //set to 2000m
//byte SetStartPos[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0xEF, 0xEE, 0x01, 0x00, 0x00};
//byte SetLossCal[] = {0x00, 0x00, 0x00, 0x08, 0x00, 0x2c, 0x2a, 0x01, 0x00, 0x00, 0x00, 0x01}; //set to 1 db/km
//byte SetPulse[] = {0x00, 0x00, 0x00, 0x08, 0x00, 0x0D, 0x06, 0x01, 0x00, 0x03, 0x00, 0x03}; //set to 40us
//byte SetFibAlarmRange[] = {0x00, 0x00, 0x00, 0x08, 0x00, 0xE3, 0x0B, 0x01, 0x00, 0x00, 0x07, 0xD0}; // set alarm range from 0 to 2000m
////byte SetAvgTimeHW[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0x13, 0x12, 0x01, 0x00, 0x01}; // set averaging time to hardware avg
//byte SetAvgTimeSW[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0x19, 0x12, 0x01, 0x00, 0x06}; //Set averaging time to software avg (2^21)
//
//// Start Measurement Commands:
//byte SetContMeasure[] = {0x00, 0x00, 0x00, 0x06, 0x00, 0x07, 0x04, 0x01, 0x00, 0x02}; // set measurement type to continious
//byte ReadRaman[] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x02, 0x00}; //read raman data with total buffer size = 16390 byte




EthernetClient client;
FTR ftr(&client, &Serial);


byte RamanBin[17000];
uint16_t StokesTemp[1750]; //Stokes array that is to contain insertion loss corrected data
uint16_t AstokesTemp[1750]; //Astokes array that is to contain insertion loss corrected data for SAS ratio
int Elements = 1750;
uint16_t StokesElements[1750];
float StokesAvg[1750];
uint8_t AveCount = 0;


int counter;
int lastcount;
uint8_t statbin[14];
byte statbyte;




void setup() {

  
  Serial.begin(115200);
  delay(50);

  //SPI.begin();
  //Serial.println("SPI Configured: ");

  //Fiber Switch
  pinMode(FOTSSwitch, OUTPUT);
  pinMode(EFUSwitch, OUTPUT);

  digitalWrite(FOTSSwitch, LOW);
  digitalWrite(EFUSwitch,LOW);

  digitalWrite(FOTSSwitch, HIGH);
  delay(10);
  digitalWrite(FOTSSwitch, LOW);

  //uint16_t config_err;
  ftr.SwitchFTRon();
  ftr.start();

  delay(10000);
  //ftr.EthernetConnect();

  if(ftr.EthernetConnect()){
    Serial.println("initialized");
    //ftr.InitializeFTR();
    
  }
  
  delay(20);
  
  counter = millis();
  lastcount = counter;


  
}

void loop() {

  

  if (counter - lastcount >= 20000) { //do 2 minute requests for Raman data

    lastcount = counter;
    resetFtrSpi();
    statbyte = ftr.status();

    Serial.println(statbyte,HEX);

    if(statbyte != 0x27){
    
    ftr.readRaman(RamanBin);
    ftr.BintoArray(RamanBin, StokesTemp, AstokesTemp, Elements);
    AveCount += ftr.RamanCoAdd(StokesElements, StokesTemp, StokesAvg, Elements);
    
      if(AveCount == 5){
        ftr.RamanAverage(StokesElements, StokesAvg, StokesTemp, Elements);
        PrintArrays();
        PrintArrays2();
        AveCount = 0;
        ftr.ClearArray(StokesElements, Elements);
        ftr.ClearArray(StokesAvg, Elements);
      }

    Serial.println(AveCount);
        
        for (int i = 0; i < 17000; i = i + 1) {

      Serial.print(RamanBin[i],HEX);
      Serial.print(",");
      //RamanBin[i] = (char)0;
      if(i == 16999){
      Serial.println("");        
      }    
    }  
    PrintArrays();
   // ftr.ClearArray(RamanBin);
   // ftr.ClearArray(StokesTemp);
   // ftr.ClearArray(AstokesTemp);
   
  }
  }

  counter = millis();
}


void FTRBintoArray(){
  
  Serial.println("Enter Bin to Array");
  int indexCounter = 0;
  
  for (int i = 20; i < 8020; i = i + 4) {

    if(indexCounter<2100){
      StokesTemp[indexCounter] = ((int)RamanBin[i] * 256) + (int)RamanBin[i + 1]; //convert binary to int for math operations **typical value ~59000 dBm*1000**
      AstokesTemp[indexCounter] = ((int)RamanBin[i + 2] * 256) + (int)RamanBin[i + 3];
      indexCounter++;  
    }
  }

}  


void PrintArrays(){

  int arraypnts = sizeof(StokesAvg) / sizeof( StokesAvg[0] );
  Serial.println(String(arraypnts));
  for(int i = 0; i<arraypnts; i++){
      Serial.print(StokesAvg[i]);  //print all data to serial
      Serial.print("," );
      //Serial.print(AstokesTemp[i]);
      //Serial.print(", ");
  
  }

  Serial.println("");

}

void PrintArrays2(){

  int arraypnts = sizeof(StokesTemp) / sizeof( StokesTemp[0] );
  Serial.println(String(arraypnts));
  for(int i = 0; i<arraypnts; i++){
      Serial.print(StokesTemp[i]);  //print all data to serial
      Serial.print("," );
      //Serial.print(AstokesTemp[i]);
      //Serial.print(", ");
  
  }

  Serial.println("");

}


void resetLtcSpi() {
  SPI0_SR |= SPI_DISABLE;
  SPI0_CTAR0 = 0x38004005;
  SPI0_SR &= ~(SPI_DISABLE);
}

void resetFtrSpi() {
  SPI0_SR |= SPI_DISABLE;
  SPI0_CTAR0 = 0xB8020000;
  SPI0_SR &= ~(SPI_DISABLE);
}
   
    

