#include "accelerometer_functions.h"
#include "barometer.h"
#include "servo_functions.h"
#include "sd_card.h"
#include "prediction.h"
#include "gps.h"
#include <I2Cdev.h>   // provides simple and intuitive interfaces to I2C devices
#include <SPI.h>    // Serial Peripheral Interface(SPI) used for communicating with one or more peripheral devices quickly over short distances

extern bool burnout;

void setup(){
  SerialSetup();
  MpuSetup();     
  ms5611Setup();
  ServoSetup();
  SDcardSetup();
  SDcardWriteSetup();    
  GpsSetup();
  Burnout();
}

void loop() { // run code (main code) 

  UpdateData();

  if(burnout){
    ApogeePrediction();
  }

  EndGame();
  
  WriteData();
}

//Serial Setup
void SerialSetup(){
 Serial.begin(9600);
}
