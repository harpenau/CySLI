#include <SD.h>

File dataFile;      // the datafile variable to save stuff to microSD

extern unsigned long currentTime;
extern double altitude;
extern double altRefine;
extern int16_t accX;
extern int16_t accY;
extern int16_t accZ;
extern short pos;
extern double Ax;
extern double velocity;
extern double projHeight;

void SDcardSetup(){
    /*Set up sd card to read RC data*/
      
    pinMode(53, OUTPUT);
    SD.begin(53);  
}

void SDcardWriteSetup(){
    dataFile = SD.open("Data.txt", FILE_WRITE);
    dataFile.println(F("Time(ms),F Alt(ft),AccY(ft/s^2), F Acc(ft/s^2),zVel(ft/s)"));
    dataFile.close(); 
}

void WriteData(){
 dataFile = SD.open("Data.txt", FILE_WRITE);
if(dataFile)
    Serial.println(F("file successfully opened"));
    dataFile.print(currentTime);
    dataFile.print(",");
    dataFile.print(altitude);
    dataFile.print(",");
    dataFile.print(altRefine);
    dataFile.print(",");
    dataFile.print(accX);  
    dataFile.print(",");
    dataFile.print(accY);
    dataFile.print(",");
    dataFile.print(accZ);
    dataFile.print(",");
    dataFile.print(pos - 35);
    dataFile.print(",");
    dataFile.print(Ax);
    dataFile.print(",");
    dataFile.println(velocity);
    dataFile.print(","); 
    dataFile.println(projHeight);
    dataFile.close();
}

void LogWrite(short reason){
  
    /*Called to write important checkpoints during flight*/
    
    //reason  //determines which event to write to log  //input
    
    dataFile = SD.open("Data.txt", FILE_WRITE);
    switch(reason){
        case 1: dataFile.println(F("LAUNCH DETECTED"));
            break;
        case 2:dataFile.println(F("LAUNCH DETECTED"));
            break;
        case 3:dataFile.println(F("FREEFALL DETECTED"));
            break;
        case 4:dataFile.println(F("BRAKE OPENED 5 DEG"));
            break;
        case 5: dataFile.println(F("BRAKE CLOSED"));
            break;
        case 6: dataFile.println(F("TARGET APOGEE REACHED, VEL > 0, BRAKING UNTIL FREEFALL"));
            break;
        case 7: dataFile.println(F("MOTOR BURNOUT"));
            break;
        default: 
            break;
      }
      dataFile.close();
}