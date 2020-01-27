#include "sd_card.h"
#include <ServoTimer2.h>

ServoTimer2 servo;        //airbrake servo

extern bool brake;

short pos=35;      // postition of the servo, defaults to closed position (degrees)

//            Adjust this to the current settings?
void ServoSetup(){
  servo.attach(47);
  servo.write(pos);
}
void ServoFunction(){
  
  /*Function that will close and open brakes. If brake a true, 
    we want brakes to open. If false, brakes will closed. */
    
  if(brake){
      if(pos < 125){
        LogWrite(4);
        pos += 5;  //opens brakes 10 more degrees
        servo.write(pos);
      }
  } else if (pos > 35){
      LogWrite(5);
      pos = 35;     //closes brakes by 10 deg
      servo.write(pos);
  }
  
}
