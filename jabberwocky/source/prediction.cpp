#include "servo_functions.h"
#include "sd_card.h"
#include "accelerometer_functions.h"
#include "barometer.h"
#include "gps.h"
#include <math.h>
#include <stdint.h>
#include <Arduino.h>

#define g 32.174    // gravity, ft/s^2
#define CDCLOSED 0.675  //The coefficient of drag when the air brakes are closed
#define AREACLOSED 0.19634  //The area of the rocket when the air brakes are closed
#define FINALHEIGHT 5280.0  //Final height we want rocket to reach at apogee, in ft
#define MASS 46.6   //Mass of the rocket in lb (without fuel), SUBJECT TO FREQUENT CHANGE <---------------------

extern int16_t accY;
extern double altitude;
extern short pos;

bool burnout = false; //current status of motor (false = motor active)
bool brake = false; //status of the brakes (false = closing, true = opening)

//These are not all on the same line to comment them
double AxPrev = 0;      // previous x acceleration
double PnextAx = 0;     // prediction of next x acceleration for kalman filter, used exclusively in the kalman function
double PnextALT = 0;    // prediction of next altitude for kalman filter, used exclusively in the kalman function
double altPrev = 0;     // saved altitude from previous loop
double altRefine = 0;   // smoothed altitude
double correction = 0;
double Ax = 0;          // smoothed vertical acceleration
double velocity = 0;    // current velocity
double maxHeight = 0; //current max height the rocket has reached in current flight
double projHeight = 0;

unsigned long currentTime=0;     // current time(ms), used for integration and derivation
unsigned long OldTime=0;  // time from previous loop(ms)

double Kalman(double UnFV,double FR1,double *Pold){

/* function filtering results in real time with Kalman filter */

  char A=1,un=0,H=1,B=0;
  double Prediction,P,y,S,K,FR2;
  float Q = 0.1, R = 0.2;

    //Q=0.1;
   // R=0.2;      // defined these from the Matlab code
  // inputs are Unfiltered Response, Filtered  response, Prediction value old
  // make sure to predefine P and possibly other values at the beginning of the main code. starts at 1

  //UnFV    // unfiltered value from sensors read in (zn)           -internal logic
  //FR1       // read this variable in from main (AX(i)) defined last iteration -internal logicexc
  //Pold      // output Pold and read in Pold from last iteration         -input,output
  //Q     // sensor specific value, we might default this           -input
  //R     // sensor specific value, we might default this           -input
  //A         
  //FR2
  
  Prediction= A*FR1 +B*un;    // State Prediction     (Predict where we're goning to be)
  P=A*(*Pold)*A +Q;           // Covariance Prediction  (Predict how much error)
  y=UnFV-H*Prediction;        // Innovation       (Compare reality against prediction)
  S= H*P*H +R;                // innovation Covariance  (Compare real error against prediction)
  K=(P*H)/S;                  //Kalman Gain         (Moderate the prediction)
  FR2=Prediction+K*y;         //state update      (New estimate of where we are)
  *Pold=(1-K*H)*P;              //Covariance update     (New estimate of error)
  
  return FR2;  
}


double ApogeePrediction(){

  //all heights are in ft
  //double projHeight;        //The projected final height with no air brakes at this moment in time
  //double k ;           //Drag to be used tp calculate extraHeight

  //Calcualtes projectedHeight
  double k = 0.5 * AREACLOSED * CDCLOSED * 0.0022; //from old code, check constants
  projHeight = (MASS / (2.0 * k)) * log(((MASS * g) +(k * velocity * velocity)) / (MASS * g)) + altRefine;

  //If projectedHeight will surpass desiredFinalHeight
  
  if (projHeight > (FINALHEIGHT + 100) ) //error of 50ft, pelican
  {
    brake = true;
  }
  else
  {
    brake = false;
  }
  ServoFunction();
}

double Integrate(unsigned long prevTime, unsigned long currTime, double Val) {

  /*function requires two times, and two data points*/

  //Val1        //input of initial value    -input
  //Val2        //input of secondary inital   -input
  //deltaT      //change in time        -internal logic
  //deltaA      //change in value       -internal logic
  //prevTime       //Previous time         -input
  //currTime       //time current          -input

  double deltaT;
  
  deltaT = (currTime-prevTime)/1000.0;      // computes deltaT
  return Val*deltaT; //computes and returns Area, the result of the integration
} 
  
double Derive(unsigned long OldTime, unsigned long currentTime, double altPrev, double altRefine){
  
  return (altRefine-altPrev)/( ((double) currentTime -(double) OldTime) / 1000);
}

void UpdateData(){
  
  /* Updates all global variables for calculations */
  
  currentTime = millis();
  GetAcc();
  GetAlt();

  altRefine = Kalman(altitude, altPrev, &PnextALT);
  Ax = Kalman(accY, AxPrev, &PnextAx);//pelican
  
  velocity += Integrate(OldTime, currentTime, Ax); //Integrating to get new velocity  
// ServoTest();

  OldTime=currentTime;   // ms, reassigns time for lower bound at next integration cycle (move this to top of loop to eliminate any time delays between time and oldTime to have a better integration and derivation?)
  // AxPrev=Ax;    // reassigns Ax for initial acceleration at next integration cycle  
  //aPrev = sqrt(accX*accX+accY*accY);
  // altPrev=altRefine;    // reassigns altRefine for initial altitude at next derivation
  
  if(altRefine > maxHeight)
    maxHeight = altRefine;
}

void Burnout(){

  /* Detects when the vertical acceleration is negative. Holds arduino hostage until it's satisfied*/
  
  //Ax    //Filtered vertical acceleration      //input
  //Burnout //False when motor is on, true afterwards //output
  
  OldTime = currentTime = millis();
  UpdateData();
  UpdateData();
  WriteData();
  correction = accY;
  UpdateData();
  while(Ax < 3){
    UpdateData();
    WriteData();
  }
  LogWrite(2);
  velocity = 0;
  while(!burnout){
    UpdateData();
    WriteData();
    if(Ax <= -(g-3.0) && altRefine > 0.5) //pelican //checks if vertical acc is <= ~ -30 && is over a set height
      burnout = true;
  }
  
  LogWrite(7);
  // Serial.println(F("leaving Burnout"));
}

void EndGame(){
  
  /* Checks if apogee has been reached or if the rocket is on a poor trajectory. 
  If so, the brakes will permanently close and data will be logged until end of flight*/

  if( ( (maxHeight > (altRefine+1) ) && (velocity < -2))){//pelican
    
    LogWrite(3);    //checks what to write to log file
    brake = false;
    ServoFunction(); //closes brakes since we set brake to false
    WriteData();
    
    while(true){        //brakes closed, flight data will be logged until computer is turned off
      UpdateData();
      WriteData();
      GPS_get_fix();
    }
  } else if(altRefine > 3){ //pelican
    LogWrite(6);
    brake = true;
    while(pos < 125){
      ServoFunction();
      WriteData();
      // delay(50);
    }
    while(true){
        UpdateData();
        WriteData();
        if( maxHeight > (altRefine+25) ){ //pelican
            LogWrite(3);    
            brake = false;
            ServoFunction(); 
    
            while(true){        //brakes closed, flight data will be logged until computer is turned off
                UpdateData();
                WriteData();
                GPS_get_fix();
            }
       }
    }
  }  
}
