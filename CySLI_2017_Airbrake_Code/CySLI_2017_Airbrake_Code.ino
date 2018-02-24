// formatting rules:
// 1) { on same line as statement
//   ex: if (condition) {
// 2) multicomment lines go above line
//  ex: // comment
//      // comment continued
//      variable = 0;
// 3) single line comments on same line, tabbed and spaced
//  ex: variable = 0; // comment
// 4) for function:what it does, name inputs and outputs with definition
//  ex: see ApogeePrediction()
// 5) variables have camel case starting with lowercase
//  ex: helloWorld = 0;
// 6) functions have camel case starting with uppercase
//  ex: HelloWorld()

#include <I2Cdev.h>   // provides simple and intuitive interfaces to I2C devices
#include <MPU6050.h>  // IMU library
#include <Wire.h>   // allows you to communicate with I2C / TWI devices
#include <Servo.h>    // servo library
#include <SPI.h>    // Serial Peripheral Interface(SPI) used for communicating with one or more peripheral devices quickly over short distances
#include <SD.h>     // microSD card library
#include <MS5611.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>

#define g 32.174    // ft/s^2
#define CDCLOSED 0.675  //The coefficient of drag when the air brakes are closed
#define AREACLOSED 0.19634  //The area of the rocket when the air brakes are closed
#define FINALHEIGHT 5280.0  //Final height we want rocket to reach at apogee, in ft
#define MASS 1.448   //Mass of the rocket in lb (without fuel), SUBJECT TO FREQUENT CHANGE <---------------------

short pos=35;      // postition of the servo (degrees?)

bool burnout = false; //current status of motor (false = motor active)
bool brake = false; //status of the brakes (false = closed, true = open)

//These are not all on the same line to comment them
double AxPrev = 0;    // previous x acceleration
double PnextAx = 0;   // prediction of next x acceleration for kalman filter, used exclusively in the kalman function
double PnextALT = 0;    // prediction of next altitude for kalman filter, used exclusively in the kalman function
double altPrev = 0;     // saved altitude from previous loop
double altRefine = 0;   // smoothed altitude
double Ax = 0;      // smoothed x acceleration
double velocity = 0;    // current velocity
//double oldVelocity = 0;   // saved velocity from previous loop
double baseline;    // baseline pressure
double maxHeight = 0; //current max height the rocket has been at
double aPrev = 0;
double totalv = 0;
double projHeight = 0;
unsigned long time=0;     // current time(stationary once called?), used for integration and derivation
unsigned long OldTime=0;  // time at end of loop (stationary once called?)

static NMEAGPS  gps;
static gps_fix  fix;

//Objects
MS5611 ms5611;

MPU6050 mpu;
Servo servo;
File dataFile;      // the datafile variable to save stuff to microSD

//MPU 6050 Accelerations
int16_t accX, accY, accZ;   // unfiltered accelerations, assigns 16 bit signed integers, used for balence of precision and speed

//ms5611180 Altitude
double altitude;  

void setup(){
  SerialSetup();
  MpuSetup();     
  ms5611Setup();
  ServoSetup();
  SDcardSetup();
  SDcardWriteSetup();     //setup sd card to write data to
  GpsSetup();
  Burnout();
}

void loop() { // run code (main code) 

  UpdateData();

  if(burnout){
    ApogeePrediction();
  }

  EndGame();
  //GpsLoop();
  WriteData();
}
//--------------------------------------------------------------Beginning of the Functions---------------------------------------------

//------------------------------------------------------------------GPS Functions------------------------------------------------------
void GpsSetup(){
  DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT);

  DEBUG_PORT.print( F("NMEA.INO: started\n") );
  DEBUG_PORT.print( F("  fix object size = ") );
  DEBUG_PORT.println( sizeof(gps.fix()) );
  DEBUG_PORT.print( F("  gps object size = ") );
  DEBUG_PORT.println( sizeof(gps) );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );

  #ifndef NMEAGPS_RECOGNIZE_ALL
    #error You must define NMEAGPS_RECOGNIZE_ALL in NMEAGPS_cfg.h!
  #endif

  #ifdef NMEAGPS_INTERRUPT_PROCESSING
    #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
  #endif

  #if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
      !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
      !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
      !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST )

    DEBUG_PORT.println( F("\nWARNING: No NMEA sentences are enabled: no fix data will be displayed.") );

  #else
    if (gps.merging == NMEAGPS::NO_MERGING) {
      DEBUG_PORT.print  ( F("\nWARNING: displaying data from ") );
      DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
      DEBUG_PORT.print  ( F(" sentences ONLY, and only if ") );
      DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
      DEBUG_PORT.println( F(" is enabled.\n"
                            "  Other sentences may be parsed, but their data will not be displayed.") );
    }
  #endif

  DEBUG_PORT.print  ( F("\nGPS quiet time is assumed to begin after a ") );
  DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
  DEBUG_PORT.println( F(" sentence is received.\n"
                        "  You should confirm this with NMEAorder.ino\n") );

  trace_header( DEBUG_PORT );
  DEBUG_PORT.flush();

  gpsPort.begin( 9600 );
}

static void doSomeWork(){
  // Print all the things!

  trace_all( DEBUG_PORT, gps, fix );

} // doSomeWork

static void GPSloop(){
  while (gps.available( gpsPort )) {
    fix = gps.read();
    doSomeWork();
  }
}
//-----------------------------------------------------------------SD Card Functions---------------------------------------------------
void SDcardSetup(){
  /*Set up sd card to read RC data*/
  
  pinMode(53, OUTPUT);
    SD.begin(53);  
}

void SDcardWriteSetup(){
  dataFile = SD.open("Data.txt", FILE_WRITE);
  dataFile.println(F("Time(ms),Height(ft),F Alt(ft),AccX(ft/s^2),AccY(ft/s^2),AccZ(ft/s^2),Brake Angle, F Acc(ft/s^2),SlopeVel(ft/s),totalv,ap"));
  dataFile.close(); 
}

void WriteData(){
 dataFile = SD.open("Data.txt", FILE_WRITE);
// if(dataFile)
//      Serial.println(F("file successfully opened"));
 dataFile.print(time);
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
 dataFile.print(velocity);
 dataFile.print(",");
 dataFile.print(totalv);
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
    case 2:dataFile.println(F("ABNORMAL FLIGHT DETECTED"));
      break;
    case 3:dataFile.println(F("FREEFALL DETECTED"));
      break;
    case 4:dataFile.println(F("BRAKE OPENED 10 DEG"));
      break;
    case 5: dataFile.println(F("BRAKE CLOSED 10 DEG"));
      break;
    case 6: dataFile.println(F("TARGET APOGEE REACHED, VELOCITY > 0, BRAKING UNTIL FREEFALL"));
      break;
    case 7: dataFile.println(F("MOTOR BURNOUT"));
      break;
    default: break;
  }
  dataFile.close();
}

//Serial Setup
void SerialSetup(){
 Serial.begin(9600);
}

//-----------------------------------------------------------Servo Methods--------------------------------------------------------------                                   
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
        pos += 10;  //opens brakes 10 more degrees
        servo.write(pos);
      }
  } else if (pos > 35){
      LogWrite(5);
      pos -=10;     //closes brakes by 10 deg
      servo.write(pos);
  }
  
}
//-------------------------------------------------------------------ms5611 180 Methods----------------------------------------------------

void ms5611Setup(){
  ms5611.begin();
  baseline = ms5611.readPressure();
}

double GetPressure(){
  return ms5611.readPressure();
}

void GetAlt(){
double a,P;
  
  // Get a new pressure reading:
  P = ms5611.readPressure();

  // Show the relative altitude difference between
  // the new reading and the baseline reading:
  a = ms5611.getAltitude(P,baseline);
  
  //convert to ft
  altitude =a*3.28084; 
  }

void Resetms5611(){
  baseline = ms5611.readPressure();
}

//--------------------------------------------------------------------MPU 6050 Methods--------------------------------------------------
void GetAcc(){
   mpu.getAcceleration(&accX, &accY, &accZ);
   accX = map(accX, 0, 4096, 0, 32);
   accY = map(accY, 0, 4096, 0, 32); //if y is pointing up (or usb port) subtract 31
   accZ = map(accZ, 0, 4096, 0, 32)-31.0; //pelican?
}

void MpuSetup(){
    //joins I2C bus (I2Cdev library doesn't do this automatically) (Was found in available rocket code on internet) 
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
    mpu.setFullScaleAccelRange(2);  //0=2g, 1=4g, 2=8g, 3=16g
    mpu.setFullScaleGyroRange(2); //0=250 deg/s, 1=500 deg/s, 2=1000 deg/s, 3=2000 deg/s 
//    calibrateMPU();                                    // <------------------------does this exist?
}

//------------------------------------------------------------IMPORTANT FUNCTIONS-------------------------------------------------------
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
  Serial.print("AP:"); Serial.print(projHeight);
  if (projHeight > FINALHEIGHT) // + ERROR)
  {
    brake = true;
  }
  else
  {
    brake = false;
  }
  ServoFunction();
}

double Integrate(unsigned long prevTime, unsigned long currTime, double Val1, double Val2) {

  /*function requires two times, and two data points*/

  //Val1        //input of initial value    -input
  //Val2        //input of secondary inital   -input
  //deltaT      //change in time        -internal logic
  //deltaA      //change in value       -internal logic
  //prevTime       //Previous time         -input
  //currTime       //time current          -input

  double deltaT, deltaA;
  
  deltaT = (currTime-prevTime)/1000.0;      // computes deltaT
 // deltaA = (Val2+Val1)/2;    // computes deltaA
  deltaA=(Val2-Val1);
  return deltaT*deltaA; //computes and returns Area, the result of the integration
} 
  
double Derive(unsigned long OldTime, unsigned long time, double altPrev, double altRefine){
  
  return (altRefine-altPrev)/( ((double) time -(double) OldTime) / 1000);
}

void Burnout(){

  /* Detects when the vertical acceleration is negative. Holds arduino hostage until it's satisfied*/
  
  //Ax    //Filtered vertical acceleration      //input
  //Burnout //False when motor is on, true afterwards //output
  OldTime = time = millis();
  UpdateData();
  UpdateData();
  UpdateData();
  UpdateData();
  UpdateData();
  UpdateData();
  UpdateData();
  velocity = 0;
  
  while(!burnout){
    UpdateData();
    WriteData();
    if(Ax <= -(g-16.0) && altRefine > 0.5) //pelican //checks if vertical acceleration is <= ~ -30 && is over a set height
      burnout = true;
  }
  
  LogWrite(7);
  Serial.println(F("leaving Burnout"));
}

void EndGame(){
  
  /* Checks if apogee has been reached or if the rocket is on a poor trajectory. 
  If so, the brakes will permanently close and data will be logged until end of flight*/

  if( ( (maxHeight > (altRefine+0.3) ) && (velocity < -0.5) && burnout) ){ //|| ( abs(accX)>=27  || abs(accY)>=27) ){//pelican
    if(maxHeight > (altRefine+0.5) && (velocity < -0.5) ){  //pelican
      LogWrite(3);    //checks what to write to log file
    }else{
      LogWrite(2);
    }
    
    brake = false;
    while(pos > 35){
      ServoFunction(); //closes brakes since we set brake to false
    }
    
    while(true){        //brakes closed, flight data will be logged until computer is turned off
      UpdateData();
      WriteData();
      GPSloop();
    }
  } else if(altRefine > 5200){ //pelican
    LogWrite(6);
    brake = true;
    while(pos < 125){
      ServoFunction();
    }
  }  
}

void UpdateData(){
  
  /* Updates all global variables for calculations */
  
  time = millis();
  GetAcc();
  GetAlt();

  altRefine = Kalman(altitude, altPrev, &PnextALT);
  Ax = Kalman(accZ, AxPrev, &PnextAx);//pelican
  
  velocity += Integrate(OldTime, time, AxPrev, Ax); //Integrating to get new velocity  
  totalv += Integrate(OldTime,time, aPrev, sqrt(Ax*Ax+accX*accX+accY*accY));
  
  if(pos == 35)
    brake = true;
  if(pos == 125)
    brake = false;
  
  ServoFunction();
  delay(500);
  ServoFunction();
  delay(500);

//   Serial.print("time: "); Serial.print(time); Serial.print("  Altrefine:"); Serial.print(altRefine);
  //Serial.print("   Ax:"); Serial.println(Ax, 4); 
//  Serial.print("   velocity:"); Serial.print(velocity, 4);
//  Serial.print(  "totalv: "); Serial.println(totalv, 4);
  
  OldTime=time;   // ms, reassigns time for lower bound at next integration cycle (move this to top of loop to eliminate any time delays between time and oldTime to have a better integration and derivation?)
  AxPrev=Ax;    // reassigns Ax for initial acceleration at next integration cycle  
  aPrev = sqrt(Ax*Ax+accX*accX+accY*accY);
  //oldVelocity = velocity; // still need the now old velocity to find position at that time 
  altPrev=altRefine;    // reassigns altRefine for initial altitude at next derivation
  
  if(altRefine > maxHeight)
    maxHeight = altRefine;

}
