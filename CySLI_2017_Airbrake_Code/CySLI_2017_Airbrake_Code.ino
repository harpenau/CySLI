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
//#include <SFE_ms5611180.h> // pressure/temp sensor library
#include <Servo.h>    // servo library
#include <SPI.h>    // Serial Peripheral Interface(SPI) used for communicating with one or more peripheral devices quickly over short distances
#include <SD.h>     // microSD card library
#include <MS5611.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>

#define Ridk 1716     // (ft*lb)/(slug*degree_R)  //DIFFERENT NAME??????????????DELETE?????????-------------------------------
#define g 32.174    // ft/s^2
#define CDCLOSED 0.675  //The coefficient of drag when the air brakes are closed <-- VERIFY
#define AREACLOSED 3.4  //The area of the rocket when the air brakes are closed <--VERIFY
#define FINALHEIGHT 5280.0  //Final height we want rocket to reach at apogee, in ft
#define MASS 32.6   //Mass of the rocket in lb, SUBJECT TO FREQUENT CHANGE <---------------------

int pos=0;      // postition of the servo (degrees?)

bool burnout = false; //current status of motor (false = motor active)
bool brake = false; //status of the brakes (false = closed, true = open)

//These are not all on the same line to comment them
double AxPrev = 0;    // previous x acceleration
double PnextAx = 0;   // prediction of next x acceleration for kalman filter, used exclusively in the kalman function
double PnextALT = 0;    // prediction of next altitude for kalman filter, used exclusively in the kalman function
double altPrev = 0;     // saved altitude from previous loop
double altRefine;   // smoothed altitude
double Ax;      // smoothed x acceleration
double velocityNew;   // added or subtracted velocity since last loop
double velocity = 0;    // current velocity
double oldVelocity = 0;   // saved velocity from previous loop
double positionNew;   // integrated velocity since last loop
double position = 0;    // current position
double velocityms5611 = 0;   // velocity from ms5611, derived from position
double accelerationms5611 = 0;
double baseline;    // baseline pressure
double avPosition;
double avVelocity;
double avAcceleration;
  
unsigned long time=0;     // current time(stationary once called?), used for integration and derivation
unsigned long OldTime=0;  // time at end of loop (stationary once called?)

static NMEAGPS  gps;
static gps_fix  fix;

//Objects
MS5611 ms5611;

//SFE_ms5611180 ms5611;
MPU6050 mpu;
Servo servo;
File dataFile;      // the datafile variable to save stuff to microSD



//MPU 6050 Accelerations
int16_t accX, accY, accZ;   // unfiltered accelerations, assigns 16 bit signed integers, used for balence of precision and speed

//ms5611180 Altitude
double altitude;  

void setup(){
  MpuSetup();     
  ms5611Setup();
  SerialSetup();
  ServoSetup();
  SDcardSetup();
  SDcardWriteSetup();     //setup sd card to write data to
  Burnout();
  GpsSetup();
}

void loop() { // run code (main code) 

UpdateData();

if(burnout){
  ApogeePrediction();
}

EndGame();

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
} // GPSloop
//-----------------------------------------------------------------SD Card Functions---------------------------------------------------
void SDcardSetup(){
  /*Set up sd card to read RC data*/
  
  pinMode(4, OUTPUT);
    SD.begin(4);  
}

void SDcardWriteSetup(){
  dataFile = SD.open("Data.txt", FILE_WRITE);
  dataFile.println("Time(ms),Height(ft),F Alt(ft),AccX(ft/s^2),AccY(ft/s^2),AccZ(ft/s^2),F Acc(ft/s^2),SlopeVel(ft/s)");
  dataFile.close(); 
}

void WriteData(){
 dataFile = SD.open("Data.txt", FILE_WRITE);
 dataFile.print(millis());
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
 dataFile.print(brake);
 dataFile.print(",");
 dataFile.print(Ax);
 dataFile.print(",");
 dataFile.println(velocityms5611);
 dataFile.close();
}

void LogWrite(int reason){
  
  /*Called to write important checkpoints during flight*/
  
  //reason  //determines which event to write to log  //input
  
  dataFile = SD.open("Data.txt", FILE_WRITE);
  switch(reason){
    case 1: dataFile.println("LAUNCH DETECTED");
      break;
    case 2:dataFile.println("ABORT DETECTED");
      break;
    case 3:dataFile.println("FREEFALL DETECTED");
      break;
    case 4:dataFile.println("BRAKE OPENED");
      break;
    case 5: dataFile.println("BRAKE CLOSED");
      break;
    case 6:
      break;
    case 7: dataFile.println("MOTOR BURNOUT");
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
  servo.attach(9);
  servo.write(0);
}
void ServoFunction(){
  
  /*Function that will close and open brakes. If brake a true, 
    we want brakes to open. If false, brakes will closed. */
    
  if(brake){
    LogWrite(4);
    for(; pos >= 200; pos += 10){
      servo.write(pos);
      delay(10);
    }
  } else{
    LogWrite(5);
    for(; pos <= 0; pos -= 10){
      servo.write(pos);
      delay(10);
    }
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
   //what is map?
   accX = map(accX, 0, 4096, 0, 32);
//   Serial.print(accX);
   accY = map(accY, 0, 4096, 0, 32);
//   Serial.print("  ");
//   Serial.println(accY);
   accZ = 0-map(accZ, 0, 4096, 0, 32);
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

  int A=1,un=0,H=1,B=0;
  double Prediction,P,y,S,K,FR2,Q,R;

    Q=0.1;
    R=0.2;      // defined these from the Matlab code
  // inputs are Unfiltered Response, Filtered  response, Prediction value old
  // make sure to predefine P and possibly other values at the beginning of the main code. starts at 1

  //UnFV    // unfiltered value from sensors read in (zn)           -internal logic
  //FR1       // read this variable in from main (AX(i)) defined last iteration -internal logic
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

  //all heights are in ft, multiplier has no dimenstions
  double projHeight;        //The projected final height with no air brakes at this moment in time
  double desiredFinalHeight;    //The final height that the rocket should reach at this moment in time
  double multiplier;        //This is multiplied with FINALHEIGHT to equal desiredFinalHeight
  double k;           //Drag to be used tp calculate extraHeight
  double extraHeight;       //This is added to ALTREFINE to equal projectedHeight


  //Calcualtes projectedHeight
  k = 0.5 * AREACLOSED * CDCLOSED * 0.0023769; //from old code, check constants
  extraHeight = (MASS / (2.0 * k)) * log(((MASS * g) +(k * velocity * velocity)) / (MASS * g));
  projHeight = (altRefine + extraHeight);

  //Calculates desiredFinalHeight
  //multiplier = ((204 - (double) whichBrake) / 200); //figure out whichBrake
  desiredFinalHeight = FINALHEIGHT * multiplier;

  //If projectedHeight will surpass desiredFinalHeight
  if (projHeight > desiredFinalHeight) // + ERROR)
  {
    brake = true;
  }
  else
  {
    brake = false;
  
  }
  ServoFunction();
}

double Integrate(unsigned long time1, unsigned long time2, double Val1, double Val2) {

  /*function requires two times, and two data points*/

  double Time2 = time2/1000, Time1 = time1/1000;

  //Area        //result of integration     -output
  //Val1        //input of initial value    -input
  //Val2        //input of secondary inital   -input
  //deltaT      //change in time        -internal logic
  //deltaA      //change in value       -internal logic
  //time2       //Previous time         -input
  //time1       //time current          -input

  double deltaT, deltaA, Area;
  
  deltaT = Time2-Time1;      // computes deltaT
  deltaA = (Val2+Val1)/2;    // computes deltaA
  Area = deltaT*deltaA;      // computes Area
  
  return Area; 
} 
  
double Derive(unsigned long OldTime, unsigned long time, double altPrev, double altRefine){
  
  double Slope;
  
  Slope=(altRefine-altPrev)/(((double) time/1000)-((double) OldTime/1000));
  return Slope;
}

void Burnout(){

  /* Detects when the vertical acceleration is negative. Holds arduino hostage until it's satisfied*/
  
  //Ax    //Filtered vertical acceleration      //input
  //Burnout //False when motor is on, true afterwards //output
  
  while(!burnout){
    UpdateData();   
//    Serial.println(accX);
//    Serial.println(Ax);
    if(Ax <= -(g+1.0))  //checks if vertical acceleration is <= ~ -30
      burnout = true;
  }

  LogWrite(7);
  Serial.println("leaving Burnout");
}

void EndGame(){
  
  /* Checks if apogee has been reached or if the rocket is on a poor trajectory. 
  If so, the brakes will permanently close and data will be logged until end of flight*/

  
  if((velocityms5611 < 0 && burnout == true) || ( abs(accY)>=32  || abs(accZ)>=32)){
    if(velocityms5611 < 0){  
      LogWrite(3);    //checks what to write to log file
    }else{
      LogWrite(2);
    }
    
    brake = false;
    ServoFunction(); //closes brakes since we set brake to false
    
    while(true){        //brakes closed, flight data will be logged until computer is turned off
      UpdateData();
      WriteData();
      GPSloop();
    }
  } 
}

void UpdateData(){
  
  /* Updates all global variables for calculations */
  
  time = millis();
  GetAcc();
//  Serial.println(accZ);
  //Serial.print(avAcceleration);
  //Serial.print("    ");
  GetAlt();
  //Serial.println(altitude);
  Serial.println(altRefine);
  velocityNew = Integrate(OldTime, time, AxPrev, Ax);
  positionNew = Integrate(OldTime, time, oldVelocity, velocity);  // integrating acceleration from MPU since last measurement to get velocity
  velocityms5611 = Derive(OldTime, time, altPrev, altitude);   // deriving velocity from position from ms5611
  accelerationms5611 = Derive(OldTime, time, oldVelocity, velocityms5611);
  positionNew=Integrate(OldTime,time,oldVelocity,velocity);   // integration to find position from MPU
  velocity+=velocityNew;            // calculates the now new velocity
  position+=positionNew;            // Position is defined as 0 at start 
  
  avAcceleration = (accelerationms5611 + accX) / 2.0;
  avPosition = (position + altitude) / 2.0;
  avVelocity = (velocity + velocityms5611) / 2.0;
  
  Ax = Kalman(avAcceleration, AxPrev, &PnextAx);
  altRefine = Kalman(avPosition, altPrev, &PnextALT);
  
  OldTime=time;   // ms, reassigns time for lower bound at next integration cycle (move this to top of loop to eliminate any time delays between time and oldTime to have a better integration and derivation?)
  AxPrev=avAcceleration;    // reassigns Ax for initial acceleration at next integration cycle  
  oldVelocity = velocity; // still need the now old velocity to find position at that time 
  altPrev=altRefine;    // reassigns altRefine for initial altitude at next derivation

}
