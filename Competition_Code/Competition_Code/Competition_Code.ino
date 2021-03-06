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
// for proper formatting, please use arduino IDE 

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

#define g 32.174    // gravity, ft/s^2
#define CDCLOSED 0.7  //The coefficient of drag when the air brakes are closed 
#define AREACLOSED 0.25  //The frontal area of the rocket when the air brakes are closed, ft^2
#define FINALHEIGHT 5280.0  //Apogee we want rocket to reach, in ft
#define AIRDENSITY 0.0023 //Air density of launch field, lb/(g*ft^3)
#define MASS 3   //Mass of the rocket, lb/g

short pos=35;      // postition of the servo, defaults to closed position (degrees)

bool burnout = false; //current status of motor (false = motor active)
bool brake = false; //status of the brakes (false = closing, true = opening)

//These are not all on the same line to comment them
double AccPrev = 0;      // previous vertical acceleration
double PnextAcc = 0;     // prediction of next vertical acceleration for kalman filter, used exclusively in the kalman function
double PnextAlt = 0;     // prediction of next altitude for kalman filter, used exclusively in the kalman function
double PnextVel = 0;     // prediction of next velocity for kalman filter, ft/s
double altPrev = 0;      // saved altitude from previous loop, ft
double velPrev = 0;
double altRefine = 0;    // filtered altitude, ft
double accRefine = 0;    // filtered vertical acceleration, ft/s^2
double velIntegral = 0;  // integrated velocity from vertical acceleration, ft/s
double velDerive = 0;    // derivated velocity from altitude, ft/s
double velocity = 0;     // averaged velocity, ft/s
double baseline;         // baseline pressure, taken during IMU setup
double maxHeight = 0;    // current max height the rocket has reached in current flight, ft
double projHeight = 0;   // predicted agpogee, ft

unsigned long time=0;     // current time, ms
unsigned long OldTime=0;  // time from previous loop, ms

static NMEAGPS  gps;  //gps variables
static gps_fix  fix;

//Objects
MS5611 ms5611;      //IMU1
MPU6050 mpu;        //IMU2
Servo servo;        // airbrake servo
File dataFile;      // the datafile variable to save stuff to microSD

//MPU 6050 Accelerations
int16_t accX, accY, accZ;   // raw accelerations, ft/s^2

//ms5611 Altitude
double altitude;  //raw altitude, ft

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
    ApogeePrediction(velocity);
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
}
//-----------------------------------------------------------------SD Card Functions---------------------------------------------------
void SDcardSetup(){
  /*Set up sd card to read RC data*/
  
  pinMode(53, OUTPUT);
    SD.begin(53);  
}

void SDcardWriteSetup(){
  dataFile = SD.open("Data.txt", FILE_WRITE);
  dataFile.println(F("Time(ms),Height(ft),F Alt(ft),AccX(ft/s^2),AccY(ft/s^2),AccZ(ft/s^2),Brake Angle, F Acc(ft/s^2),zVel(ft/s),AP(ft)"));
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
 dataFile.print(accRefine);
 dataFile.print(",");
 dataFile.print(velocity);
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
    case 4:dataFile.println(F("BRAKE OPENED 5 DEG"));
      break;
    case 5: dataFile.println(F("BRAKE CLOSED"));
      break;
    case 6: dataFile.println(F("TARGET APOGEE REACHED, VEL > 0, BRAKING UNTIL FREEFALL"));
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
        pos += 5;            // opens brakes 5 degrees
        servo.write(pos);
      }
  } else if (pos > 35){
      LogWrite(5);
      pos = 35;     //closes brakes 
      servo.write(pos);
  }
  
}
//-------------------------------------------------------------------ms5611 Methods----------------------------------------------------

void ms5611Setup(){
  ms5611.begin();
  baseline = ms5611.readPressure();
}

double GetPressure(){
  return ms5611.readPressure();
}

void GetAlt(){

  /*Reads current air pressure to obtain altitude*/
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
   accX = map(accX, 0, 2048, 0, 32);
   accY = map(accY, 0, 2048, 0, 32)-32; //if y is pointing up (or usb port) subtract 31
   accZ = map(accZ, 0, 2048, 0, 32); //pelican
 
   /* Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg */
}

void MpuSetup(){
    //joins I2C bus (I2Cdev library doesn't do this automatically) (Was found in available rocket code on internet) 
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
    mpu.setFullScaleAccelRange(3);  //0=2g, 1=4g, 2=8g, 3=16g 
    mpu.setFullScaleGyroRange(2); //0=250 deg/s, 1=500 deg/s, 2=1000 deg/s, 3=2000 deg/s 
}

//------------------------------------------------------------IMPORTANT FUNCTIONS-------------------------------------------------------
double Kalman(double UnFV,double FR1,double *Pold){

/* function filtering results in real time with Kalman filter */

  char A=1,un=0,H=1,B=0;
  double Prediction,P,y,S,K,FR2;
  float Q = 0.1, R = 0.2;

  // Q=0.1, R=0.2      // defined these from the Matlab code
  // inputs are Unfiltered Response, Filtered  response, Prediction value old
  // make sure to predefine P and possibly other values at the beginning of the main code. starts at 1

  //UnFV    // unfiltered value from sensors read in (zn)           -internal logic
  //FR1     // read this variable in from main (AX(i)) defined last iteration -internal logicexc
  //Pold    // output Pold and read in Pold from last iteration       -input,output
  //Q       // sensor specific value, we might default this           -input
  //R       // sensor specific value, we might default this           -input
  //A         
  //FR2
  
  Prediction= A*FR1 +B*un;    // State Prediction     (Predict where we're goning to be)
  P=A*(*Pold)*A +Q;           // Covariance Prediction  (Predict how much error)
  y=UnFV-H*Prediction;        // Innovation       (Compare reality against prediction)
  S= H*P*H +R;                // innovation Covariance  (Compare real error against prediction)
  K=(P*H)/S;                  // Kalman Gain         (Moderate the prediction)
  FR2=Prediction+K*y;         // state update      (New estimate of where we are)
  *Pold=(1-K*H)*P;            // Covariance update     (New estimate of error)
  
  return FR2;  
}


double ApogeePrediction(double vel){

  /*Predicts apopgee from current velocity, deploys brakes
    if needed. CAN ONLY BE USED AFTER BURNOUT  */
  //all heights are in ft
  //double vel              //current velocity
  //double projHeight;      //The projected final height with no air brakes at this moment in time
  //double k ;              //Drag to be used to calculate projHeight

  //Calcualtes projectedHeight
  double k = 0.5 * AREACLOSED * CDCLOSED * AIRDENSITY; //from old code, check constants
  
  projHeight = (MASS / (2.0 * k)) * log(((MASS * g) +(k * vel * vel)) / (MASS * g)) + altRefine;
  
  if (projHeight > (FINALHEIGHT + 15) ) //error of 30ft, pelican
  {
    brake = true;
  }
  else
  {
    brake = false;
  }
  ServoFunction();    //opens or closes brakes
}

double Integrate(unsigned long prevTime, unsigned long currTime, double val) {

  /*function requires two times, and one data point
  DESIGNED FOR ACCELERATION INEGRATION ONLY*/
  
  return val*((currTime-prevTime)/1000.0); //computes and returns Area, the result of the integration
} 
  
double Derive(unsigned long OldTime, unsigned long time, double altPrev, double altRefine){
  
  return (altRefine-altPrev)/( ((double) time -(double) OldTime) / 1000);
}

void Burnout(){

  /* Detects when the vertical acceleration is negative. Holds arduino hostage until it's satisfied*/
  
  //Burnout      //False when motor is on, true afterwards //output
  
  UpdateData();
  UpdateData(); //these calibrate the prev values for filtering
  
  while(accRefine < 30){ // simple launch detection using vertical acc, 100% necessary for real flights
    UpdateData();
    //WriteData();  // uncomment if data before launch is needed
  }
  
  velocity = 0; //gets rid of garbage velocity values from sitting on the pad
  
  while(!burnout){  //waits until burnout is complete
    UpdateData();
    WriteData();
    if(accRefine <= -(g-3.0) && altRefine > 1500) //pelican //checks if vertical acc is <= ~ -30 && is over a set height
      burnout = true;
  }
  
  LogWrite(7);  //writes burnout event to datalog
 // Serial.println(F("leaving Burnout"));
}

void EndGame(){
  
  /* Checks if apogee has been reached or if the rocket is on a poor trajectory. 
  If so, the brakes will permanently close and data will be logged until end of flight*/

  if(  (maxHeight > (altRefine+30) ) && (velocity < -50)){ //checks for falling rocket using altitude and velocity

      LogWrite(3);    
      brake = false;
      ServoFunction(); //closes brakes since we set brake to false
      WriteData();
    
      while(true){        //flight data will be logged until computer is turned off
        UpdateData();
        WriteData();
        GPSloop();
      }
    
  } else if(altRefine > 5280){ // failsafe if rocket goes over 1 mile 
    LogWrite(6);
    brake = true;
    while(pos < 125){   // fully deploys brakes
      ServoFunction();
      WriteData();
      delay(40);
    }
    while(true){
        UpdateData();
        WriteData();
        if( maxHeight > (altRefine+25) ){ //checks for falling rocket, then closes brakes
            LogWrite(3);    
            brake = false;
            ServoFunction(); 
    
            while(true){        //flight data will be logged until computer is turned off
                UpdateData();
                WriteData();
                GPSloop();
            }
       }
    }
  }  
}

void UpdateData(){
  
  /* Updates all global variables for calculations */
  
  time = millis();
  GetAcc();
  GetAlt();

  altRefine = Kalman(altitude, altPrev, &PnextAlt);
  accRefine = Kalman(accY, AccPrev, &PnextAcc);
  velocity += Integrate(OldTime, time, accRefine); //Integrating to get new velocity  
  
  //ServoTest();
  SerialTest();
  
  OldTime=time;         // reassigns time for next integration cycle (move this to top of loop to eliminate any time delays between time and oldTime to have a better integration and derivation?)
  AccPrev=accRefine;    // reassigns accRefine for initial acceleration at next integration cycle and kalman
  altPrev=altRefine;    // reassigns altRefine for initial altitude at next derivation and kalman
  velPrev = velocity;   // saves velocity for next kalman cycle
  
  if(altRefine > maxHeight) //keep max height stored for apogee confirmation
    maxHeight = altRefine;
}

void ServoTest(){ 
  
  /*opens and closes brakes constantly for brake line/servo check */
  
    if(pos == 35)
      brake = true;
    if(pos == 125)
      brake = false;
  
  ServoFunction();
  delay(500);

}

void SerialTest(){

 /*For debugging, uncomment desired variables and 
     call this function within UpdateData()*/

  Serial.print("rawAlt: "); Serial.print(altitude); Serial.print(" ,");
   /*Serial.print("rawAlt: "); */Serial.print(altitude); Serial.print(" ,");
  Serial.print("AltRefine: "); Serial.print(altRefine); Serial.print(" ,");
  Serial.print("accY: "); Serial.print(accY); Serial.print(" , ");
  Serial.print("AccRefine: "); Serial.print(accRefine); Serial.print(" ,");
  Serial.print("Velocity: "); Serial.print(velocity); Serial.print(" ,");
  
  Serial.println(""); // prints new line
  delay(500);         // optional delay of output
 
}

