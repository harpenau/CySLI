#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SysCall.h>

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
#include <limits.h>
#include <SPI.h>    // Serial Peripheral Interface(SPI) used for communicating with one or more peripheral devices quickly over short distances
//#include <SD.h>     // microSD card library
#include <MS5611.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>

#define g 32.174    // gravity, ft/s^2
#define CDCLOSED 0.7  //The coefficient of drag when the air brakes are closed 
#define AREACLOSED 0.25  //The area of the rocket when the air brakes are closed, ft^2
#define FINALHEIGHT 5280.0  //Final height we want rocket to reach at apogee, in ft
#define AIRDENSITY 0.0023 //Air density of launch field, lb/(g*ft^3)
#define MASS 3   //Mass of the rocket, lb/g
#define CS_PIN 53

short pos=35;      // postition of the servo, defaults to closed position (degrees)

bool burnout = false; //current status of motor (false = motor active)
bool brake = false; //status of the brakes (false = closing, true = opening)

//These are not all on the same line to comment them
double AccPrev = 0;      // previous x acceleration
double PnextAcc = 0;     // prediction of next x acceleration for kalman filter, used exclusively in the kalman function
double PnextAlt = 0;    // prediction of next altitude for kalman filter, used exclusively in the kalman function
double PnextVel = 0;    // prediction of next velocity for kalman filter
double altPrev = 0;     // saved altitude from previous loop, ft
double velPrev = 0;
double altRefine = 0;   // smoothed altitude, ft
double accRefine = 0;          // smoothed vertical acceleration, ft/s^2
double velIntegral = 0; // integrated velocity from vertical acceleration, ft/s
double velDerive = 0;   // derivated velocity from altitude, ft/s
double velocity = 0;    // averaged velocity, ft/s
double baseline;        // baseline pressure, taken during IMU setup
double maxHeight = 0;   // current max height the rocket has reached in current flight, ft
double projHeight = 0;

unsigned long time=0;     // current time, ms
unsigned long OldTime=0;  // time from previous loop, ms

static NMEAGPS  gps;  //gps variables
static gps_fix  fix;

//Objects
MS5611 ms5611;
MPU6050 mpu;
Servo servo;        // airbrake servo
File dataFile;      // the datafile variable to save stuff to microSD

File file;

SdFat SD;
File csv = SD.open("importdata.csv", FILE_READ);
char delim = ',';


//MPU 6050 Accelerations
int16_t accX, accY, accZ;   // unfiltered accelerations, ft/s^2

//ms5611 Altitude
double altitude;  //raw altitude, ft

void setup(){
  SerialSetup();
  MpuSetup();     
  ms5611Setup();
  ServoSetup();

  csvSetup();
  //SDcardSetup();
  SDcardWriteSetup();    
  //GpsSetup();
  Burnout();
}

void loop() { // run code (main code) 
  while(csv.available()){
  UpdateData();

  if(burnout){
    ApogeePrediction(velocity);
  }

  EndGame();
  
  WriteData();
  }
  return;
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
//   accX = map(accX, 0, 4096, 0, 32);
//   accY = map(accY, 0, 4096, 0, 32)-32; //if y is pointing up (or usb port) subtract 31
//   accZ = map(accZ, 0, 4096, 0, 32); //pelican
     accY = accY / 2048;
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
  //double vel    //current filtered velocity
  //double projHeight;        //The projected final height with no air brakes at this moment in time
  //double k ;           //Drag to be used to calculate projHeight

  //Calcualtes projectedHeight
  double k = 0.5 * AREACLOSED * CDCLOSED * AIRDENSITY; //from old code, check constants
  
  projHeight = (MASS / (2.0 * k)) * log(((MASS * g) +(k * vel * vel)) / (MASS * g)) + altRefine;
  //If projectedHeight will surpass desiredFinalHeight
  
  if (projHeight > (FINALHEIGHT + 30) ) //error of 30ft, pelican
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
  
  return val*(currTime-prevTime)/1000.0; //computes and returns Area, the result of the integration
} 
  
double Derive(unsigned long OldTime, unsigned long time, double altPrev, double altRefine){
  
  return (altRefine-altPrev)/( ((double) time -(double) OldTime) / 1000);
}

void Burnout(){

  /* Detects when the vertical acceleration is negative. Holds arduino hostage until it's satisfied*/
  
  //Burnout      //False when motor is on, true afterwards //output
  
  UpdateData();
  UpdateData(); //these calibrate the prev values for filtering
  
  while(accRefine < 30){ // simple launch detection using vertical acc, 100% necessary for test flight/competition
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
  
  LogWrite(7);
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
    
      while(true){        //brakes closed, flight data will be logged until computer is turned off
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
    
            while(true){        //brakes closed, flight data will be logged until computer is turned off
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


//  GetAcc();
//  GetAlt();
  time = csvReadUint32(&csv, &time, delim);
  altitude = csvReadDouble(&csv, &altitude, delim);
  csvReadInt16(&csv, &accX, delim);
  csvReadInt16(&csv, &accY, delim);
  csvReadInt16(&csv, &accZ, delim);
  
  altRefine = Kalman(altitude, altPrev, &PnextAlt);   //DO NOT TOUCH
  accRefine = Kalman(accY, AccPrev, &PnextAcc);       //Only change first parameter for desired vertical acc direction
  
  velIntegral += Integrate(OldTime, time, accRefine); //Integrating to get new velocity  
  velDerive = Derive(OldTime, time, altPrev, altRefine);
  velocity = Kalman( (velDerive + velIntegral)/2, velPrev, &PnextVel);
  
  //ServoTest();
  SerialTest();
  OldTime=time;         // reassigns time for next integration cycle, DO NOT TOUCH
  AccPrev=accRefine;    // reassigns accRefine for initial acceleration at next integration cycle and kalman, DO NOT TOUCH
  altPrev=altRefine;    // reassigns altRefine for initial altitude at next derivation and kalman, DO NOT TOUCH
  velPrev = velocity;   //for kalman filter, DO NOT TOUCH
    
  if(altRefine > maxHeight)
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

 /*For debugging, uncomment desired variables and call within UpdateData()*/
  Serial.print("Time: "); Serial.print(time);
  Serial.print("rawAlt: "); Serial.print(altitude); Serial.print(",");
  Serial.print("AltRefine: "); Serial.print(altRefine); Serial.print(",");
  Serial.print("AccRefine: "); Serial.print(accRefine); Serial.print(",");
  Serial.print("Velocity: "); Serial.print(velocity); Serial.print(",");
  
  Serial.println(""); // prints new line
  delay(200);         // optional delay of output
 
}

// Functions to read a CSV text file one field at a time.
//

/*
   Read a file one field at a time.

   file - File to read.

   str - Character array for the field.

   size - Size of str array.

   delim - csv delimiter.

   return - negative value for failure.
            delimiter, '\n' or zero(EOF) for success.
*/
int csvReadText(File* file, char* str, size_t size, char delim) {
  char ch;
  int rtn;
  size_t n = 0;
  while (true) {
    // check for EOF
    if (!file->available()) {
      rtn = 0;
      break;
    }
    if (file->read(&ch, 1) != 1) {
      // read error
      rtn = -1;
      break;
    }
    // Delete CR.
    if (ch == '\r') {
      continue;
    }
    if (ch == delim || ch == '\n') {
      rtn = ch;
      break;
    }
    if ((n + 1) >= size) {
      // string too long
      rtn = -2;
      n--;
      break;
    }
    str[n++] = ch;
  }
  str[n] = '\0';
  return rtn;
}
//------------------------------------------------------------------------------
int csvReadInt32(File* file, int32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtol(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadInt16(File* file, int16_t* num, char delim) {
  int32_t tmp;
  int rtn = csvReadInt32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp < INT_MIN || tmp > INT_MAX) return -5;
  *num = tmp;
  return rtn;
}
//------------------------------------------------------------------------------
int csvReadUint32(File* file, uint32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtoul(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadUint16(File* file, uint16_t* num, char delim) {
  uint32_t tmp;
  int rtn = csvReadUint32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp > UINT_MAX) return -5;
  *num = tmp;
  return rtn;
}
//------------------------------------------------------------------------------
int csvReadDouble(File* file, double* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtod(buf, &ptr);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadFloat(File* file, float* num, char delim) {
  double tmp;
  int rtn = csvReadDouble(file, &tmp, delim);
  if (rtn < 0)return rtn;
  // could test for too large.
  *num = tmp;
  return rtn;
}
//------------------------------------------------------------------------------
void csvSetup() {

  // Initialize the SD.
  if (!SD.begin(CS_PIN)) {
    Serial.println("begin failed");
    return;
  }
  // Remove existing file.
    SD.remove("READTEST.TXT");

  // Create the file.
  file = SD.open("importdata.csv", FILE_READ);
  if (!file) {
    Serial.println("open failed");
    return;
  }
  // Write test data.   (Disabled for my test with real data)
    file.print(F(
  "36,23.20,20.70,57.60,79.50,01:08:14,23.06.16\r\n"
  "37,23.21,20.71,57.61,79.51,02:08:14,23.07.16\r\n"
  ));

  // Rewind the file for read.
  file.seek(0);

  // Read the file and print fields.
  int16_t tcalc;
  float t1, t2, h1, h2;
  // Must be dim 9 to allow for zero byte.
  char timeS[9], dateS[9];
  while (file.available()) {
  if (csvReadUint32(&csv, &time, delim) != delim
        || csvReadDouble(&csv, &altitude, delim) != delim
        || csvReadInt16(&csv, &accX, delim) != delim
        || csvReadInt16(&csv, &accY, delim) != delim
        || csvReadInt16(&csv, &accZ, delim) != delim
        || csvReadText(&file, dateS, sizeof(dateS), delim) != '\n') {
      Serial.println("read error");
      int ch;
      int nr = 0;
      // print part of file after error.
      while ((ch = file.read()) > 0 && nr++ < 100) {
        Serial.write(ch);
      }
      break;
    }
    Serial.print(time);
    Serial.print(delim);
    Serial.print(altitude);
    Serial.print(delim);
    Serial.print(accX);
    Serial.print(delim);
    Serial.print(accY);
    Serial.print(delim);
    Serial.print(accZ);
    Serial.print(delim);
    Serial.println(dateS);
  }
  file.close();
}

