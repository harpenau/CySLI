#include <MS5611.h>

MS5611 ms5611;

//ms5611 Altitude
double altitude; 
double baseline;    // baseline pressure

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
