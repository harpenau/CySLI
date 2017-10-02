#include <Wire.h>
#include <MS5611.h>
#include <servo.h> 
 
MS5611 ms5611;
 
double referencePressure;
 
void setup()
{
  Serial.begin(9600);
 
  // Inicjalizacja MS5611
  Serial.println("Inicjalizacja MS5611 Sensor");
 
  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  while(!ms5611.begin(MS5611_HIGH_RES))
  {
    Serial.println("Nie mozna znalezc czujnika MS5611, sprawdz polaczenie!");
    delay(500);
  }
 
  // Pobieramy poczatkowe cisnienie do okreslenia wysokosci
  referencePressure = ms5611.readPressure();
 
  // Sprawdzamy ustawienia
  checkSettings();
}
 
void checkSettings()
{
  Serial.print("Oversampling: ");
  Serial.println(ms5611.getOversampling());
}
 
void loop()
{
  // Odczyt surowych wartosci
  uint32_t rawTemp = ms5611.readRawTemperature();
  uint32_t rawPressure = ms5611.readRawPressure();
 
  // Odczyt przekonwertowanych wartosci
  double realTemperature = ms5611.readTemperature();
  long realPressure = ms5611.readPressure();
 
  // Obliczanie wysokosci
  float absoluteAltitude = ms5611.getAltitude(realPressure);
  float relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);
 
  // Wyswietlenie wynikow
  Serial.println("--");
 
  Serial.print(" rawTemp = ");
  Serial.print(rawTemp);
  Serial.print(", realTemp = ");
  Serial.print(realTemperature);
  Serial.println(" *C");
 
  Serial.print(" rawPressure = ");
  Serial.print(rawPressure);
  Serial.print(", realPressure = ");
  Serial.print(realPressure);
  Serial.println(" Pa");
 
  Serial.print(" absoluteAltitude = ");
  Serial.print(absoluteAltitude);
  Serial.print(" m, relativeAltitude = ");
  Serial.print(relativeAltitude);    
  Serial.println(" m");
 
  delay(1000);
}
