#include "Arduino.h"
#include "MPU9250.h" // from Bolder Flight Systems MPU9250
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <TinyGPS.h>


//GLOBALS

//GENERIC
unsigned char led;

//IMU
MPU9250 IMU(Wire1,0x68);
int imuStatus;

//GPS
TinyGPS gps;
char s[80];
long lat, lon;
unsigned long fix_age, time, date;

//EPS
Adafruit_INA219 ina219_bat(0x40);
Adafruit_INA219 ina219_3v3(0x41);
Adafruit_INA219 ina219_5v(0x44);


//SETUP
void setup() {
  //GENERIC
  Serial.begin(115200);
  while(!Serial);

  pinMode(13, OUTPUT);
  led = 0;
  digitalWrite(13, LOW);

  //IMU 
  imuStatus = IMU.begin();
  if (imuStatus < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("imuStatus: ");
    Serial.println(imuStatus);
    while(1) {}
  }

  //GPS
  Serial2.begin(115200);

  //RFD
  Serial1.begin(9600);

  //EPS
  ina219_bat.begin();
  ina219_5v.begin();
  ina219_3v3.begin();
  
}


//LOOP
void loop() {
  
  //IMU
  IMU.readSensor();
  // display the data
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);

  //GPS
  int gpsDidUpdate = 0;
  while (Serial2.available() > 0) {
          // read the incoming byte:
          char c = Serial2.read();
          
          if (gps.encode(c)) {
            Serial.println("Updated");
            sprintf(s, "Lat: %ld Lon: %ld Time: %ld", lat, lon, time);
            Serial.println(s);
            Serial1.println(s);
            gpsDidUpdate = 1;
          }
  }
  if (gpsDidUpdate > 0 || 1) {
          gps.get_position(&lat, &lon, &fix_age);
          gps.get_datetime(&date, &time, &fix_age);

          if (fix_age == TinyGPS::GPS_INVALID_AGE) {
            Serial.println("No fix detected");
            Serial1.println("NFD");
            Serial.print("Time: ");
            Serial.println(time);
          }
          else if (fix_age > 5000) {
            Serial.println("Warning: possible stale data!");
            Serial1.println("Stale");
          }
          else {
            
          }
  }

  //EPS

  float batVoltage = ina219_bat.getBusVoltage_V();
  
  Serial.print("Battery Voltage:   "); Serial.print(ina219_bat.getBusVoltage_V()); Serial.println(" V");
  Serial.print("Battery Current:       "); Serial.print(ina219_bat.getCurrent_mA()); Serial.println(" mA");
  Serial.print("5V Voltage:   "); Serial.print(ina219_5v.getBusVoltage_V()); Serial.println(" V");
  Serial.print("5V Current:       "); Serial.print(ina219_5v.getCurrent_mA()); Serial.println(" mA");
  Serial.print("3V3 Voltage:   "); Serial.print(ina219_3v3.getBusVoltage_V()); Serial.println(" V");
  Serial.print("3V3 Current:       "); Serial.print(ina219_3v3.getCurrent_mA()); Serial.println(" mA");

  Serial1.print("~~~");
  for (char i = 0; i < 4; i++) {
    Serial1.print(((char*)&batVoltage)[i]);
  }
  
  
  delay(100);
}
