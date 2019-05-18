#include "Arduino.h"
#include "MPU9250.h" // from Bolder Flight Systems MPU9250
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <TinyGPS.h>

#define WOD_TRANSMIT_BUF_LEN (81)


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

//TRANSMIT BUFFER
char transmit[WOD_TRANSMIT_BUF_LEN];


//SETUP
void setup() {
  //GENERIC
  Serial.begin(115200);
  //while(!Serial); //will hang if teensy not connected to pc

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
            gps.get_position(&lat, &lon, &fix_age);
            gps.get_datetime(&date, &time, &fix_age);
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
            //Serial1.println("NFD");
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

  
  
  Serial.print("Battery Voltage:   "); Serial.print(ina219_bat.getBusVoltage_V()); Serial.println(" V");
  Serial.print("Battery Current:       "); Serial.print(ina219_bat.getCurrent_mA()); Serial.println(" mA");
  Serial.print("5V Voltage:   "); Serial.print(ina219_5v.getBusVoltage_V()); Serial.println(" V");
  Serial.print("5V Current:       "); Serial.print(ina219_5v.getCurrent_mA()); Serial.println(" mA");
  Serial.print("3V3 Voltage:   "); Serial.print(ina219_3v3.getBusVoltage_V()); Serial.println(" V");
  Serial.print("3V3 Current:       "); Serial.print(ina219_3v3.getCurrent_mA()); Serial.println(" mA");

  // WOD send
  
  long long time_send = 0;
  if (fix_age == TinyGPS::GPS_INVALID_AGE) {
    *((int*)&transmit[0]) = millis();
    *((int*)&transmit[2]) = millis();
    *((int*)&transmit[4]) = micros();
    *((int*)&transmit[6]) = micros();
  } else {
    time_send = time;
  }
  
  transmit[8] = 0xFF;
  transmit[9] = 0x00;

  *((float*)&transmit[9]) = ina219_bat.getBusVoltage_V();
  *((float*)&transmit[13]) = ina219_bat.getCurrent_mA();
  *((float*)&transmit[17]) = 12.34;

  *((float*)&transmit[21]) = ina219_5v.getBusVoltage_V();
  *((float*)&transmit[25]) = ina219_5v.getCurrent_mA();
  *((float*)&transmit[29]) = ina219_3v3.getBusVoltage_V();
  *((float*)&transmit[33]) = ina219_3v3.getCurrent_mA();
  *((float*)&transmit[37]) = 23.45;

  *((float*)&transmit[41]) = 34.56;
  
  unsigned long fixAge;

  float latitude, longitude;
  gps.f_get_position(&latitude, &longitude, &fixAge);

  if (fixAge == TinyGPS::GPS_INVALID_AGE) {
    *((float*)&transmit[45]) = 360.0;
    *((float*)&transmit[49]) = 360.0;
    *((float*)&transmit[53]) = -1.0;
  } else {
    *((float*)&transmit[45]) = latitude;
    *((float*)&transmit[49]) = longitude;
    *((float*)&transmit[53]) = gps.f_altitude();
  }

  *((float*)&transmit[57]) = 45.67;
  *((float*)&transmit[61]) = 56.78;
  *((float*)&transmit[65]) = 67.89;
  *((float*)&transmit[69]) = IMU.getGyroZ_rads();
  *((float*)&transmit[73]) = IMU.getGyroY_rads();
  *((float*)&transmit[77]) = IMU.getGyroX_rads();

  Serial1.write(0x7E); //flag
  for (char i = 0; i < WOD_TRANSMIT_BUF_LEN; i++) {
    Serial1.write(transmit[i]);
    Serial.write(transmit[i]);
  }
  Serial1.write(0x7E); //flag
  
  //delay(1000);
}
