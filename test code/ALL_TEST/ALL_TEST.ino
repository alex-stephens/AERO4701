#include <Time.h>
#include <TimeLib.h>
#include <TimedAction.h>
#include "Arduino.h"
#include "MPU9250.h" // from Bolder Flight Systems MPU9250
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <TinyGPS.h>
#include <L298N.h>

#define WOD_TRANSMIT_BUF_LEN (86)

//GLOBALS

//GENERIC
unsigned char mode;

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

// MOTOR DRIVERS
L298N motorX(2, 24, -1); // -1 for unused pins
L298N motorY(5, 25, -1);
L298N motorZ(6, 26, -1);

// Main timed events
TimedAction telemetryTimedAction = TimedAction(1000, transmitWOD);
TimedAction ADCSTimedAction = TimedAction(100, updateADCS);

// Other timed events
TimedAction debugPrintTimedAction = TimedAction(1000, debugPrint);


//SETUP
void setup() {
  //GENERIC
  Serial.begin(115200);
  //while(!Serial); //will hang if teensy not connected to pc

  //CLOCK
  setSyncProvider(getTeensy3Time);
  setSyncInterval(1);
  setTime(0);

  pinMode(13, OUTPUT);
  mode = 0;
  digitalWrite(13, LOW);

  analogReference(0);
  analogReadRes(10);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(39, INPUT);
  
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

  // MOTOR - use setSpeed(0-255) then run forward() or backward()
//  motorX.setSpeed(100);
//  motorX.forward();
//  delay(3000);
//  motorX.backward();
//  delay(3000);


  // set time intervals for all timed actions
  telemetryTimedAction.setInterval(1000);
  ADCSTimedAction.setInterval(100);
  
  debugPrintTimedAction.setInterval(5000);
}


void loop() {
  
  // C&C
  while (Serial1.available()) {
    mode = Serial1.read();
  }
//  digitalWrite(13, HIGH); // mode&0x01);

  // check which timed actions to execute
  telemetryTimedAction.check();
  ADCSTimedAction.check();
  debugPrintTimedAction.check();

//  // flash LEDs for number of GPS satellites
//  delay(2000);
//  if (transmit[81] != 255) {
//    for (char i = 0; i < transmit[81]; i++) {
//      digitalWrite(13, LOW);
//      delay(20);
//      digitalWrite(13, HIGH);
//      delay(20);
//    }
//  }
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void debugPrint() {
  // GPS
  printGPSdata();

  // EPS
  printEPSdata();
}


void transmitWOD() {
  getWOD();
  
  Serial1.write(0x7E); //flag
  for (char i = 0; i < WOD_TRANSMIT_BUF_LEN; i++) {
    Serial1.write(transmit[i]);
    Serial.write(transmit[i]);
  }
  Serial1.write(0x7E); //flag
  Serial.write('\n');
}

void getWOD() {
  // WOD send

  *((unsigned long*)&transmit[0]) = now();
  *((unsigned long*)&transmit[4]) = millis();
  
  transmit[8] = mode;

  *((float*)&transmit[9]) = ina219_bat.getBusVoltage_V();
  *((float*)&transmit[13]) = ina219_bat.getCurrent_mA();
  unsigned int batTemp = analogRead(15);
  *((float*)&transmit[17]) = 0.0000006*batTemp*batTemp*batTemp - 0.0008*batTemp*batTemp + 0.3892*batTemp - 51.964;

  *((float*)&transmit[21]) = ina219_5v.getBusVoltage_V();
  *((float*)&transmit[25]) = ina219_5v.getCurrent_mA();
  *((float*)&transmit[29]) = ina219_3v3.getBusVoltage_V();
  *((float*)&transmit[33]) = ina219_3v3.getCurrent_mA();
  unsigned int regTemp = analogRead(14);
  *((float*)&transmit[37]) = 0.0000006*regTemp*regTemp*regTemp - 0.0008*regTemp*regTemp + 0.3892*regTemp - 51.964;

  unsigned int rfdTemp = analogRead(39);
  *((float*)&transmit[41]) = 0.0000006*rfdTemp*rfdTemp*rfdTemp - 0.0008*rfdTemp*rfdTemp + 0.3892*rfdTemp - 51.964;

  
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

  transmit[81] = gps.satellites();
  *((unsigned long*)&transmit[82]) = gps.hdop();  
}

void updateADCS() {
  static int x = 0;
  Serial.print("ADCS update ");
  Serial.println(x++);
}

void setMotorSpeed(L298N& motor, int newSpeed) {

  // max input
  if (newSpeed > 255) {
    newSpeed = 255;
  } 
  else if (newSpeed < -255) {
    newSpeed = -255;
  }

  // update the speed
  motor.setSpeed(abs(newSpeed));

  // set the direction
  if (newSpeed >= 0) {
    motor.forward();
  }
  else {
    motor.backward();
  }
}

void printEPSdata() {
  Serial.print("Battery Voltage:   "); Serial.print(ina219_bat.getBusVoltage_V()); Serial.println(" V");
  Serial.print("Battery Current:       "); Serial.print(ina219_bat.getCurrent_mA()); Serial.println(" mA");
  Serial.print("5V Voltage:   "); Serial.print(ina219_5v.getBusVoltage_V()); Serial.println(" V");
  Serial.print("5V Current:       "); Serial.print(ina219_5v.getCurrent_mA()); Serial.println(" mA");
  Serial.print("3V3 Voltage:   "); Serial.print(ina219_3v3.getBusVoltage_V()); Serial.println(" V");
  Serial.print("3V3 Current:       "); Serial.print(ina219_3v3.getCurrent_mA()); Serial.println(" mA");
}


void printIMUdata() {
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
}

void printGPSdata() {
    while (Serial2.available() > 0) {
          // read the incoming byte:
          char c = Serial2.read();
          
          if (gps.encode(c)) {
            Serial.println("got gps!");
            gps.get_position(&lat, &lon, &fix_age);
            //gps.get_datetime(&date, &time, &fix_age);
            int year; byte month, day, hour, minute, second, hundredth;
            Serial.println("cracking...");
            gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredth, &fix_age);
            setTime(hour, minute, second, day, month, year);
            sprintf(s, "Lat: %ld Lon: %ld Time: %ld", lat, lon, time);
            Serial.println(s);
          }
          unsigned long chars; unsigned short sentences, failed_checksum;
          gps.stats(&chars, &sentences, &failed_checksum);
          sprintf(s, "chars: %ld sentences: %ld failed checks: %ld", chars, sentences, failed_checksum);
          Serial.println(s);    
  }
}
