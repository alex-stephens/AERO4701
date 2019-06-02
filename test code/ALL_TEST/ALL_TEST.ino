#include <Time.h>
#include <TimeLib.h>
#include "TimedAction.h"
#include "Arduino.h"
#include "MPU9250.h" // from Bolder Flight Systems MPU9250
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "TinyGPS++.h"
#include <L298N.h>
#include <math.h>

#define WOD_TRANSMIT_BUF_LEN (86)
#define SCI_TRANSMIT_BUF_LEN (12)
#define ADCS_BUF_LEN (500)


// MAIN OPERATION MODES
#define MODE_OPERATIONAL 0
#define MODE_DOWNLINK 1
#define MODE_DETUMBLE 2
#define MODE_POINTING 3
#define MODE_DEPLOYMENT 4
#define MODE_SAFE 5
#define MODE_LAUNCH 6

// OTHER MODES
#define MODE_STARTUP 7
#define MODE_TESTING 8 // for any debugging stuff we want to do

unsigned char mode;
unsigned char prevMode;

//GLOBALS


//IMU
MPU9250 IMU(Wire1,0x68);
int imuStatus;

//GPS
TinyGPSPlus gps;
TinyGPSCustom numSat(gps, "GPGSV", 3);
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

// MAGNETORQUERS
L298N coilX(20, 27, -1); // -1 for unused pins
L298N coilY(17, 28, -1);
L298N coilZ(16, 29, -1);


// Main timed events
void transmitWOD();
void transmitSci();
void updateADCS();
void readGPSdata();
void printGPSdata();
void debugPrint();
void ADCSPointing();
void ADCSDetumble();

TimedAction transmitWodThread = TimedAction(1000, transmitWOD);
TimedAction transmitSciThread = TimedAction(1000, transmitSci);
TimedAction ADCSDetumbleThread = TimedAction(100, ADCSDetumble);
TimedAction ADCSPointingThread = TimedAction(100, ADCSPointing);

// Other timed events
TimedAction debugPrintThread = TimedAction(1000, debugPrint);

TimedAction readGPSdataThread = TimedAction(1000, readGPSdata);

// ADCS control gains
float yaw, pitch, roll; // degrees
void updateYawPitchRoll(); 

int Kp = 2, Ki = 0.3, Kd = 1;

// ADCS buffers (integral control)
float ADCS_wx_err[ADCS_BUF_LEN];
float ADCS_wy_err[ADCS_BUF_LEN];
float ADCS_wz_err[ADCS_BUF_LEN];

float ADCS_wx_integral = 0;
float ADCS_wy_integral = 0;
float ADCS_wz_integral = 0;

float ADCS_yaw_err[ADCS_BUF_LEN];
float ADCS_yaw_err_integral;

// Magnetometer calibration
#define MAG_CAL_X (7.5)
#define MAG_CAL_Y (-29.5)
#define MAG_CAL_Z (-34.5)


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
  digitalWrite(13, LOW);

  analogReference(0);
  analogReadRes(10);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(39, INPUT);
  pinMode(23, INPUT);
  pinMode(34, INPUT);
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);

  // IMU
  while (IMU.begin() < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    delay(500);
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
  delay(3000);
//  motorX.backward();
//  set
//  setMotorSpeed(motorX, -100);
  delay(3000);

  // set the initial operating mode
  mode = MODE_TESTING;
  prevMode = mode;
}

// ---------------------------------------------------------------- //
//                             MAIN LOOP                            //
// ---------------------------------------------------------------- //


void loop() {
 
  // C&C
  while (Serial1.available()) {
    mode = Serial1.read();
  }

  // mode transition handler
  modeTransition(prevMode, mode);


  switch (mode) {
    case MODE_OPERATIONAL :
      modeOperational();
      break;

    case MODE_DOWNLINK :
      modeDownlink();
      break;

    case MODE_DETUMBLE :
      modeDetumble();
      break;

    case MODE_POINTING :
      modePointing();
      break;

    case MODE_DEPLOYMENT :
      modeDeployment();
      break;

    case MODE_SAFE :
      modeSafe();
      break;

    case MODE_LAUNCH :
      modeLaunch();
      break;

    case MODE_STARTUP :
      modeStartup();
      break;

    case MODE_TESTING :
      modeTesting();
      break;

    default :
      Serial.println("Invalid mode :( ");
      delay(1000);
      break;
  }

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

  prevMode = mode;
}

// ---------------------------------------------------------------- //
//                       OPERATIONAL MODES                          //
// ---------------------------------------------------------------- //

void modeTransition(unsigned char mode1, unsigned char mode2) {

  // no transition
  if (mode1 == mode2) {
    return;
  }

  Serial.print("Transitioning from Mode "); Serial.print(mode1); 
  Serial.print(" to Mode "); Serial.println(mode2);


  // ENTERING NEW MODE
  switch (mode2) {
    case MODE_OPERATIONAL :
      Serial.println("Entering operational mode. . .");
      break;

    case MODE_DOWNLINK :
      Serial.println("Entering dpwnlink mode. . .");
      break;

    case MODE_DETUMBLE :
      Serial.println("Entering detumble mode. . .");
      break;

    case MODE_POINTING :
      Serial.println("Entering pointing mode. . .");
      break;

    case MODE_DEPLOYMENT :
      Serial.println("Entering deployment mode. . .");
      break;

    case MODE_SAFE :
      Serial.println("Entering safe mode. . .");
      break;

    case MODE_LAUNCH :
      Serial.println("Entering launch mode. . .");
      break;

    case MODE_STARTUP :
      Serial.println("Entering startup mode. . .");
      break;

    case MODE_TESTING :
      modeTesting();
      break;
  }


  if (mode2 == MODE_SAFE) {
    // behaviour for entering safe mode
  }

  if (mode1 == MODE_SAFE) {
    // behaviour for exiting safe mode
  }

  // behaviour for a specific mode transition
//  if (mode1 == MODE_OPERATIONAL && mode2 == MODE_DOWNLINK) {
//
//  }
}

void modeOperational() {
  transmitWodThread.check();
  transmitSciThread.check();
  ADCSPointingThread.check();
}

void modeDownlink() {
  transmitWodThread.check();
  transmitSciThread.check();
}

void modeDetumble() {
  ADCSDetumbleThread.check();
}

void modePointing() {
  ADCSPointingThread.check();
}


void modeDeployment() {
  tetherDeploy();
  delay(1000);
  mode = MODE_OPERATIONAL;
}

void modeSafe() {

}

void modeLaunch() {

}

void modeStartup() {

}

void modeTesting() {
  delay(1000);
  readGPSdataThread.check();
  transmitWodThread.check();
  transmitSciThread.check();
//  ADCSPointingThread.check();


  // gyro print
//  Serial.print("Gyro - X:");
//  Serial.print(IMU.getGyroX_rads(),6);
//  Serial.print("\tY: ");
//  Serial.print(IMU.getGyroY_rads(),6);
//  Serial.print("\tZ: ");
//  Serial.println(IMU.getGyroZ_rads(),6);

  // accel print
//  Serial.print(IMU.getAccelX_mss(),6);
//  Serial.print("\t");
//  Serial.print(IMU.getAccelY_mss(),6);
//  Serial.print("\t");
//  Serial.print(IMU.getAccelZ_mss(),6);
//  Serial.println("\t");
}


// ---------------------------------------------------------------- //
//                       HELPER FUNCTIONS                           //
// ---------------------------------------------------------------- //


time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void debugPrint() {

  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  
  // GPS
  printGPSdata();

  // EPS
  printEPSdata();

  // IMU
  printIMUdata();
}

void transmitSci() {
  getSci();
  
  Serial1.write(0x7E); //flag
  Serial1.write(0x53); // S for Sci
  for (char i = 0; i < SCI_TRANSMIT_BUF_LEN; i++) {
    Serial1.write(transmit[i]);
    Serial.write(transmit[i]);
  }
  Serial1.write(0x7E); //flag
  Serial.write('\n');
}

void getSci() {
  *((unsigned long*)&transmit[0]) = now();
  unsigned int vt = analogRead(23);
  unsigned int vz = analogRead(34);
  *((float*)&transmit[4]) = vt*3.3/1024;
  *((float*)&transmit[8]) = vz*3.3/1024;
}

void transmitWOD() {
  getWOD();
  
  Serial1.write(0x7E); //flag
  Serial1.write(0x57); // W for WOD 
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

  
  if (gps.satellites.value() == 0) {
    *((float*)&transmit[45]) = 360.0;
    *((float*)&transmit[49]) = 360.0;
    *((float*)&transmit[53]) = -1.0;
  } else {
    *((float*)&transmit[45]) = gps.location.lat();
    *((float*)&transmit[49]) = gps.location.lng();
    *((float*)&transmit[53]) = gps.altitude.meters();
  }

  // update yaw, pitch and roll before storing
  updateYawPitchRoll();
  *((float*)&transmit[57]) = yaw;
  *((float*)&transmit[61]) = pitch;
  *((float*)&transmit[65]) = roll;
  *((float*)&transmit[69]) = IMU.getGyroZ_rads();
  *((float*)&transmit[73]) = IMU.getGyroY_rads();
  *((float*)&transmit[77]) = IMU.getGyroX_rads();

  transmit[81] = gps.satellites.value();
  *((unsigned long*)&transmit[82]) = gps.hdop.value();
}

// single axis attitude control (yaw) using reaction wheels 
void ADCSPointing() {
  // index to insert into buffer
  static int i = 0;

  // ensure that yaw, pitch, roll are current
  updateYawPitchRoll();
  
  float yaw_err = 0 - yaw; // negative to account for direction we want the motor to rotate
  ADCS_yaw_err_integral += yaw_err - (ADCS_yaw_err[i]);

  // update integral buffers
  ADCS_yaw_err[i] = yaw_err;

  // calculate control values (PI control)
  int inZ = - (Kp * yaw_err) - (Ki * ADCS_yaw_err_integral);

//  Serial.println(ADCS_yaw_err_integral);

  // set motor speeds
  setMotorSpeed(motorZ, inZ);  

  i = (i+1) % ADCS_BUF_LEN;
}

// detumbling using the magnetorquers
void ADCSDetumble() {
  static int i = 0;
  
  int wx = IMU.getGyroX_rads();
  int wy = IMU.getGyroY_rads();
  int wz = IMU.getGyroZ_rads();

  float Bx = IMU.getMagX_uT() - MAG_CAL_X;
  float By = IMU.getMagY_uT() - MAG_CAL_Y;
  float Bz = IMU.getMagZ_uT() - MAG_CAL_Z;

  // B DOT CONTROL
  float Bdotx = wy*Bz - wz*By;
  float Bdoty = wz*Bx - wx*Bz;
  float Bdotz = wx*By - wy*Bx;

  float Bmax = max(max(abs(Bdotx), abs(Bdoty)), abs(Bdotz));

  int cX = (Bdotx * 255) / Bmax;
  int cY = (Bdoty * 255) / Bmax;
  int cZ = (Bdotz * 255) / Bmax;

  setMotorSpeed(coilX, cX);
  setMotorSpeed(coilY, cY);
  setMotorSpeed(coilZ, cZ);

  i = (i+1) % ADCS_BUF_LEN;
}

void updateYawPitchRoll() {

  IMU.readSensor();

  // accelerometer readings
  float accX = IMU.getAccelX_mss();
  float accY = IMU.getAccelY_mss();
  float accZ = IMU.getAccelZ_mss();

  // calibrated magnetometer readings
  float magX = IMU.getMagX_uT() - MAG_CAL_X;
  float magY = IMU.getMagY_uT() - MAG_CAL_Y;
  float magZ = IMU.getMagZ_uT() - MAG_CAL_Z;

  // update global parameters
  yaw =  -180 * atan2(magY, magX)/M_PI; // pretty dubious formula
  pitch = 180 * atan (accX/sqrt(accY*accY + accZ*accZ))/M_PI;
  roll = 180 * atan (accY/sqrt(accX*accX + accZ*accZ))/M_PI;

  sprintf(s, "Yaw: %-9.4f, pitch: %-9.4f, roll: %-9.4f", yaw, pitch, roll);
//  Serial.println(s);   

  // Magnetometer raw values for calibration
//  sprintf(s, "Magnetometer - x: %7.4f, y: %7.4f, z: %7.4f", magX, magY, magZ);
//  Serial.println(s);   
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

void tetherDeploy() {
  Serial.print("Start deploy... ");
  digitalWrite(21, HIGH);
  delay(1000);
  digitalWrite(21, LOW);
  Serial.println("Done!");
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

void readGPSdata() {
  while (Serial2.available() > 0) {
    // read the incoming byte:
    char c = Serial2.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year()); 
    }
  }
}

void printGPSdata() {
    sprintf(s, "Lat: %ld, Lon: %ld, Time: %ld", gps.location.lat(), gps.location.lng(), gps.time.value());
    Serial.println(s);       
}
