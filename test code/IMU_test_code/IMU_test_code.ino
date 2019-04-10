// Licensed under the GNU General Public License v3.0

#include "Arduino.h"
#include "i2c_t3.h"  // I2C library

// MPU = Motion Processing Unit
// Register information source from the document
// MPU-9250 Register Map and Descriptions Revision 4.2 18/09/2013
//
// Accelerometer - 16bit resolution
// Gyroscope     - 16bit resolution
// Magnet        - 16bit resolution access via the auxiliary I2C bus
//
// The class intentionally reads one register at a time. Reading all registers together
// is available in other public classes. 
//

//---------- ENUMs -----------------------

enum mpu9250_gyro_range
{
    GYRO_RANGE_250DPS,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
};

enum mpu9250_accel_range
{
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G    
};

//-------------------------------------

  float ax, ay, az;
  float gx, gy, gz;
  float hx, hy, hz, t;
  int beginStatus;

  uint8_t _i2cAddress;
  uint8_t _bus;
  i2c_pins _pins;
  i2c_pullup _pullups;
  
  float _accelScale;
  float _gyroScale;
  float _magScaleX, _magScaleY, _magScaleZ;
  const float _tempScale = 333.87f;
  const float _tempOffset = 21.0f;

  // i2c bus frequency
  const uint32_t _i2cRate = 400000;

  // constants
  const float G = 9.807f;
  const float _deg2rad = 3.14159265359f/180.0f;

  // MPU9250 registers
  const uint8_t _REGISTER_ACCELERO_X = 0x3B;
  const uint8_t _REGISTER_GYRO_X = 0x43;
  const uint8_t _REGISTER_TEMP = 0x41;
  const uint8_t _REGISTER_EXT_SENS_DATA_00 = 0x49;

  const uint8_t _REGISTER_ACCEL_CONFIG = 0x1C;
  const uint8_t ACCEL_FS_SEL_2G = 0x00;
  const uint8_t ACCEL_FS_SEL_4G = 0x08;
  const uint8_t ACCEL_FS_SEL_8G = 0x10;
  const uint8_t ACCEL_FS_SEL_16G = 0x18;

  const uint8_t _REGISTER_GYRO_CONFIG = 0x1B;
  const uint8_t GYRO_FS_SEL_250DPS = 0x00;
  const uint8_t GYRO_FS_SEL_500DPS = 0x08;
  const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
  const uint8_t GYRO_FS_SEL_2000DPS = 0x18;

  const uint8_t SMPDIV = 0x19;

  const uint8_t _REGISTER_INT_PIN_CFG = 0x37;
  //const uint8_t INT_ENABLE = 0x38;
  const uint8_t INT_DISABLE = 0x00;
  const uint8_t INT_PULSE_50US = 0x00;
  const uint8_t INT_RAW_RDY_EN = 0x01;

  const uint8_t _REGISTER_PWR_MGMT_1 = 0x6B;
  const uint8_t PWR_RESET = 0x80;
  const uint8_t CLOCK_SEL_PLL = 0x01;

  const uint8_t _REGISTER_PWR_MGMT_2 = 0x6C;
  const uint8_t SEN_ENABLE = 0x00;

  const uint8_t _REGISTER_USER_CTRL = 0x6A;
  const uint8_t I2C_MST_EN = 0x20;
  const uint8_t I2C_MST_CLK = 0x0D;
  const uint8_t _REGISTER_MST_CTRL = 0x24;
  const uint8_t _REGISTER_SLV0_ADDR = 0x25;
  const uint8_t _REGISTER_SLV0_REG = 0x26;
  const uint8_t _REGISTER_SLV0_DO = 0x63;
  const uint8_t _REGISTER_SLV0_CTRL = 0x27;
  const uint8_t I2C_SLV0_EN = 0x80;
  const uint8_t I2C_READ_FLAG = 0x80;

  const uint8_t WHO_AM_I = 0x75;

  // AK8963 registers
  const uint8_t _i2cAddress_Magnet = 0x0C;

  const uint8_t _REGISTER_AK8963_HXL = 0x03; 

  const uint8_t _REGISTER_MAGNET_CONTROL = 0x0A;
  const uint8_t AK8963_PWR_DOWN = 0x00;
  const uint8_t AK8963_CNT_MEAS1 = 0x12;
  const uint8_t AK8963_CNT_MEAS2 = 0x16;
  const uint8_t AK8963_FUSE_ROM = 0x0F;

  const uint8_t _REGISTER_AK8963_CNTL2 = 0x0B;
  const uint8_t AK8963_RESET = 0x01;

  const uint8_t _REGISTER_AK8963_ASA = 0x10;

  const uint8_t AK8963_WHO_AM_I = 0x00;

  // transformation matrix
  const int16_t tX[3] = {0,  1,  0}; 
  const int16_t tY[3] = {1,  0,  0};
  const int16_t tZ[3] = {0,  0, -1};

//------------------------------


void setup() {

  _i2cAddress = 0x68;      // I2C address
  _bus = 0;             // I2C bus
  
  // serial to display data
  Serial.begin(115200);

  beginStatus = begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
}

void loop() {

  // get the accel (m/s^2), gyro (rad/s), and magnetometer (uT), and temperature (C) data
  getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
  
  // print the data
  Serial.print("$ACC,");
  Serial.print(ax,6);
  Serial.print(",");
  Serial.print(ay,6);
  Serial.print(",");
  Serial.print(az,6);
  Serial.print("\t");

  Serial.print("$GYR,");
  Serial.print(gx,6);
  Serial.print(",");
  Serial.print(gy,6);
  Serial.print(",");
  Serial.print(gz,6);
  Serial.print("\t");

  Serial.print("$MAG,");
  Serial.print(hx,6);
  Serial.print(",");
  Serial.print(hy,6);
  Serial.print(",");
  Serial.print(hz,6);
  Serial.print("\t");

  Serial.print("$TMP,");
  Serial.println(t,6);
  
  // Wait 50ms
  delay(50);

}

void getMotion10Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz, int16_t* t){
    
  uint8_t buff[21];
  int16_t axx, ayy, azz, gxx, gyy, gzz;

  // Download the data from the MPU9250
  readRegisters(_REGISTER_ACCELERO_X, sizeof(buff), &buff[0]); 

  axx = (((int16_t)buff[0]) << 8) | buff[1];
  ayy = (((int16_t)buff[2]) << 8) | buff[3];
  azz = (((int16_t)buff[4]) << 8) | buff[5];

  *t = (((int16_t)buff[6]) << 8) | buff[7];

  gxx = (((int16_t)buff[8]) << 8) | buff[9];
  gyy = (((int16_t)buff[10]) << 8) | buff[11];
  gzz = (((int16_t)buff[12]) << 8) | buff[13];

  *hx = (((int16_t)buff[15]) << 8) | buff[14];
  *hy = (((int16_t)buff[17]) << 8) | buff[16];
  *hz = (((int16_t)buff[19]) << 8) | buff[18];

  // transform axes
  *ax = tX[0]*axx + tX[1]*ayy + tX[2]*azz;
  *ay = tY[0]*axx + tY[1]*ayy + tY[2]*azz;
  *az = tZ[0]*axx + tZ[1]*ayy + tZ[2]*azz;

  *gx = tX[0]*gxx + tX[1]*gyy + tX[2]*gzz;
  *gy = tY[0]*gxx + tY[1]*gyy + tY[2]*gzz;
  *gz = tZ[0]*gxx + tZ[1]*gyy + tZ[2]*gzz;
}

void getMotion10(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz, float* t){
  
  int16_t accel[3];
  int16_t gyro[3];
  int16_t mag[3];
  int16_t tempCount;

  getMotion10Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2], &mag[0], &mag[1], &mag[2], &tempCount);

  *ax = ((float) accel[0]) * _accelScale;
  *ay = ((float) accel[1]) * _accelScale;
  *az = ((float) accel[2]) * _accelScale;

  *gx = ((float) gyro[0]) * _gyroScale;
  *gy = ((float) gyro[1]) * _gyroScale;
  *gz = ((float) gyro[2]) * _gyroScale;

  *hx = ((float) mag[0]) * _magScaleX;
  *hy = ((float) mag[1]) * _magScaleY;
  *hz = ((float) mag[2]) * _magScaleZ;

  *t = (( ((float) tempCount) - _tempOffset )/_tempScale) + _tempOffset; 
}

void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){

  // open the device
  i2c_t3(_bus).beginTransmission(_i2cAddress); 
  
  // specify the starting register address
  i2c_t3(_bus).write(subAddress); 

  //If false, endTransmission() sends a restart message after transmission. The bus will not be released, 
  //which prevents another master device from transmitting between messages. This allows one master device 
  //to send multiple transmissions while in control. The default value is true.
  i2c_t3(_bus).endTransmission(false);

  // specify the number of bytes to receive
  i2c_t3(_bus).requestFrom(_i2cAddress, count); 
  
  uint8_t i = 0; 

  // read the data into the buffer
  while( i2c_t3(_bus).available() ){
    dest[i++] = i2c_t3(_bus).readByte();
  }
}

void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){

  // set slave 0 to the AK8963 and set for read
  writeRegister(_REGISTER_SLV0_ADDR, _i2cAddress_Magnet | I2C_READ_FLAG);

  // set the register to the desired AK8963 sub address
  writeRegister(_REGISTER_SLV0_REG, subAddress); 

  // enable I2C and request the bytes
  writeRegister(_REGISTER_SLV0_CTRL, I2C_SLV0_EN | count); 
  
  // takes some time for these registers to fill
  delayMicroseconds(100); 

  // read the bytes off the MPU9250 EXT_SENS_DATA registers
  readRegisters(_REGISTER_EXT_SENS_DATA_00, count, dest); 
}

bool writeRegister(uint8_t subAddress, uint8_t data){
    
  uint8_t buff[1];

  // open the device
  i2c_t3(_bus).beginTransmission(_i2cAddress); 

  // write the register address
  i2c_t3(_bus).write(subAddress); 

  // write the data
  i2c_t3(_bus).write(data); 
  
  i2c_t3(_bus).endTransmission();

  //Wait for 10ms
  delay(10);

  //Read the register
  readRegisters(subAddress,sizeof(buff),&buff[0]);

  //Check the data read back is correct
  if(buff[0] == data)  return true;
 
  return false;
  
}

bool writeAK8963Register(uint8_t subAddress, uint8_t data){
  
  uint8_t count = 1;
  uint8_t buff[1];

  // set slave 0 to the AK8963 and set for write
  writeRegister(_REGISTER_SLV0_ADDR, _i2cAddress_Magnet); 

  // set the register to the desired AK8963 sub address
  writeRegister(_REGISTER_SLV0_REG, subAddress); 

  // store the data for write
  writeRegister(_REGISTER_SLV0_DO, data); 

  // enable I2C and send 1 byte
  writeRegister(_REGISTER_SLV0_CTRL, I2C_SLV0_EN | count); 

  // read the register and confirm
  readAK8963Registers(subAddress, sizeof(buff), &buff[0]);

  if(buff[0] == data)  return true;
  return false;
  
}

//------------------------------ WHO AM I ------------------

uint8_t whoAmI(){
  
    uint8_t buff[1];

    // read the WHO AM I register
    readRegisters(WHO_AM_I,sizeof(buff),&buff[0]);

    // return the register value
    return buff[0];
}

uint8_t whoAmIAK8963(){
  
    uint8_t buff[1];

    // read the WHO AM I register
    readAK8963Registers(AK8963_WHO_AM_I,sizeof(buff),&buff[0]);

    // return the register value
    return buff[0];
}

//----------------------------- BEGIN ----------------------

/* starts I2C communication and sets up the MPU-9250 */
int begin(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange){
    
  uint8_t buff[3];
  uint8_t data[7];
 
  // starting the I2C bus
  i2c_t3(_bus).begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, _i2cRate);

  // This register allows the user to configure the clock source - page 38
  // 0 = Internal 8MHz oscillator
  // 1 = PLL with X axis gyroscope reference
  // 2 = PLL with Y axis gyroscope reference
  // 3 = PLL with Z axis gyroscope reference
  // 4 = PLL with external 32.768kHz reference
  // 5 = PLL with external 19.2MHz reference
  if( !writeRegister(_REGISTER_PWR_MGMT_1, CLOCK_SEL_PLL) ) return -1;

  // enable I2C master mode
  if( !writeRegister(_REGISTER_USER_CTRL, I2C_MST_EN) ) return -1;

  // set the I2C bus speed to 400 kHz
  if( !writeRegister(_REGISTER_MST_CTRL, I2C_MST_CLK) ) return -1;

  // set AK8963 to Power Down
  if( !writeAK8963Register(_REGISTER_MAGNET_CONTROL, AK8963_PWR_DOWN) ) return -1;

  // reset the MPU9250
  writeRegister(_REGISTER_PWR_MGMT_1,PWR_RESET);

  // wait for MPU-9250 to reboot
  delay(100);

  // reset the AK8963
  writeAK8963Register(_REGISTER_AK8963_CNTL2, AK8963_RESET);

  // This register allows the user to configure the clock source - page 38
  // 0 = Internal 8MHz oscillator
  // 1 = PLL with X axis gyroscope reference
  // 2 = PLL with Y axis gyroscope reference
  // 3 = PLL with Z axis gyroscope reference
  // 4 = PLL with external 32.768kHz reference
  // 5 = PLL with external 19.2MHz reference
  if( !writeRegister(_REGISTER_PWR_MGMT_1, CLOCK_SEL_PLL))  return -1;
  
  // check the WHO AM I byte, expected value is 0x71 (decimal 113)
  if( whoAmI() != 113 ) return -1;
  
  // enable accelerometer and gyro
  if( !writeRegister(_REGISTER_PWR_MGMT_2, SEN_ENABLE) )return -1;
 
  // Accelerometer Configuration - page 13
  // 0 = +/-  2g <- highest sensitivity
  // 1 = +/-  4g
  // 2 = +/-  8g
  // 3 = +/- 16g
  switch(accelRange) {

    case ACCEL_RANGE_2G:
            // setting the accel range to 2G
            if( !writeRegister(_REGISTER_ACCEL_CONFIG, ACCEL_FS_SEL_2G) ) return -1;
            _accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
            break;

    case ACCEL_RANGE_4G:
            // setting the accel range to 4G
            if( !writeRegister(_REGISTER_ACCEL_CONFIG, ACCEL_FS_SEL_4G) ) return -1;
            _accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
            break;

    case ACCEL_RANGE_8G:
            // setting the accel range to 8G
            if( !writeRegister(_REGISTER_ACCEL_CONFIG, ACCEL_FS_SEL_8G) ) return -1;
            _accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
            break;

    case ACCEL_RANGE_16G:
            // setting the accel range to 16G
            if( !writeRegister(_REGISTER_ACCEL_CONFIG, ACCEL_FS_SEL_16G) ) return -1;
            _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
            break;
  }

  // Gyroscope Configuration - page 12
  // 0 =  250 degrees/second <- highest sensitivity
  // 1 =  500 degrees/second
  // 2 = 1000 degrees/second
  // 3 = 2000 degrees/second
  switch(gyroRange) {

    case GYRO_RANGE_250DPS:
            // setting the gyro range to 250DPS
            if( !writeRegister(_REGISTER_GYRO_CONFIG, GYRO_FS_SEL_250DPS) )return -1;
            _gyroScale = 250.0f/32767.5f * _deg2rad; // setting the gyro scale to 250DPS
            break;

    case GYRO_RANGE_500DPS:
            // setting the gyro range to 500DPS
            if( !writeRegister(_REGISTER_GYRO_CONFIG, GYRO_FS_SEL_500DPS) ) return -1;
            _gyroScale = 500.0f/32767.5f * _deg2rad; // setting the gyro scale to 500DPS
            break;

    case GYRO_RANGE_1000DPS:
            // setting the gyro range to 1000DPS
            if( !writeRegister(_REGISTER_GYRO_CONFIG, GYRO_FS_SEL_1000DPS) ) return -1;
            _gyroScale = 1000.0f/32767.5f * _deg2rad; // setting the gyro scale to 1000DPS
            break;

    case GYRO_RANGE_2000DPS:
            // setting the gyro range to 2000DPS
            if( !writeRegister(_REGISTER_GYRO_CONFIG, GYRO_FS_SEL_2000DPS) )return -1;
            _gyroScale = 2000.0f/32767.5f * _deg2rad; // setting the gyro scale to 2000DPS
            break;
  }

  // enable I2C master mode
  if( !writeRegister(_REGISTER_USER_CTRL, I2C_MST_EN) )return -1;
  
  // set the I2C bus speed to 400 kHz
  if( !writeRegister(_REGISTER_MST_CTRL, I2C_MST_CLK) ) return -1;

  // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
  if( whoAmIAK8963() != 72 ) return -1;

  // get the magnetometer calibration

  // set AK8963 to Power Down
  if( !writeAK8963Register(_REGISTER_MAGNET_CONTROL, AK8963_PWR_DOWN) )return -1;

  // Wait 100ms
  delay(100);

  // set AK8963 to FUSE ROM access
  if( !writeAK8963Register(_REGISTER_MAGNET_CONTROL, AK8963_FUSE_ROM) )return -1;
  
  // Wait 100ms
  delay(100);

  // read the AK8963 ASA registers and compute magnetometer scale factors
  readAK8963Registers(_REGISTER_AK8963_ASA,sizeof(buff),&buff[0]);
  
  _magScaleX = ((((float)buff[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleY = ((((float)buff[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleZ = ((((float)buff[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla 

  // set AK8963 to Power Down
  if( !writeAK8963Register(_REGISTER_MAGNET_CONTROL, AK8963_PWR_DOWN) ) return -1;
 
  // Wait 100ms
  delay(100);  

  // set AK8963 to 16 bit resolution, 100 Hz update rate
  if( !writeAK8963Register(_REGISTER_MAGNET_CONTROL, AK8963_CNT_MEAS2) )return -1;
  
  // Wait 100ms
  delay(100);

  // This register allows the user to configure the clock source - page 38
  // 0 = Internal 8MHz oscillator
  // 1 = PLL with X axis gyroscope reference
  // 2 = PLL with Y axis gyroscope reference
  // 3 = PLL with Z axis gyroscope reference
  // 4 = PLL with external 32.768kHz reference
  // 5 = PLL with external 19.2MHz reference
  if( !writeRegister(_REGISTER_PWR_MGMT_1, CLOCK_SEL_PLL) )return -1;
  
  // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
  readAK8963Registers(_REGISTER_AK8963_HXL,sizeof(data),&data[0]);

  // successful init, return 0
  return 0;
}
