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
