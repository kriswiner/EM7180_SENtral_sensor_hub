#ifndef Globals_h
#define Globals_h

/*************************************************************************************************/
/*************                                                                     ***************/
/*************                        Parameter Definitions                        ***************/
/*************                                                                     ***************/
/*************************************************************************************************/

// These are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Kp 2.0f * 5.0f
#define Ki 0.0f

// BMP280 registers
#define BMP280_TEMP_XLSB  0xFC
#define BMP280_TEMP_LSB   0xFB
#define BMP280_TEMP_MSB   0xFA
#define BMP280_PRESS_XLSB 0xF9
#define BMP280_PRESS_LSB  0xF8
#define BMP280_PRESS_MSB  0xF7
#define BMP280_CONFIG     0xF5
#define BMP280_CTRL_MEAS  0xF4
#define BMP280_STATUS     0xF3
#define BMP280_RESET      0xE0
#define BMP280_ID         0xD0  // should be 0x58
#define BMP280_CALIB00    0x88

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_XOUT_H  0x04
#define AK8963_YOUT_L  0x05
#define AK8963_YOUT_H  0x06
#define AK8963_ZOUT_L  0x07
#define AK8963_ZOUT_H  0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// EM7180 SENtral register map
// see http://www.emdeveloper.com/downloads/7180/EMSentral_EM7180_Register_Map_v1_3.pdf
//
#define EM7180_QX                 0x00  // this is a 32-bit normalized floating point number read from registers 0x00-03
#define EM7180_QY                 0x04  // this is a 32-bit normalized floating point number read from registers 0x04-07
#define EM7180_QZ                 0x08  // this is a 32-bit normalized floating point number read from registers 0x08-0B
#define EM7180_QW                 0x0C  // this is a 32-bit normalized floating point number read from registers 0x0C-0F
#define EM7180_QTIME              0x10  // this is a 16-bit unsigned integer read from registers 0x10-11
#define EM7180_MX                 0x12  // int16_t from registers 0x12-13
#define EM7180_MY                 0x14  // int16_t from registers 0x14-15
#define EM7180_MZ                 0x16  // int16_t from registers 0x16-17
#define EM7180_MTIME              0x18  // uint16_t from registers 0x18-19
#define EM7180_AX                 0x1A  // int16_t from registers 0x1A-1B
#define EM7180_AY                 0x1C  // int16_t from registers 0x1C-1D
#define EM7180_AZ                 0x1E  // int16_t from registers 0x1E-1F
#define EM7180_ATIME              0x20  // uint16_t from registers 0x20-21
#define EM7180_GX                 0x22  // int16_t from registers 0x22-23
#define EM7180_GY                 0x24  // int16_t from registers 0x24-25
#define EM7180_GZ                 0x26  // int16_t from registers 0x26-27
#define EM7180_GTIME              0x28  // uint16_t from registers 0x28-29
#define EM7180_Baro               0x2A  // start of two-byte MS5637 pressure data, 16-bit signed interger
#define EM7180_BaroTIME           0x2C  // start of two-byte MS5637 pressure timestamp, 16-bit unsigned
#define EM7180_Temp               0x2E  // start of two-byte MS5637 temperature data, 16-bit signed interger
#define EM7180_TempTIME           0x30  // start of two-byte MS5637 temperature timestamp, 16-bit unsigned
#define EM7180_QRateDivisor       0x32  // uint8_t 
#define EM7180_EnableEvents       0x33
#define EM7180_HostControl        0x34
#define EM7180_EventStatus        0x35
#define EM7180_SensorStatus       0x36
#define EM7180_SentralStatus      0x37
#define EM7180_AlgorithmStatus    0x38
#define EM7180_FeatureFlags       0x39
#define EM7180_ParamAcknowledge   0x3A
#define EM7180_SavedParamByte0    0x3B
#define EM7180_SavedParamByte1    0x3C
#define EM7180_SavedParamByte2    0x3D
#define EM7180_SavedParamByte3    0x3E
#define EM7180_ActualMagRate      0x45
#define EM7180_ActualAccelRate    0x46
#define EM7180_ActualGyroRate     0x47
#define EM7180_ActualBaroRate     0x48
#define EM7180_ActualTempRate     0x49
#define EM7180_ErrorRegister      0x50
#define EM7180_AlgorithmControl   0x54
#define EM7180_MagRate            0x55
#define EM7180_AccelRate          0x56
#define EM7180_GyroRate           0x57
#define EM7180_BaroRate           0x58
#define EM7180_TempRate           0x59
#define EM7180_LoadParamByte0     0x60
#define EM7180_LoadParamByte1     0x61
#define EM7180_LoadParamByte2     0x62
#define EM7180_LoadParamByte3     0x63
#define EM7180_ParamRequest       0x64
#define EM7180_ROMVersion1        0x70
#define EM7180_ROMVersion2        0x71
#define EM7180_RAMVersion1        0x72
#define EM7180_RAMVersion2        0x73
#define EM7180_ProductID          0x90
#define EM7180_RevisionID         0x91
#define EM7180_RunStatus          0x92
#define EM7180_UploadAddress      0x94 // uint16_t registers 0x94 (MSB)-5(LSB)
#define EM7180_UploadData         0x96  
#define EM7180_CRCHost            0x97  // uint32_t from registers 0x97-9A
#define EM7180_ResetRequest       0x9B   
#define EM7180_PassThruStatus     0x9E   
#define EM7180_PassThruControl    0xA0
#define EM7180_ACC_LPF_BW         0x5B  //Register GP36
#define EM7180_GYRO_LPF_BW        0x5C  //Register GP37
#define EM7180_BARO_LPF_BW        0x5D  //Register GP38
#define EM7180_GP36               0x5B
#define EM7180_GP37               0x5C
#define EM7180_GP38               0x5D
#define EM7180_GP39               0x5E
#define EM7180_GP40               0x5F
#define EM7180_GP50               0x69
#define EM7180_GP51               0x6A
#define EM7180_GP52               0x6B
#define EM7180_GP53               0x6C
#define EM7180_GP54               0x6D
#define EM7180_GP55               0x6E
#define EM7180_GP56               0x6F

#define EM7180_ADDRESS           0x28   // Address of the EM7180 SENtral sensor hub
#define M24512DFM_DATA_ADDRESS   0x50   // Address of the 500 page M24512DRC EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
#define M24512DFM_IDPAGE_ADDRESS 0x58   // Address of the single M24512DRC lockable EEPROM ID page
#define MPU9250_ADDRESS          0x68   // Device address of MPU9250 when ADO = 0
#define AK8963_ADDRESS           0x0C   // Address of magnetometer
#define BMP280_ADDRESS           0x76   // Address of BMP280 altimeter when ADO = 0

/*************************************************************************************************/
/*************                                                                     ***************/
/*************                 Enumerators and Structure Variables                 ***************/
/*************                                                                     ***************/
/*************************************************************************************************/

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

enum Posr {
  P_OSR_00 = 0,  // no op
  P_OSR_01,
  P_OSR_02,
  P_OSR_04,
  P_OSR_08,
  P_OSR_16
};

enum Tosr {
  T_OSR_00 = 0,  // no op
  T_OSR_01,
  T_OSR_02,
  T_OSR_04,
  T_OSR_08,
  T_OSR_16
};

enum IIRFilter {
  full = 0,  // bandwidth at full sample rate
  BW0_223ODR,
  BW0_092ODR,
  BW0_042ODR,
  BW0_021ODR // bandwidth at 0.021 x sample rate
};

enum Mode {
  BMP280Sleep = 0,
  forced,
  forced2,
  normal
};

enum SBy {
  t_00_5ms = 0,
  t_62_5ms,
  t_125ms,
  t_250ms,
  t_500ms,
  t_1000ms,
  t_2000ms,
  t_4000ms,
};

struct global_conf_t
{
  uint8_t currentSet;
  int16_t accZero_max[3];
  int16_t accZero_min[3];
  int16_t magZero[3];
  int16_t grav;
  uint8_t checksum; // Last position in structure
};

struct Sentral_WS_params
{
  uint8_t Sen_param[35][4];
};

/*************************************************************************************************/
/*************                                                                     ***************/
/*************                        Global Scope Variables                       ***************/
/*************                                                                     ***************/
/*************************************************************************************************/

// General purpose variables
int16_t                                 serial_input;
static int16_t                          warm_start = 0;
static int16_t                          warm_start_saved = 0;

// Specify BMP280 configuration
uint8_t                                 Posr = P_OSR_16;
uint8_t                                 Tosr = T_OSR_02;
uint8_t                                 Mode = normal;
uint8_t                                 IIRFilter = BW0_042ODR;
uint8_t                                 SBy = t_62_5ms;

// t_fine carries fine temperature as global value for BMP280
int32_t                                 t_fine;

// Specify sensor full scale
uint8_t                                 Gscale = GFS_250DPS;
uint8_t                                 Ascale = AFS_2G;

// Choose either 14-bit or 16-bit magnetometer resolution
uint8_t                                 Mscale = MFS_16BITS;

// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
uint8_t                                 Mmode = 0x02;

// scale resolutions per LSB for the sensors
float                                   aRes;
float                                   gRes;
float                                   mRes;

// Pin definitions
// These can be changed, 2 and 3 are the Arduinos ext int pins
int                                     intPin = 8;

// LED on the Teensy 3.1
int                                     myLed = 13;

// BMP280 compensation parameters
uint16_t                                dig_T1;
uint16_t                                dig_P1;
int16_t                                 dig_T2;
int16_t                                 dig_T3;
int16_t                                 dig_P2;
int16_t                                 dig_P3;
int16_t                                 dig_P4;
int16_t                                 dig_P5;
int16_t                                 dig_P6;
int16_t                                 dig_P7;
int16_t                                 dig_P8;
int16_t                                 dig_P9;

// stores BMP280 pressures sensor pressure and temperature
double                                  Temperature;
double                                  Pressure;

// pressure and temperature raw count output for BMP280
int32_t                                 rawPress;
int32_t                                 rawTemp;

// MPU9250 variables
// Stores the 16-bit signed accelerometer sensor output
int16_t                                 accelCount[3];

// Stores the 16-bit signed gyro sensor output
int16_t                                 gyroCount[3];

// Stores the 16-bit signed magnetometer sensor output
int16_t                                 magCount[3];

// quaternion data register
float                                   Quat[4] = {0, 0, 0, 0};

// Factory mag calibration and mag bias
float                                   magCalibration[3] = {0, 0, 0};

// Bias corrections for gyro, accelerometer, mag
float                                   gyroBias[3] = {0, 0, 0};
float                                   accelBias[3] = {0, 0, 0};
float                                   magBias[3] = {0, 0, 0};
float                                   magScale[3]  = {0, 0, 0};

// Pressure, temperature raw count output
int16_t                                 tempCount;
int16_t                                 rawPressure;
int16_t                                 rawTemperature;

// Stores the MPU9250 internal chip temperature in degrees Celsius
float                                   temperature;
float                                   pressure;
float                                   altitude;

// holds results of gyro and accelerometer self test
float SelfTest[6];

// Global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
// Gyroscope measurement error in rads/s (start at 40 deg/s)
float                                   GyroMeasError = PI * (40.0f / 180.0f);

// Gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float                                   GyroMeasDrift = PI * (0.0f  / 180.0f);

// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.

// Compute beta
float                                   beta = sqrt(3.0f / 4.0f) * GyroMeasError;

// Compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float                                   zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;

// Used to control display output rate
uint32_t                                delt_t = 0;
uint32_t                                count = 0;
uint32_t                                sumCount = 0;
float                                   pitch;
float                                   yaw;
float                                   roll;
float                                   Yaw;
float                                   Pitch;
float                                   Roll;

// Integration interval for both filter schemes
float                                   deltat = 0.0f;
float                                   sum = 0.0f;

// used to calculate integration interval
uint32_t                                lastUpdate = 0;
uint32_t                                firstUpdate = 0;

// used to calculate integration interval
uint32_t                                Now = 0;

// used for param transfer
uint8_t                                 param[4];

// EM7180 sensor full scale ranges
uint16_t                                EM7180_mag_fs;
uint16_t                                EM7180_acc_fs;
uint16_t                                EM7180_gyro_fs;

// variables to hold latest sensor data values 
float                                   ax;
float                                   ay;
float                                   az;
float                                   gx;
float                                   gy;
float                                   gz;
float                                   mx;
float                                   my;
float                                   mz;

// Vector to hold quaternion
float                                   q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// Vector to hold integral error for Mahony method
float                                   eInt[3] = {0.0f, 0.0f, 0.0f};

global_conf_t                           global_conf;
Sentral_WS_params                       WS_params;

#endif // Globals_h
