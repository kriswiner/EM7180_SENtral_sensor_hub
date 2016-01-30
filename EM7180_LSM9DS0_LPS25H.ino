/* EM7180_LSM9DS0_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: January 24, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 The EM7180 SENtral sensor hub is not a motion sensor, but rather takes raw sensor data from a variety of motion sensors,
 in this case the LSM9DS0, and does sensor fusion with quaternions as its output. The SENtral loads firmware from the
 on-board M24512DFMC 512 kbit EEPROM upon startup, configures and manages the sensors on its dedicated master I2C bus,
 and outputs scaled sensor data (accelerations, rotation rates, and magnetic fields) as well as quaternions and
 heading/pitch/roll, if selected.
 
 This sketch demonstrates basic EM7180 SENtral functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms to compare with the hardware sensor fusion results.
 Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is specifically for the Teensy 3.1 Mini Add-On shield with the EM7180 SENtral sensor hub as master,
 the LSM9DS0 9-axis motion sensor (accel/gyro/mag) as slave, an LPS25H pressure/temperature sensor, and an M24512DFM
 512kbit (64 kByte) EEPROM as slave all connected via I2C. The SENtral can use the pressure data in the sensor fusion
 and there is currently a driver for the LPS25H in the SENtral firmware. 
 
 This sketch uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The LPS25H is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application. The LPS25H is connected to the EM7180 I2C master bus and has an interrupt 
 connected to the EM7180 just like the LSM9DS0. The driver will use the data ready interrupt from the LPS25H to signal
 to the EM7180 that it should read and process the pressure/temperature data.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4k7 resistors are on the EM7180+LSM9DS0+LPS25H+M24512DFM Mini Add-On board for Teensy 3.1.
 
 Hardware setup:
 EM7180 Mini Add-On ------- Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- 17
 SCL ----------------------- 16
 GND ---------------------- GND
 INT------------------------ 8
 
 Note: The LSM9DS0 is an I2C sensor and uses the Teensy 3.1 i2c_t3.h Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 */
//#include "Wire.h"   
#include <i2c_t3.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 3 - LCD chip select (SCE)
// pin 4 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 3, 4);

// See LPS25H "MEMS pressure sensor: 260-1260 hPa absolute digital output barometer" Data Sheet
#define LPS25H_REF_P_XL      0x08
#define LPS25H_REF_P_L       0x09
#define LPS25H_REF_P_H       0x0A
#define LPS25H_WHOAMI        0x0F // should return 0xBD
#define LPS25H_RES_CONF      0x10
#define LPS25H_CTRL_REG1     0x20
#define LPS25H_CTRL_REG2     0x21
#define LPS25H_CTRL_REG3     0x22
#define LPS25H_CTRL_REG4     0x23
#define LPS25H_INT_CFG       0x24
#define LPS25H_INT_SOURCE    0x25
#define LPS25H_STATUS_REG    0x27
#define LPS25H_PRESS_OUT_XL  0x28
#define LPS25H_PRESS_OUT_L   0x29
#define LPS25H_PRESS_OUT_H   0x2A
#define LPS25H_TEMP_OUT_L    0x2B
#define LPS25H_TEMP_OUT_H    0x2C
#define LPS25H_FIFO_CTRL     0x2E
#define LPS25H_FIFO_STATUS   0x2F
#define LPS25H_THS_P_L       0x30
#define LPS25H_THS_P_H       0x31
#define LPS25H_RPDS_L        0x39
#define LPS25H_RPDS_H        0x3A

// See also LSM9DS0 Register Map and Descriptions,http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00087365.pdf
//
////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define  LSM9DS0G_WHO_AM_I_G		0x0F
#define  LSM9DS0G_CTRL_REG1_G		0x20
#define  LSM9DS0G_CTRL_REG2_G		0x21
#define  LSM9DS0G_CTRL_REG3_G		0x22
#define  LSM9DS0G_CTRL_REG4_G		0x23
#define  LSM9DS0G_CTRL_REG5_G		0x24
#define  LSM9DS0G_REFERENCE_G		0x25
#define  LSM9DS0G_STATUS_REG_G		0x27
#define  LSM9DS0G_OUT_X_L_G		0x28
#define  LSM9DS0G_OUT_X_H_G		0x29
#define  LSM9DS0G_OUT_Y_L_G		0x2A
#define  LSM9DS0G_OUT_Y_H_G		0x2B
#define  LSM9DS0G_OUT_Z_L_G		0x2C
#define  LSM9DS0G_OUT_Z_H_G		0x2D
#define  LSM9DS0G_FIFO_CTRL_REG_G	0x2E
#define  LSM9DS0G_FIFO_SRC_REG_G	0x2F
#define  LSM9DS0G_INT1_CFG_G		0x30
#define  LSM9DS0G_INT1_SRC_G		0x31
#define  LSM9DS0G_INT1_THS_XH_G		0x32
#define  LSM9DS0G_INT1_THS_XL_G		0x33
#define  LSM9DS0G_INT1_THS_YH_G		0x34
#define  LSM9DS0G_INT1_THS_YL_G		0x35
#define  LSM9DS0G_INT1_THS_ZH_G		0x36
#define  LSM9DS0G_INT1_THS_ZL_G		0x37
#define  LSM9DS0G_INT1_DURATION_G	0x38

//////////////////////////////////////////
//  LSM9DS0XM Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define  LSM9DS0XM_OUT_TEMP_L_XM	0x05
#define  LSM9DS0XM_OUT_TEMP_H_XM	0x06
#define  LSM9DS0XM_STATUS_REG_M		0x07
#define  LSM9DS0XM_OUT_X_L_M		0x08
#define  LSM9DS0XM_OUT_X_H_M		0x09
#define  LSM9DS0XM_OUT_Y_L_M		0x0A
#define  LSM9DS0XM_OUT_Y_H_M		0x0B
#define  LSM9DS0XM_OUT_Z_L_M		0x0C
#define  LSM9DS0XM_OUT_Z_H_M		0x0D
#define  LSM9DS0XM_WHO_AM_I_XM		0x0F
#define  LSM9DS0XM_INT_CTRL_REG_M	0x12
#define  LSM9DS0XM_INT_SRC_REG_M	0x13
#define  LSM9DS0XM_INT_THS_L_M		0x14
#define  LSM9DS0XM_INT_THS_H_M		0x15
#define  LSM9DS0XM_OFFSET_X_L_M		0x16
#define  LSM9DS0XM_OFFSET_X_H_M		0x17
#define  LSM9DS0XM_OFFSET_Y_L_M		0x18
#define  LSM9DS0XM_OFFSET_Y_H_M		0x19
#define  LSM9DS0XM_OFFSET_Z_L_M		0x1A
#define  LSM9DS0XM_OFFSET_Z_H_M		0x1B
#define  LSM9DS0XM_REFERENCE_X		0x1C
#define  LSM9DS0XM_REFERENCE_Y		0x1D
#define  LSM9DS0XM_REFERENCE_Z		0x1E
#define  LSM9DS0XM_CTRL_REG0_XM		0x1F
#define  LSM9DS0XM_CTRL_REG1_XM		0x20
#define  LSM9DS0XM_CTRL_REG2_XM		0x21
#define  LSM9DS0XM_CTRL_REG3_XM		0x22
#define  LSM9DS0XM_CTRL_REG4_XM		0x23
#define  LSM9DS0XM_CTRL_REG5_XM		0x24
#define  LSM9DS0XM_CTRL_REG6_XM		0x25
#define  LSM9DS0XM_CTRL_REG7_XM		0x26
#define  LSM9DS0XM_STATUS_REG_A		0x27
#define  LSM9DS0XM_OUT_X_L_A		0x28
#define  LSM9DS0XM_OUT_X_H_A		0x29
#define  LSM9DS0XM_OUT_Y_L_A		0x2A
#define  LSM9DS0XM_OUT_Y_H_A		0x2B
#define  LSM9DS0XM_OUT_Z_L_A		0x2C
#define  LSM9DS0XM_OUT_Z_H_A		0x2D
#define  LSM9DS0XM_FIFO_CTRL_REG	0x2E
#define  LSM9DS0XM_FIFO_SRC_REG		0x2F
#define  LSM9DS0XM_INT_GEN_1_REG	0x30
#define  LSM9DS0XM_INT_GEN_1_SRC	0x31
#define  LSM9DS0XM_INT_GEN_1_THS	0x32
#define  LSM9DS0XM_INT_GEN_1_DURATION	0x33
#define  LSM9DS0XM_INT_GEN_2_REG	0x34
#define  LSM9DS0XM_INT_GEN_2_SRC	0x35
#define  LSM9DS0XM_INT_GEN_2_THS	0x36
#define  LSM9DS0XM_INT_GEN_2_DURATION	0x37
#define  LSM9DS0XM_CLICK_CFG		0x38
#define  LSM9DS0XM_CLICK_SRC		0x39
#define  LSM9DS0XM_CLICK_THS		0x3A
#define  LSM9DS0XM_TIME_LIMIT		0x3B
#define  LSM9DS0XM_TIME_LATENCY		0x3C
#define  LSM9DS0XM_TIME_WINDOW		0x3D
#define  LSM9DS0XM_ACT_THS		0x3E
#define  LSM9DS0XM_ACT_DUR		0x3F


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
#define EM7180_Baro               0x2A  // start of two-byte LPS25H pressure data, 16-bit signed interger
#define EM7180_BaroTIME           0x2C  // start of two-byte LPS25H pressure timestamp, 16-bit unsigned
#define EM7180_Temp               0x2E  // start of two-byte LPS25H temperature data, 16-bit signed interger
#define EM7180_TempTIME           0x30  // start of two-byte LPS25H temperature timestamp, 16-bit unsigned
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

// Using the Teensy Mini Add-On board, LSM9DS0 SDOG = SDOXM = GND as designed
// Seven-bit LSM9DS0 device addresses are ACC = 0x1E, GYRO = 0x6A, MAG = 0x1E

// Using the EM7180+LSM9DS0+LPS25H Teensy 3.1 Add-On shield, ADO is set to 0 
#define ADO 0
#if ADO
#define LSM9DS0XM_ADDRESS        0x1D // Address of accel/magnetometer when ADO = 1
#define LSM9DS0G_ADDRESS         0x6B // Address of gyro when ADO = 1
#else 
#define LSM9DS0XM_ADDRESS        0x1E // Address of accel/magnetometer when ADO = 0
#define LSM9DS0G_ADDRESS         0x6A // Address of gyro when ADO = 0
#endif
#define LPS25H_ADDRESS           0x5D   // Address of altimeter with LPS25H ADO = 1
#define EM7180_ADDRESS           0x28   // Address of the EM7180 SENtral sensor hub
#define M24512DFM_DATA_ADDRESS   0x50   // Address of the 500 page M24512DFM EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
#define M24512DFM_IDPAGE_ADDRESS 0x58   // Address of the single M24512DFM lockable EEPROM ID page

#define SerialDebug true  // set to true to get Serial output for debugging

enum PODR {  // Altimeter output data rate
  P_1shot = 0,
  P_1Hz,
  P_7Hz,
  P_12Hz,  // 12.5 Hz output data rate
  P_25Hz
};

enum Pavg {     // Altimeter pressure internal data averaging
  P_avg_8 = 0,  // average pressure data 8 times  
  P_avg_32,     // average pressure data 32 times 
  P_avg_128,    // average pressure data 128 times 
  P_avg_512     // average pressure data 32 times 
};

enum Tavg {     // Altimeter temperature internal data averaging
  T_avg_8 = 0,  // average temperature data 8 times  
  T_avg_16,     // average temperature data 16 times 
  T_avg_32,     // average temperature data 32 times 
  T_avg_64      // average temperature data 64 times 
};

// Set initial input parameters
enum Ascale {  // set of allowable accel full scale settings
  AFS_2G = 0,
  AFS_4G,
  AFS_6G,
  AFS_8G,
  AFS_16G
};

enum Aodr {  // set of allowable gyro sample rates
  AODR_PowerDown = 0,
  AODR_3_125Hz,
  AODR_6_25Hz,
  AODR_12_5Hz,
  AODR_25Hz,
  AODR_50Hz,
  AODR_100Hz,
  AODR_200Hz,
  AODR_400Hz,
  AODR_800Hz,
  AODR_1600Hz
};

enum Abw {  // set of allowable accewl bandwidths
   ABW_773Hz = 0,
   ABW_194Hz,
   ABW_362Hz,
   ABW_50Hz
};

enum Gscale {  // set of allowable gyro full scale settings
  GFS_245DPS = 0,
  GFS_500DPS,
  GFS_NoOp,
  GFS_2000DPS
};

enum Godr {  // set of allowable gyro sample rates
  GODR_95Hz = 0,
  GODR_190Hz,
  GODR_380Hz,
  GODR_760Hz
};

enum Gbw {   // set of allowable gyro data bandwidths
  GBW_low = 0,  // 12.5 Hz at Godr = 95 Hz, 12.5 Hz at Godr = 190 Hz,  30 Hz at Godr = 760 Hz
  GBW_med,      // 25 Hz   at Godr = 95 Hz, 25 Hz   at Godr = 190 Hz,  35 Hz at Godr = 760 Hz
  GBW_high,     // 25 Hz   at Godr = 95 Hz, 50 Hz   at Godr = 190 Hz,  50 Hz at Godr = 760 Hz
  GBW_highest   // 25 Hz   at Godr = 95 Hz, 70 Hz   at Godr = 190 Hz, 100 Hz at Godr = 760 Hz
};

enum Mscale {  // set of allowable mag full scale settings
  MFS_2G = 0,
  MFS_4G,
  MFS_8G,
  MFS_12G
};

enum Mres {
  MRES_LowResolution = 0, 
  MRES_NoOp,
  MRES_HighResolution
};

enum Modr {  // set of allowable mag sample rates
  MODR_3_125Hz = 0,
  MODR_6_25Hz,
  MODR_12_5Hz,
  MODR_25Hz,
  MODR_50Hz,
  MODR_100Hz
};

// Specify sensor full scale
uint8_t PODR = P_7Hz, Pavg = P_avg_512, Tavg = T_avg_64;     // set LPS25H pressure amd temperature output data rate
uint8_t Gscale = GFS_245DPS; // gyro full scale
uint8_t Godr = GODR_380Hz;   // gyro data sample rate
uint8_t Gbw = GBW_low;       // gyro data bandwidth
uint8_t Ascale = AFS_2G;     // accel full scale
uint8_t Aodr = AODR_400Hz;   // accel data sample rate
uint8_t Abw = ABW_50Hz;      // accel data bandwidth
uint8_t Mscale = MFS_12G;     // mag full scale
uint8_t Modr = MODR_100Hz;    // mag data sample rate
uint8_t Mres = MRES_LowResolution;  // magnetometer operation mode
float aRes, gRes, mRes;            // scale resolutions per LSB for the sensors

// Pin definitions
int myLed     = 13;  // LED on the Teensy 3.1

// LPS25H variables
float Temperature, Pressure; // stores LPS25H pressures sensor pressure and temperature

// LSM9DS0 variables
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float Quat[4] = {0, 0, 0, 0}; // quaternion data register
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
int16_t tempCount, rawPressure, rawTemperature;   // pressure, temperature raw count output
float   temperature;          // Stores the LSM9DS0 internal chip temperature in degrees Celsius
float SelfTest[6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll, Yaw, Pitch, Roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
uint8_t param[4];                         // used for param transfer
uint16_t EM7180_mag_fs, EM7180_acc_fs, EM7180_gyro_fs; // EM7180 sensor full scale ranges

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

bool passThru = false;

void setup()
{
  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(5000);
  Serial.begin(38400);
  
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  I2Cscan(); // should detect SENtral at 0x28
  
  // Read SENtral device information
  uint16_t ROM1 = readByte(EM7180_ADDRESS, EM7180_ROMVersion1);
  uint16_t ROM2 = readByte(EM7180_ADDRESS, EM7180_ROMVersion2);
  Serial.print("EM7180 ROM Version: 0x"); Serial.print(ROM1, HEX); Serial.println(ROM2, HEX); Serial.println("Should be: 0xE609");
  uint16_t RAM1 = readByte(EM7180_ADDRESS, EM7180_RAMVersion1);
  uint16_t RAM2 = readByte(EM7180_ADDRESS, EM7180_RAMVersion2);
  Serial.print("EM7180 RAM Version: 0x"); Serial.print(RAM1); Serial.println(RAM2);
  uint8_t PID = readByte(EM7180_ADDRESS, EM7180_ProductID);
  Serial.print("EM7180 ProductID: 0x"); Serial.print(PID, HEX); Serial.println(" Should be: 0x80");
  uint8_t RID = readByte(EM7180_ADDRESS, EM7180_RevisionID);
  Serial.print("EM7180 RevisionID: 0x"); Serial.print(RID, HEX); Serial.println(" Should be: 0x02");

  delay(2000); // give some time to read the screen

  // Check which sensors can be detected by the EM7180
  uint8_t featureflag = readByte(EM7180_ADDRESS, EM7180_FeatureFlags);
    if(featureflag & 0x01)  Serial.println("A barometer is installed");
    if(featureflag & 0x02)  Serial.println("A humidity sensor is installed");
    if(featureflag & 0x04)  Serial.println("A temperature sensor is installed");
    if(featureflag & 0x08)  Serial.println("A custom sensor is installed");
    if(featureflag & 0x10)  Serial.println("A second custom sensor is installed");
    if(featureflag & 0x20)  Serial.println("A third custom sensor is installed");
  
  delay(2000); // give some time to read the screen

  // Check SENtral status, make sure EEPROM upload of firmware was accomplished
  byte STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
  int count = 0;
  while(!STAT) {
    writeByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
    delay(500);  
    count++;  
    STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
    if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
    if(count > 10) break;
  }
  
   if(!(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04))  Serial.println("EEPROM upload successful!");
   delay(1000); // give some time to read the screen
    
  // Set up the SENtral as sensor bus in normal operating mode
if(!passThru) {
// Enter EM7180 initialized state
writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); // make sure pass through mode is off
// Set accel/gyro/mage desired ODR rates
writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, 0x02); // 95 Hz
writeByte(EM7180_ADDRESS, EM7180_MagRate, 0x1E); // 30 Hz
writeByte(EM7180_ADDRESS, EM7180_AccelRate, 0x14); // 200/10 Hz
writeByte(EM7180_ADDRESS, EM7180_GyroRate, 0x13);  // 190/10 Hz
writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | 0x19);  // set enable bit and set Baro rate to 25 Hz
//writeByte(EM7180_ADDRESS, EM7180_TempRate, 0x19);  // set enable bit and set rate to 25 Hz
// Configure operating mode
writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data
// Enable interrupt to host upon certain events
// choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
// new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x7F);
// Enable EM7180 run mode
writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
delay(100);

// EM7180 parameter adjustments
  Serial.println("Beginning Parameter Adjustments");
  
  // Read sensor default FS values from parameter space
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read parameter 74
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
  byte param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(param_xfer==0x4A)) {
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
  EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
  Serial.print("Magnetometer Default Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
  Serial.print("Accelerometer Default Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
  param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(param_xfer==0x4B)) {
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
  Serial.print("Gyroscope Default Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm
  
  //Disable stillness mode
  EM7180_set_integer_param (0x49, 0x00);
  
  //Write desired sensor full scale ranges to the EM7180
  EM7180_set_mag_acc_FS (0x3E8, 0x08); // 1000 uT, 8 g
  EM7180_set_gyro_FS (0x7D0); // 2000 dps
  
  // Read sensor new FS values from parameter space
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read  parameter 74
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
  param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(param_xfer==0x4A)) {
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
  EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);
  Serial.print("Magnetometer New Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
  Serial.print("Accelerometer New Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
  param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(param_xfer==0x4B)) {
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);
  Serial.print("Gyroscope New Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm
  

// Read EM7180 status
uint8_t runStatus = readByte(EM7180_ADDRESS, EM7180_RunStatus);
if(runStatus & 0x01) Serial.println(" EM7180 run status = normal mode");
uint8_t algoStatus = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
if(algoStatus & 0x01) Serial.println(" EM7180 standby status");
if(algoStatus & 0x02) Serial.println(" EM7180 algorithm slow");
if(algoStatus & 0x04) Serial.println(" EM7180 in stillness mode");
if(algoStatus & 0x08) Serial.println(" EM7180 mag calibration completed");
if(algoStatus & 0x10) Serial.println(" EM7180 magnetic anomaly detected");
if(algoStatus & 0x20) Serial.println(" EM7180 unreliable sensor data");
uint8_t passthruStatus = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
if(passthruStatus & 0x01) Serial.print(" EM7180 in passthru mode!");
uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus);
if(eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
if(eventStatus & 0x02) Serial.println(" EM7180 Error");
if(eventStatus & 0x04) Serial.println(" EM7180 new quaternion result");
if(eventStatus & 0x08) Serial.println(" EM7180 new mag result");
if(eventStatus & 0x10) Serial.println(" EM7180 new accel result");
if(eventStatus & 0x20) Serial.println(" EM7180 new gyro result"); 
if(eventStatus & 0x40) Serial.println(" EM7180 new baro result"); 
  
  delay(1000); // give some time to read the screen
  
  // Check sensor status
  uint8_t sensorStatus = readByte(EM7180_ADDRESS, EM7180_SensorStatus);
  Serial.print(" EM7180 sensor status = "); Serial.println(sensorStatus);
  if(sensorStatus == 0x00) Serial.println("All sensors OK!");
  if(sensorStatus & 0x01) Serial.println("Magnetometer not acknowledging!");
  if(sensorStatus & 0x02) Serial.println("Accelerometer not acknowledging!");
  if(sensorStatus & 0x04) Serial.println("Gyro not acknowledging!");
  if(sensorStatus & 0x10) Serial.println("Magnetometer ID not recognized!");
  if(sensorStatus & 0x20) Serial.println("Accelerometer ID not recognized!");
  if(sensorStatus & 0x40) Serial.println("Gyro ID not recognized!");
  
  Serial.print("Actual MagRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualMagRate)); Serial.println(" Hz"); 
  Serial.print("Actual AccelRate = "); Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualAccelRate)); Serial.println(" Hz"); 
  Serial.print("Actual GyroRate = "); Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualGyroRate)); Serial.println(" Hz"); 
  Serial.print("Actual BaroRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualBaroRate)); Serial.println(" Hz"); 
  Serial.print("Actual TempRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualTempRate)); Serial.println(" Hz"); 

  delay(3000); // give some time to read the screen
   
  }
 
  // If pass through mode desired, set it up here
  if(passThru) {
 // Put EM7180 SENtral into pass-through mode
  SENtralPassThroughMode();
  delay(1000);
  
  I2Cscan(); // should see all the devices on the I2C bus including two from the EEPROM (ID page and data pages)
 
// Read first page of EEPROM
   uint8_t data[128];
   M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x00, 0x00, 128, data);
   Serial.println("EEPROM Signature Byte"); 
   Serial.print(data[0], HEX); Serial.println("  Should be 0x2A");
   Serial.print(data[1], HEX); Serial.println("  Should be 0x65");
   for (int i = 0; i < 128; i++) {
     Serial.print(data[i], HEX); Serial.print(" ");
   }

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  
  display.begin(); // Initialize the display
  display.setContrast(58); // Set the contrast
  
// Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0); display.print("LSM9DS0");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("9-DOF 16-bit");
  display.setCursor(0, 30); display.print("motion sensor");
  display.setCursor(20,40); display.print("60 ug LSB");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer

  // Read the WHO_AM_I registers, this is a good test of communication
  Serial.println("LSM9DS0 9-axis motion sensor...");
  byte c = readByte(LSM9DS0G_ADDRESS, LSM9DS0G_WHO_AM_I_G);  // Read WHO_AM_I register for LSM9DS0 gyro
  Serial.println("LSM9DS0 gyro"); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0xD4, HEX);
  byte d = readByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_WHO_AM_I_XM);  // Read WHO_AM_I register for LSM9DS0 accel/magnetometer
  Serial.println("LSM9DS0 accel/magnetometer"); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x49, HEX);


   if (c == 0xD4 && d == 0x49) // WHO_AM_I should always be 0xD4 for the gyro and 0x49 for the accel/mag
  {  
    Serial.println("LSM9DS0 is online...");
 
   initLSM9DS0(); 
   Serial.println("LSM9DS0 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

   // get sensor resolutions, only need to do this once
   getAres();
   getGres();
   getMres();
   Serial.print("accel sensitivity is "); Serial.print(1./(1000.*aRes)); Serial.println(" LSB/mg");
   Serial.print("gyro sensitivity is "); Serial.print(1./(1000.*gRes)); Serial.println(" LSB/mdps");
   Serial.print("mag sensitivity is "); Serial.print(1./(1000.*mRes)); Serial.println(" LSB/mGauss");

  accelgyrocalLSM9DS0(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
  Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
  Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
 
  magcalLSM9DS0(magBias);
  Serial.println("mag biases (mG)"); Serial.println(1000.*magBias[0]); Serial.println(1000.*magBias[1]); Serial.println(1000.*magBias[2]);
  
  /* display.clearDisplay();
     
  display.setCursor(0, 0); display.print("LSM9DS0bias");
  display.setCursor(0, 8); display.print(" x   y   z  ");

  display.setCursor(0,  16); display.print((int)(1000*accelBias[0])); 
  display.setCursor(24, 16); display.print((int)(1000*accelBias[1])); 
  display.setCursor(48, 16); display.print((int)(1000*accelBias[2])); 
  display.setCursor(72, 16); display.print("mg");
    
  display.setCursor(0,  24); display.print(gyroBias[0], 1); 
  display.setCursor(24, 24); display.print(gyroBias[1], 1); 
  display.setCursor(48, 24); display.print(gyroBias[2], 1); 
  display.setCursor(66, 24); display.print("o/s");   
 
  display.display();
  delay(1000); 
 */ 
 }
  else
  {
    Serial.print("Could not connect to LSM9DS0: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

 
  
  // Read the WHO_AM_I register of the altimeter this is a good test of communication
  byte e = readByte(LPS25H_ADDRESS, LPS25H_WHOAMI);  // Read WHO_AM_I register for LPS25H
  Serial.print("LPS25H "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0xBD, HEX);
  display.clearDisplay();
  display.setCursor(20,0); display.print("LPS25H");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(e, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0xBD, HEX);  
  display.display();
  delay(1000); 
    
    if (e == 0xBD) // WHO_AM_I should always be 0xBD
  {  
    
      LPS25HInit();  // Initialize lPS25H altimeter

    display.clearDisplay();   // clears the screen and buffer
    display.setCursor(0, 0); display.print("LPS25H");
    display.setCursor(0,10); display.print("ready ");   
    display.display();
    delay(1000);
  }
  else
  {
    Serial.print("Could not connect to LPS25H: 0x");
    Serial.println(e, HEX);
    display.clearDisplay();   // clears the screen and buffer
    display.setCursor(0, 0); display.print("LPS25H");
    display.setCursor(0,10); display.print("Error! on 0x"); display.print(e, HEX);
    display.display();
    
    while(1) ; // Loop forever if communication doesn't happen
  }
  
  }
}


void loop()
{  
  if(!passThru) {
    
  // Check event status register, way to check data ready by polling rather than interrupt
  uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register
  
   // Check for errors
  if(eventStatus & 0x02) { // error detected, what is it?
  
  uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
  if(!errorStatus) {
  Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);
  if(errorStatus == 0x11) Serial.print("Magnetometer failure!");
  if(errorStatus == 0x12) Serial.print("Accelerometer failure!");
  if(errorStatus == 0x14) Serial.print("Gyro failure!");
  if(errorStatus == 0x21) Serial.print("Magnetometer initialization failure!");
  if(errorStatus == 0x22) Serial.print("Accelerometer initialization failure!");
  if(errorStatus == 0x24) Serial.print("Gyro initialization failure!");
  if(errorStatus == 0x30) Serial.print("Math error!");
  if(errorStatus == 0x80) Serial.print("Invalid sample rate!");
  }
  
  // Handle errors ToDo
  
  }
 
 // if no errors, see if new data is ready
  if(eventStatus & 0x10) { // new acceleration data available
     readSENtralAccelData(accelCount);
  
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*0.000488;  // get actual g value
    ay = (float)accelCount[1]*0.000488;    
    az = (float)accelCount[2]*0.000488;  
  }
  
   if(eventStatus & 0x20) { // new gyro data available
    readSENtralGyroData(gyroCount);
  
    // Now we'll calculate the gyro value into actual dps's
    gx = (float)gyroCount[0]*0.153;  // get actual dps value
    gy = (float)gyroCount[1]*0.153;    
    gz = (float)gyroCount[2]*0.153;  
   }

  if(eventStatus & 0x08) { // new mag data available
    readSENtralMagData(magCount);
  
    // Now we'll calculate the mag value into actual G's
    mx = (float)magCount[0]*0.305176;  // get actual G value
    my = (float)magCount[1]*0.305176;    
    mz = (float)magCount[2]*0.305176;  
   }
   
    if(eventStatus & 0x04) { // new quaternion data available
    readSENtralQuatData(Quat); 
    }
    
    // get LPS25H pressure
   if(eventStatus & 0x40) { // new baro data available
 //   Serial.println("new Baro data!");
    rawPressure = readSENtralBaroData();
    Pressure = (float) rawPressure*3000.; // pressure in mBar

    // get LPS25H temperature
    rawTemperature = readSENtralTempData();  
    Temperature = (float) rawTemperature*0.01;  // temperature in degrees C
  // }

  
  }
 
 
 
  if(passThru) {
  if (readByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_STATUS_REG_A) & 0x08) {  // check if new accel data is ready  
    readAccelData(accelCount);  // Read the x/y/z adc values
 
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2]; 
  } 
   
  if (readByte(LSM9DS0G_ADDRESS, LSM9DS0G_STATUS_REG_G) & 0x08) {  // check if new gyro data is ready  
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   
  }
  
  if (readByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_STATUS_REG_M) & 0x08) {  // check if new mag data is ready  
    readMagData(magCount);  // Read the x/y/z adc values
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes - magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes - magBias[1];  
    mz = (float)magCount[2]*mRes - magBias[2];   
  }
  } 
 
 
 // keep track of rates
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Sensors x (y)-axis of the accelerometer is aligned with the -y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ up) is aligned with z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the BMX-055, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the MPU9250 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  mx,  my, mz);
//  if(passThru)MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 1000) { // update LCD once per half-second independent of read rate

    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
   if(!passThru) {
    Serial.print("mx = "); Serial.print( mx); 
    Serial.print(" my = "); Serial.print( my); 
    Serial.print(" mz = "); Serial.print( mz); Serial.println(" mG");
   }
   else {
    Serial.print("mx = "); Serial.print( (int)1000*mx); 
    Serial.print(" my = "); Serial.print( (int)1000*my); 
    Serial.print(" mz = "); Serial.print( (int)1000*mz); Serial.println(" mG");
   }
     
    
    Serial.println("Software quaternions:"); 
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
    Serial.println("Hardware quaternions:"); 
    Serial.print("Q0 = "); Serial.print(Quat[3]);
    Serial.print(" Qx = "); Serial.print(Quat[0]); 
    Serial.print(" Qy = "); Serial.print(Quat[1]); 
    Serial.print(" Qz = "); Serial.println(Quat[2]); 

    float altitude = 145366.45f*(1.0f - pow((Pressure/1013.25f), 0.190284f)); 
    
    Serial.print("Altimeter temperature = "); Serial.print( Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Altimeter temperature = "); Serial.print(9.*Temperature/5. + 32., 2); Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter pressure = "); Serial.print(Pressure, 2);  Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
  
    }               
 

//   tempCount = readTempData();  // Read the gyro adc values
//   temperature = ((float) tempCount/8. + 25.0); // Gyro chip temperature in degrees Centigrade
//   Print temperature in degrees Centigrade      
//    Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
   if(passThru) {
    // Get altimeter data
    if(readByte(LPS25H_ADDRESS, LPS25H_STATUS_REG) & 0x20) {  // check if new altimeter data ready
    Pressure = (float) readAltimeterPressure()/4096.0f;
    Temperature = (float) readAltimeterTemperature()/480.0f + 42.5f;
    }
    float altitude = 145366.45f*(1.0f - pow((Pressure/1013.25f), 0.190284f)); 
    
    Serial.print("Altimeter temperature = "); Serial.print( Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Altimeter temperature = "); Serial.print(9.*Temperature/5. + 32., 2); Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter pressure = "); Serial.print(Pressure, 2);  Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
   }
   
    
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    //Software AHRS:
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;
    //Hardware AHRS:
    Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);   
    Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
    Roll  = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
    Pitch *= 180.0f / PI;
    Yaw   *= 180.0f / PI; 
    Yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    Roll  *= 180.0f / PI;
     
    // Or define output variable according to the Android system, where heading (0 to 260) is defined by the angle between the y-axis 
    // and True North, pitch is rotation about the x-axis (-180 to +180), and roll is rotation about the y-axis (-90 to +90)
    // In this systen, the z-axis is pointing away from Earth, the +y-axis is at the "top" of the device (cellphone) and the +x-axis
    // points toward the right of the device.
    //
    
    if(SerialDebug) {
    Serial.print("Software yaw, pitch, roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    Serial.print("Hardware Yaw, Pitch, Roll: ");
    Serial.print(Yaw, 2);
    Serial.print(", ");
    Serial.print(Pitch, 2);
    Serial.print(", ");
    Serial.println(Roll, 2);
    
    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    }

    Serial.print(millis()/1000); Serial.print(",");
    Serial.print(yaw); Serial.print(","); Serial.print(pitch); Serial.print(","); Serial.print(roll); Serial.print(",");
    Serial.print(Yaw); Serial.print(","); Serial.print(Pitch); Serial.print(","); Serial.print(Roll); Serial.println(",");
    
 /*  
    display.clearDisplay();    
 
    display.setCursor(0, 0); display.print(" x   y   z ");

    display.setCursor(0,  8); display.print((int)(1000*ax)); 
    display.setCursor(24, 8); display.print((int)(1000*ay)); 
    display.setCursor(48, 8); display.print((int)(1000*az)); 
    display.setCursor(72, 8); display.print("mg");
    
 //   tempCount = readACCTempData();  // Read the gyro adc values
 //   temperature = ((float) tempCount) / 2.0 + 23.0; // Gyro chip temperature in degrees Centigrade
 //   display.setCursor(64, 0); display.print(9.*temperature/5. + 32., 0); display.print("F");
    
    display.setCursor(0,  16); display.print((int)(gx)); 
    display.setCursor(24, 16); display.print((int)(gy)); 
    display.setCursor(48, 16); display.print((int)(gz)); 
    display.setCursor(66, 16); display.print("o/s");    

    display.setCursor(0,  24); display.print((int)(mx)); 
    display.setCursor(24, 24); display.print((int)(my)); 
    display.setCursor(48, 24); display.print((int)(mz)); 
    display.setCursor(72, 24); display.print("mG");    
 
    display.setCursor(0,  32); display.print((int)(yaw)); 
    display.setCursor(24, 32); display.print((int)(pitch)); 
    display.setCursor(48, 32); display.print((int)(roll)); 
    display.setCursor(66, 32); display.print("ypr");  
  
 
//    display.setCursor(0, 40); display.print(altitude, 0); display.print("ft"); 
//    display.setCursor(68, 0); display.print(9.*Temperature/5. + 32., 0); 
    display.setCursor(42, 40); display.print((float) sumCount / (1000.*sum), 2); display.print("kHz"); 
    display.display();
*/
    digitalWrite(myLed, !digitalRead(myLed));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
    }

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================
void getMres() {
  switch (Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 2 Gauss (00), 4 Gauss (01), 8 Gauss (10) and 12 Gauss (11)
    case MFS_2G:
          mRes = 2.0/32768.0;
          break;
    case MFS_4G:
          mRes = 4.0/32768.0;
          break;
    case MFS_8G:
          mRes = 8.0/32768.0;
          break;
    case MFS_12G:
          mRes = 12.0/32768.0;
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), and 2000 DPS  (11). 
    case GFS_245DPS:
          gRes = 245.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (000), 4 Gs (001), 6 gs (010), 8 Gs (011), and 16 gs (100). 
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_6G:
          aRes = 6.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(LSM9DS0XM_ADDRESS, 0x80 | LSM9DS0XM_OUT_X_L_A, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(LSM9DS0G_ADDRESS, 0x80 | LSM9DS0G_OUT_X_L_G, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

void readMagData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(LSM9DS0XM_ADDRESS, 0x80 | LSM9DS0XM_OUT_X_L_M, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(LSM9DS0XM_ADDRESS, 0x80 | LSM9DS0XM_OUT_TEMP_L_XM, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a 16-bit signed value
}
       

void initLSM9DS0()
{  
   // configure the gyroscope, enable normal mode = power on
   writeByte(LSM9DS0G_ADDRESS, LSM9DS0G_CTRL_REG1_G, Godr << 6 | Gbw << 4 | 0x0F);
   writeByte(LSM9DS0G_ADDRESS, LSM9DS0G_CTRL_REG4_G, Gscale << 4 | 0x80); // enable bloack data update
   // configure the accelerometer-specify ODR (sample rate) selection with Aodr, enable block data update
   writeByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_CTRL_REG1_XM, Aodr << 4 | 0x0F);
   // configure the accelerometer-specify bandwidth and full-scale selection with Abw, Ascale 
   writeByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_CTRL_REG2_XM, Abw << 6 | Ascale << 3);
    // enable temperature sensor, set magnetometer ODR (sample rate) and resolution mode
   writeByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_CTRL_REG5_XM, 0x80 | Mres << 5 | Modr << 2);
   // set magnetometer full scale
   writeByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_CTRL_REG6_XM, Mscale << 5 & 0x60);
   writeByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_CTRL_REG7_XM, 0x00); // select continuous conversion mode
 }


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalLSM9DS0(float * dest1, float * dest2)
{  
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t samples, ii;
  
  Serial.println("Calibrating gyro...");
 
  // First get gyro bias
  byte c = readByte(LSM9DS0G_ADDRESS, LSM9DS0G_CTRL_REG5_G);
  writeByte(LSM9DS0G_ADDRESS, LSM9DS0G_CTRL_REG5_G, c | 0x40);     // Enable gyro FIFO  
  delay(400);                                                       // Wait for change to take effect
  writeByte(LSM9DS0G_ADDRESS, LSM9DS0G_FIFO_CTRL_REG_G, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(2000);  // delay 1000 milliseconds to collect FIFO samples
  
  samples = (readByte(LSM9DS0G_ADDRESS, LSM9DS0G_FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    int16_t gyro_temp[3] = {0, 0, 0};
    readBytes(LSM9DS0G_ADDRESS, 0x80 | LSM9DS0G_OUT_X_L_G, 6, &data[0]);
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    gyro_bias[1] += (int32_t) gyro_temp[1]; 
    gyro_bias[2] += (int32_t) gyro_temp[2]; 
  }  

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 
  
  dest1[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
  dest1[1] = (float)gyro_bias[1]*gRes;
  dest1[2] = (float)gyro_bias[2]*gRes;
  
  c = readByte(LSM9DS0G_ADDRESS, LSM9DS0G_CTRL_REG5_G);
  writeByte(LSM9DS0G_ADDRESS, LSM9DS0G_CTRL_REG5_G, c & ~0x40);   //Disable gyro FIFO  
  delay(200);
  writeByte(LSM9DS0G_ADDRESS, LSM9DS0G_FIFO_CTRL_REG_G, 0x00);  // Enable gyro bypass mode
 
   Serial.println("Calibrating accel...");
 
  // now get the accelerometer bias
  c = readByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_CTRL_REG0_XM);
  writeByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_CTRL_REG0_XM, c | 0x40);     // Enable gyro FIFO  
  delay(200);                                                       // Wait for change to take effect
  writeByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_FIFO_CTRL_REG, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples
  
  samples = (readByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_FIFO_SRC_REG) & 0x1F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(LSM9DS0XM_ADDRESS, 0x80 | LSM9DS0XM_OUT_X_L_A, 6, &data[0]);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1]; 
    accel_bias[2] += (int32_t) accel_temp[2]; 
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) (1.0/aRes);}
  
  dest2[0] = (float)accel_bias[0]*aRes;  // Properly scale the data to get g
  dest2[1] = (float)accel_bias[1]*aRes;
  dest2[2] = (float)accel_bias[2]*aRes;
  
  c = readByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_CTRL_REG0_XM);
  writeByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_CTRL_REG0_XM, c & ~0x40);   //Disable accel FIFO  
  delay(200);
  writeByte(LSM9DS0XM_ADDRESS, LSM9DS0XM_FIFO_CTRL_REG, 0x00);  // Enable accel bypass mode
}

void magcalLSM9DS0(float * dest1) 
{
  uint8_t data[6]; // data array to hold mag x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};
 
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
   sample_count = 128;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readBytes(LSM9DS0XM_ADDRESS, 0x80 | LSM9DS0XM_OUT_X_L_M, 6, &data[0]);  // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
   }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes;   
    dest1[2] = (float) mag_bias[2]*mRes;          

 /* //write biases to accelerometermagnetometer offset registers as counts);
  writeByte(LSM9DS0M_ADDRESS, LSM9DS0M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
  writeByte(LSM9DS0M_ADDRESS, LSM9DS0M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeByte(LSM9DS0M_ADDRESS, LSM9DS0M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
  writeByte(LSM9DS0M_ADDRESS, LSM9DS0M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeByte(LSM9DS0M_ADDRESS, LSM9DS0M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
  writeByte(LSM9DS0M_ADDRESS, LSM9DS0M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);
 */
   Serial.println("Mag Calibration done!");
}

int32_t readAltimeterPressure()
{
    uint8_t rawData[3];  // 24-bit pressure register data stored here
    readBytes(LPS25H_ADDRESS, (LPS25H_PRESS_OUT_XL | 0x80), 3, &rawData[0]); // bit 7 must be one to read multiple bytes
    return (int32_t) ((int32_t) rawData[2] << 16 | (int32_t) rawData[1] << 8 | rawData[0]);
}

int16_t readAltimeterTemperature()
{
    uint8_t rawData[2];  // 16-bit pressure register data stored here
    readBytes(LPS25H_ADDRESS, (LPS25H_TEMP_OUT_L | 0x80), 2, &rawData[0]); // bit 7 must be one to read multiple bytes
    return (int16_t)((int16_t) rawData[1] << 8 | rawData[0]);
}


void LPS25HInit()
{
  // turn on the altimeter by setting bit 7 to one
  // set sample rate by setting bits 6:4 
  // enable interrupt circuit by setting bit 3 to one
  // make sure data not updated during read by setting block data more (bit 2) to 1
  //  writeByte(LPS25H_ADDRESS, LPS25H_CTRL_REG1, 0x80 | PODR << 4 | 0x08 | 0x04);  
  //  writeByte(LPS25H_ADDRESS, LPS25H_CTRL_REG4, 0x01); // set interrupt pin to signal data ready
  //  writeByte(LPS25H_ADDRESS, LPS25H_RES_CONF, Tavg << 2 | Pavg); // specify temperature and pressure internal averaging
  // or use ultra low-power mode
  // To reduce power consumption while keeping a low noise figure, reduce the temperature and pressure averaging
  // and reduce ODR to the minimum and enable the digital filter (FIFO)
  writeByte(LPS25H_ADDRESS, LPS25H_RES_CONF,  0x05);  // set Tavg = 16 and Pavg = 32 internal averaging
  writeByte(LPS25H_ADDRESS, LPS25H_FIFO_CTRL, 0xDF);  // set FIFO mean mode with average on two samples or more
  writeByte(LPS25H_ADDRESS, LPS25H_CTRL_REG2, 0x21);  // FIFO enabled, decimation disabled
  writeByte(LPS25H_ADDRESS, LPS25H_CTRL_REG1, 0x90);  // power on device, set to 1 Hz sample rate
}


// I2C read/write functions for the LSM9DS0and AK8963 sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);     // Put slave register address in Tx buffer 
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}



static inline float uint32_reg_to_float (uint8_t *buf)
{
  union {
    uint32_t ui32;
    float f;
  } u;

  u.ui32 =     (((uint32_t)buf[0]) +
               (((uint32_t)buf[1]) <<  8) +
               (((uint32_t)buf[2]) << 16) +
               (((uint32_t)buf[3]) << 24));
  return u.f;
}


void float_to_bytes (float param_val, uint8_t *buf) {
  union {
    float f;
    uint8_t comp[sizeof(float)];
  } u;
  u.f = param_val;
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = u.comp[i];
  }
  //Convert to LITTLE ENDIAN
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = buf[(sizeof(float)-1) - i];
  }
}

void EM7180_set_gyro_FS (uint16_t gyro_fs) {
  uint8_t bytes[4], STAT;
  bytes[0] = gyro_fs & (0xFF);
  bytes[1] = (gyro_fs >> 8) & (0xFF);
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Unused
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Unused
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==0xCB)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs) {
  uint8_t bytes[4], STAT;
  bytes[0] = mag_fs & (0xFF);
  bytes[1] = (mag_fs >> 8) & (0xFF);
  bytes[2] = acc_fs & (0xFF);
  bytes[3] = (acc_fs >> 8) & (0xFF);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Mag LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Mag MSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Acc LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Acc MSB
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA); //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==0xCA)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_integer_param (uint8_t param, uint32_t param_val) {
  uint8_t bytes[4], STAT;
  bytes[0] = param_val & (0xFF);
  bytes[1] = (param_val >> 8) & (0xFF);
  bytes[2] = (param_val >> 16) & (0xFF);
  bytes[3] = (param_val >> 24) & (0xFF);
  param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==param)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_float_param (uint8_t param, float param_val) {
  uint8_t bytes[4], STAT;
  float_to_bytes (param_val, &bytes[0]);
  param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==param)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}
void readSENtralQuatData(float * destination)
{
  uint8_t rawData[16];  // x/y/z quaternion register data stored here
  readBytes(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]);       // Read the sixteen raw data registers into data array
  destination[0] = uint32_reg_to_float (&rawData[0]);
  destination[1] = uint32_reg_to_float (&rawData[4]);
  destination[2] = uint32_reg_to_float (&rawData[8]);
  destination[3] = uint32_reg_to_float (&rawData[12]);

}

void readSENtralAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(EM7180_ADDRESS, EM7180_AX, 6, &rawData[0]);       // Read the six raw data registers into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void readSENtralGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_GX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void readSENtralMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_MX, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

int16_t readSENtralBaroData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_Baro, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}

int16_t readSENtralTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(EM7180_ADDRESS, EM7180_Temp, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}


void SENtralPassThroughMode()
{
  // First put SENtral in standby mode
  uint8_t c = readByte(EM7180_ADDRESS, EM7180_AlgorithmControl);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, c | 0x01);
//  c = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
//  Serial.print("c = "); Serial.println(c);
// Verify standby status
// if(readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus) & 0x01) {
   Serial.println("SENtral in standby mode"); 
  // Place SENtral in pass-through mode
  writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x01); 
  if(readByte(EM7180_ADDRESS, EM7180_PassThruStatus) & 0x01) {
    Serial.println("SENtral in pass-through mode");
  }
  else {
    Serial.println("ERROR! SENtral not in pass-through mode!");
  }
  
// }
//   else { Serial.println("ERROR! SENtral not in standby mode!"); 
// }
 
 }

// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

        void M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data)
{
	Wire.beginTransmission(device_address);   // Initialize the Tx buffer
	Wire.write(data_address1);                // Put slave register address in Tx buffer
	Wire.write(data_address2);                // Put slave register address in Tx buffer
	Wire.write(data);                         // Put data in Tx buffer
	Wire.endTransmission();                   // Send the Tx buffer
}


       void M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{
	if(count > 128) {
        count = 128;
        Serial.print("Page count cannot be more than 128 bytes!");
        }
        
        Wire.beginTransmission(device_address);   // Initialize the Tx buffer
	Wire.write(data_address1);                // Put slave register address in Tx buffer
	Wire.write(data_address2);                // Put slave register address in Tx buffer
	for(uint8_t i=0; i < count; i++) {
	Wire.write(dest[i]);                      // Put data in Tx buffer
	}
        Wire.endTransmission();                   // Send the Tx buffer
}


        uint8_t M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(device_address);         // Initialize the Tx buffer
	Wire.write(data_address1);                // Put slave register address in Tx buffer
	Wire.write(data_address2);                // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(device_address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(device_address);   // Initialize the Tx buffer
	Wire.write(data_address1);                     // Put slave register address in Tx buffer
	Wire.write(data_address2);                     // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);         // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);              // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);       // Read bytes from slave register address 
        Wire.requestFrom(device_address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }                // Put read results in the Rx buffer
}



// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
    // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
