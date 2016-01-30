/* EM7180_BMX055_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: December 24, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 The EM7180 SENtral sensor hub is not a motion sensor, but rather takes raw sensor data from a variety of motion sensors,
 in this case the BMX0655, and does sensor fusion with quaternions as its output. The SENtral loads firmware from the
 on-board M24512DFMC 512 kbit EEPROM upon startup, configures and manages the sensors on its dedicated master I2C bus,
 and outputs scaled sensor data (accelerations, rotation rates, and magnetic fields) as well as quaternions and
 heading/pitch/roll, if selected.
 
 This sketch demonstrates basic EM7180 SENtral functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms to compare with the hardware sensor fusion results.
 Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is specifically for the Teensy 3.1 Mini Add-On shield with the EM7180 SENtral sensor hub as master,
 the BMX-055 9-axis motion sensor (accel/gyro/mag) as slave, an MS5637 pressure/temperature sensor, and an M24512DFM
 512kbit (64 kByte) EEPROM as slave all connected via I2C. The SENtral cannot use the pressure data in the sensor fusion
 yet and there is currently no driver for the MS5637 in the SENtral firmware. However, like the MAX21100, the SENtral
 can be toggled into a bypass mode where the pressure sensor (and EEPROM and BMX055) may be read directly by the
 Teensy 3.1 host micrcontroller. If the read rate is infrequent enough (2 Hz is sufficient since pressure and temperature
 do not change very fast), then the sensor fusion rate is not significantly affected.
 
 This sketch uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The MS5637 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4k7 resistors are on the EM7180+BMX055+MS5637+M24512DFM Mini Add-On board for Teensy 3.1.
 
 Hardware setup:
 EM7180 Mini Add-On ------- Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- 17
 SCL ----------------------- 16
 GND ---------------------- GND
 INT------------------------ 8
 
 Note: The BMX055 is an I2C sensor and uses the Teensy 3.1 i2c_t3.h Wire library. 
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

// See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet http://www.meas-spec.com/downloads/MS5637-02BA03.pdf
#define MS5637_RESET      0x1E
#define MS5637_CONVERT_D1 0x40
#define MS5637_CONVERT_D2 0x50
#define MS5637_ADC_READ   0x00

// BMX055 data sheet http://ae-bst.resource.bosch.com/media/products/dokumente/bmx055/BST-BMX055-DS000-01v2.pdf
// The BMX055 is a conglomeration of three separate motion sensors packaged together but
// addressed and communicated with separately by design
// Accelerometer registers
#define BMX055_ACC_WHOAMI        0x00   // should return 0xFA
//#define BMX055_ACC_Reserved    0x01
#define BMX055_ACC_D_X_LSB       0x02
#define BMX055_ACC_D_X_MSB       0x03
#define BMX055_ACC_D_Y_LSB       0x04
#define BMX055_ACC_D_Y_MSB       0x05
#define BMX055_ACC_D_Z_LSB       0x06
#define BMX055_ACC_D_Z_MSB       0x07
#define BMX055_ACC_D_TEMP        0x08
#define BMX055_ACC_INT_STATUS_0  0x09
#define BMX055_ACC_INT_STATUS_1  0x0A
#define BMX055_ACC_INT_STATUS_2  0x0B
#define BMX055_ACC_INT_STATUS_3  0x0C
//#define BMX055_ACC_Reserved    0x0D
#define BMX055_ACC_FIFO_STATUS   0x0E
#define BMX055_ACC_PMU_RANGE     0x0F
#define BMX055_ACC_PMU_BW        0x10
#define BMX055_ACC_PMU_LPW       0x11
#define BMX055_ACC_PMU_LOW_POWER 0x12
#define BMX055_ACC_D_HBW         0x13
#define BMX055_ACC_BGW_SOFTRESET 0x14
//#define BMX055_ACC_Reserved    0x15
#define BMX055_ACC_INT_EN_0      0x16
#define BMX055_ACC_INT_EN_1      0x17
#define BMX055_ACC_INT_EN_2      0x18
#define BMX055_ACC_INT_MAP_0     0x19
#define BMX055_ACC_INT_MAP_1     0x1A
#define BMX055_ACC_INT_MAP_2     0x1B
//#define BMX055_ACC_Reserved    0x1C
//#define BMX055_ACC_Reserved    0x1D
#define BMX055_ACC_INT_SRC       0x1E
//#define BMX055_ACC_Reserved    0x1F
#define BMX055_ACC_INT_OUT_CTRL  0x20
#define BMX055_ACC_INT_RST_LATCH 0x21
#define BMX055_ACC_INT_0         0x22
#define BMX055_ACC_INT_1         0x23
#define BMX055_ACC_INT_2         0x24
#define BMX055_ACC_INT_3         0x25
#define BMX055_ACC_INT_4         0x26
#define BMX055_ACC_INT_5         0x27
#define BMX055_ACC_INT_6         0x28
#define BMX055_ACC_INT_7         0x29
#define BMX055_ACC_INT_8         0x2A
#define BMX055_ACC_INT_9         0x2B
#define BMX055_ACC_INT_A         0x2C
#define BMX055_ACC_INT_B         0x2D
#define BMX055_ACC_INT_C         0x2E
#define BMX055_ACC_INT_D         0x2F
#define BMX055_ACC_FIFO_CONFIG_0 0x30
//#define BMX055_ACC_Reserved    0x31
#define BMX055_ACC_PMU_SELF_TEST 0x32
#define BMX055_ACC_TRIM_NVM_CTRL 0x33
#define BMX055_ACC_BGW_SPI3_WDT  0x34
//#define BMX055_ACC_Reserved    0x35
#define BMX055_ACC_OFC_CTRL      0x36
#define BMX055_ACC_OFC_SETTING   0x37
#define BMX055_ACC_OFC_OFFSET_X  0x38
#define BMX055_ACC_OFC_OFFSET_Y  0x39
#define BMX055_ACC_OFC_OFFSET_Z  0x3A
#define BMX055_ACC_TRIM_GPO      0x3B
#define BMX055_ACC_TRIM_GP1      0x3C
//#define BMX055_ACC_Reserved    0x3D
#define BMX055_ACC_FIFO_CONFIG_1 0x3E
#define BMX055_ACC_FIFO_DATA     0x3F

// BMX055 Gyroscope Registers
#define BMX055_GYRO_WHOAMI           0x00  // should return 0x0F
//#define BMX055_GYRO_Reserved       0x01
#define BMX055_GYRO_RATE_X_LSB       0x02
#define BMX055_GYRO_RATE_X_MSB       0x03
#define BMX055_GYRO_RATE_Y_LSB       0x04
#define BMX055_GYRO_RATE_Y_MSB       0x05
#define BMX055_GYRO_RATE_Z_LSB       0x06
#define BMX055_GYRO_RATE_Z_MSB       0x07
//#define BMX055_GYRO_Reserved       0x08
#define BMX055_GYRO_INT_STATUS_0  0x09
#define BMX055_GYRO_INT_STATUS_1  0x0A
#define BMX055_GYRO_INT_STATUS_2  0x0B
#define BMX055_GYRO_INT_STATUS_3  0x0C
//#define BMX055_GYRO_Reserved    0x0D
#define BMX055_GYRO_FIFO_STATUS   0x0E
#define BMX055_GYRO_RANGE         0x0F
#define BMX055_GYRO_BW            0x10
#define BMX055_GYRO_LPM1          0x11
#define BMX055_GYRO_LPM2          0x12
#define BMX055_GYRO_RATE_HBW      0x13
#define BMX055_GYRO_BGW_SOFTRESET 0x14
#define BMX055_GYRO_INT_EN_0      0x15
#define BMX055_GYRO_INT_EN_1      0x16
#define BMX055_GYRO_INT_MAP_0     0x17
#define BMX055_GYRO_INT_MAP_1     0x18
#define BMX055_GYRO_INT_MAP_2     0x19
#define BMX055_GYRO_INT_SRC_1     0x1A
#define BMX055_GYRO_INT_SRC_2     0x1B
#define BMX055_GYRO_INT_SRC_3     0x1C
//#define BMX055_GYRO_Reserved    0x1D
#define BMX055_GYRO_FIFO_EN       0x1E
//#define BMX055_GYRO_Reserved    0x1F
//#define BMX055_GYRO_Reserved    0x20
#define BMX055_GYRO_INT_RST_LATCH 0x21
#define BMX055_GYRO_HIGH_TH_X     0x22
#define BMX055_GYRO_HIGH_DUR_X    0x23
#define BMX055_GYRO_HIGH_TH_Y     0x24
#define BMX055_GYRO_HIGH_DUR_Y    0x25
#define BMX055_GYRO_HIGH_TH_Z     0x26
#define BMX055_GYRO_HIGH_DUR_Z    0x27
//#define BMX055_GYRO_Reserved    0x28
//#define BMX055_GYRO_Reserved    0x29
//#define BMX055_GYRO_Reserved    0x2A
#define BMX055_GYRO_SOC           0x31
#define BMX055_GYRO_A_FOC         0x32
#define BMX055_GYRO_TRIM_NVM_CTRL 0x33
#define BMX055_GYRO_BGW_SPI3_WDT  0x34
//#define BMX055_GYRO_Reserved    0x35
#define BMX055_GYRO_OFC1          0x36
#define BMX055_GYRO_OFC2          0x37
#define BMX055_GYRO_OFC3          0x38
#define BMX055_GYRO_OFC4          0x39
#define BMX055_GYRO_TRIM_GP0      0x3A
#define BMX055_GYRO_TRIM_GP1      0x3B
#define BMX055_GYRO_BIST          0x3C
#define BMX055_GYRO_FIFO_CONFIG_0 0x3D
#define BMX055_GYRO_FIFO_CONFIG_1 0x3E

// BMX055 magnetometer registers
#define BMX055_MAG_WHOAMI         0x40  // should return 0x32
#define BMX055_MAG_Reserved       0x41
#define BMX055_MAG_XOUT_LSB       0x42
#define BMX055_MAG_XOUT_MSB       0x43
#define BMX055_MAG_YOUT_LSB       0x44
#define BMX055_MAG_YOUT_MSB       0x45
#define BMX055_MAG_ZOUT_LSB       0x46
#define BMX055_MAG_ZOUT_MSB       0x47
#define BMX055_MAG_ROUT_LSB       0x48
#define BMX055_MAG_ROUT_MSB       0x49
#define BMX055_MAG_INT_STATUS     0x4A
#define BMX055_MAG_PWR_CNTL1      0x4B
#define BMX055_MAG_PWR_CNTL2      0x4C
#define BMX055_MAG_INT_EN_1       0x4D
#define BMX055_MAG_INT_EN_2       0x4E
#define BMX055_MAG_LOW_THS        0x4F
#define BMX055_MAG_HIGH_THS       0x50
#define BMX055_MAG_REP_XY         0x51
#define BMX055_MAG_REP_Z          0x52
/* Trim Extended Registers */
#define BMM050_DIG_X1             0x5D // needed for magnetic field calculation
#define BMM050_DIG_Y1             0x5E  
#define BMM050_DIG_Z4_LSB         0x62
#define BMM050_DIG_Z4_MSB         0x63
#define BMM050_DIG_X2             0x64  
#define BMM050_DIG_Y2             0x65  
#define BMM050_DIG_Z2_LSB         0x68  
#define BMM050_DIG_Z2_MSB         0x69  
#define BMM050_DIG_Z1_LSB         0x6A  
#define BMM050_DIG_Z1_MSB         0x6B  
#define BMM050_DIG_XYZ1_LSB       0x6C 
#define BMM050_DIG_XYZ1_MSB       0x6D 
#define BMM050_DIG_Z3_LSB         0x6E
#define BMM050_DIG_Z3_MSB         0x6F
#define BMM050_DIG_XY2            0x70 
#define BMM050_DIG_XY1            0x71  

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
#define EM7180_ErrorRegister      0x50
#define EM7180_AlgorithmControl   0x54
#define EM7180_MagRate            0x55
#define EM7180_AccelRate          0x56
#define EM7180_GyroRate           0x57
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

// Using the Teensy Mini Add-On board, BMX055 SDO1 = SDO2 = CSB3 = GND as designed
// Seven-bit BMX055 device addresses are ACC = 0x18, GYRO = 0x68, MAG = 0x10
#define BMX055_ACC_ADDRESS  0x18   // Address of BMX055 accelerometer
#define BMX055_GYRO_ADDRESS 0x68   // Address of BMX055 gyroscope
#define BMX055_MAG_ADDRESS  0x10   // Address of BMX055 magnetometer
#define MS5637_ADDRESS      0x76   // Address of MS5637 altimeter
#define EM7180_ADDRESS      0x28   // Address of the EM7180 SENtral sensor hub
#define M24512DFM_DATA_ADDRESS   0x50   // Address of the 500 page M24512DFM EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
#define M24512DFM_IDPAGE_ADDRESS 0x58   // Address of the single M24512DFM lockable EEPROM ID page

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
// define X055 ACC full scale options
#define AFS_2G  0x03
#define AFS_4G  0x05
#define AFS_8G  0x08
#define AFS_16G 0x0C

enum ACCBW {    // define BMX055 accelerometer bandwidths
  ABW_8Hz,      // 7.81 Hz,  64 ms update time
  ABW_16Hz,     // 15.63 Hz, 32 ms update time
  ABW_31Hz,     // 31.25 Hz, 16 ms update time
  ABW_63Hz,     // 62.5  Hz,  8 ms update time
  ABW_125Hz,    // 125   Hz,  4 ms update time
  ABW_250Hz,    // 250   Hz,  2 ms update time
  ABW_500Hz,    // 500   Hz,  1 ms update time
  ABW_100Hz     // 1000  Hz,  0.5 ms update time
};

enum Gscale {
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS
};

enum GODRBW {
  G_2000Hz523Hz = 0, // 2000 Hz ODR and unfiltered (bandwidth 523Hz)
  G_2000Hz230Hz,
  G_1000Hz116Hz,
  G_400Hz47Hz,
  G_200Hz23Hz,
  G_100Hz12Hz,
  G_200Hz64Hz,
  G_100Hz32Hz  // 100 Hz ODR and 32 Hz bandwidth
};

enum MODR {
  MODR_10Hz = 0,   // 10 Hz ODR  
  MODR_2Hz     ,   // 2 Hz ODR
  MODR_6Hz     ,   // 6 Hz ODR
  MODR_8Hz     ,   // 8 Hz ODR
  MODR_15Hz    ,   // 15 Hz ODR
  MODR_20Hz    ,   // 20 Hz ODR
  MODR_25Hz    ,   // 25 Hz ODR
  MODR_30Hz        // 30 Hz ODR
};

enum Mmode {
  lowPower         = 0,   // rms noise ~1.0 microTesla, 0.17 mA power
  Regular             ,   // rms noise ~0.6 microTesla, 0.5 mA power
  enhancedRegular     ,   // rms noise ~0.5 microTesla, 0.8 mA power
  highAccuracy            // rms noise ~0.3 microTesla, 4.9 mA power
};

// MS5637 pressure sensor sample rates
#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_8192 0x0A
#define ADC_D1   0x40
#define ADC_D2   0x50

// Specify sensor full scale
uint8_t OSR    = ADC_8192;         // set pressure amd temperature oversample rate
uint8_t Gscale = GFS_125DPS;       // set gyro full scale  
uint8_t GODRBW = G_200Hz23Hz;      // set gyro ODR and bandwidth 
uint8_t Ascale = AFS_2G;           // set accel full scale  
uint8_t ACCBW  = 0x08 & ABW_16Hz;  // Choose bandwidth for accelerometer
uint8_t Mmode  = Regular;          // Choose magnetometer operation mode
uint8_t MODR   = MODR_10Hz;        // set magnetometer data rate 
float aRes, gRes, mRes;            // scale resolutions per LSB for the sensors

// Parameters to hold BMX055 trim values
signed char   dig_x1;
signed char   dig_y1;
signed char   dig_x2;
signed char   dig_y2;
uint16_t      dig_z1;
int16_t       dig_z2;
int16_t       dig_z3;
int16_t       dig_z4;
unsigned char dig_xy1;
signed char   dig_xy2;
uint16_t      dig_xyz1;

// Pin definitions
int myLed     = 13;  // LED on the Teensy 3.1

// MS5637 variables
uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw MS5637 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data
double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature

// BMX055 variables
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 13/15-bit signed magnetometer sensor output
float Quat[4] = {0, 0, 0, 0}; // quaternion data register
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the BMX055 internal chip temperature in degrees Celsius
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

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

bool passThru = false;

void setup()
{
//  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(5000);
  Serial.begin(38400);

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
  
  delay(1000); // give some time to read the screen

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
writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, 0x02); // 100 Hz
writeByte(EM7180_ADDRESS, EM7180_MagRate, 0x1E); // 30 Hz
writeByte(EM7180_ADDRESS, EM7180_AccelRate, 0x0A); // 100/10 Hz
writeByte(EM7180_ADDRESS, EM7180_GyroRate, 0x14); // 200/10 Hz
// Configure operating mode
writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data
// Enable interrupt to host upon certain events
// choose interrupts when quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);
// Enable EM7180 run mode
writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
delay(100);

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
  
  delay(1000); // give some time to read the screen
  
  // Check sensor status
  uint8_t sensorStatus = readByte(EM7180_ADDRESS, EM7180_SensorStatus);
  Serial.print(" EM7180 sensor status = "); Serial.println(sensorStatus);
  if(sensorStatus & 0x01) Serial.print("Magnetometer not acknowledging!");
  if(sensorStatus & 0x02) Serial.print("Accelerometer not acknowledging!");
  if(sensorStatus & 0x04) Serial.print("Gyro not acknowledging!");
  if(sensorStatus & 0x10) Serial.print("Magnetometer ID not recognized!");
  if(sensorStatus & 0x20) Serial.print("Accelerometer ID not recognized!");
  if(sensorStatus & 0x40) Serial.print("Gyro ID not recognized!");
  
  Serial.print("Actual MagRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualMagRate)); Serial.println(" Hz"); 
  Serial.print("Actual AccelRate = "); Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualAccelRate)); Serial.println(" Hz"); 
  Serial.print("Actual GyroRate = "); Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualGyroRate)); Serial.println(" Hz"); 

  delay(1000); // give some time to read the screen
   
  }
 
  // If pass through mode desired, set it up here
  if(passThru) {
 // Put EM7180 SENtral into pass-through mode
  SENtralPassThroughMode();

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
  display.setCursor(0,0); display.print("BMX055");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("9-DOF 16-bit");
  display.setCursor(0, 30); display.print("motion sensor");
  display.setCursor(20,40); display.print("1 mg LSB");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer

  // Read the BMX-055 WHO_AM_I registers, this is a good test of communication
  Serial.println("BMX055 accelerometer...");
  byte c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_WHOAMI);  // Read ACC WHO_AM_I register for BMX055
  Serial.print("BMX055 ACC"); Serial.print(" I AM 0x"); Serial.print(c, HEX); Serial.print(" I should be 0x"); Serial.println(0xFA, HEX);
  display.setCursor(20,0); display.print("BMX055 ACC");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print("0x"); display.print(c, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print("0x"); display.print(0xFA, HEX); 
  display.display();
  delay(1000); 

  display.clearDisplay();   // clears the screen and buffer
  Serial.println("BMX055 gyroscope...");
  byte d = readByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_WHOAMI);  // Read GYRO WHO_AM_I register for BMX055
  Serial.print("BMX055 GYRO"); Serial.print(" I AM 0x"); Serial.print(d, HEX); Serial.print(" I should be 0x"); Serial.println(0x0F, HEX);
  display.setCursor(0, 0); display.print("BMX055 GYRO");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print("0x"); display.print(d, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print("0x"); display.print(0x0F, HEX); 
  display.display();
  delay(1000); 
  
  Serial.println("BMX055 magnetometer...");
  writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // wake up magnetometer first thing
  delay(100);
  byte e = readByte(BMX055_MAG_ADDRESS, BMX055_MAG_WHOAMI);  // Read MAG WHO_AM_I register for BMX055
  Serial.print("BMX055 MAG"); Serial.print(" I AM 0x"); Serial.print(e, HEX); Serial.print(" I should be 0x"); Serial.println(0x32, HEX);
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0, 0); display.print("BMX055 MAG");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print("0x"); display.print(e, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print("0x"); display.print(0x32, HEX); 
  display.display();
  delay(1000); 

  if ((c == 0xFA) && (d == 0x0F) && (e == 0x32)) // WHO_AM_I should always be ACC = 0xFA, GYRO = 0x0F, MAG = 0x32
  {  
  Serial.println("BMX055 is online...");
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0, 0); display.print("BMX055 online");
  display.setCursor(0,10); display.print("configuring");
  display.display();
  delay(1000); 

  initBMX055(); 
  Serial.println("BMX055 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  display.clearDisplay();   // clears the screen and buffer
  display.setCursor(0, 0); display.print("BMX055 online");
  display.setCursor(0,10); display.print("initialized");
  display.display();
  delay(1000); 
  
  // Reset the MS5637 pressure sensor
  MS5637Reset();
  delay(100);
  Serial.println("MS5637 pressure sensor reset...");
  // Read PROM data from MS5637 pressure sensor
  MS5637PromRead(Pcal);
  Serial.println("PROM data read:");
  Serial.print("C0 = "); Serial.println(Pcal[0]);
  unsigned char refCRC = Pcal[0] >> 12;
  Serial.print("C1 = "); Serial.println(Pcal[1]);
  Serial.print("C2 = "); Serial.println(Pcal[2]);
  Serial.print("C3 = "); Serial.println(Pcal[3]);
  Serial.print("C4 = "); Serial.println(Pcal[4]);
  Serial.print("C5 = "); Serial.println(Pcal[5]);
  Serial.print("C6 = "); Serial.println(Pcal[6]);
  
  nCRC = MS5637checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
  Serial.print("Checksum = "); Serial.print(nCRC); Serial.print(" , should be "); Serial.println(refCRC);  
  
  display.clearDisplay();
  display.setCursor(20,0); display.print("MS5637");
  display.setCursor(0,10); display.print("CRC is "); display.setCursor(50,10); display.print(nCRC);
  display.setCursor(0,20); display.print("Should be "); display.setCursor(50,30); display.print(refCRC);
  display.display();
  delay(1000);  
    
  // get sensor resolutions, only need to do this once
   getAres();
   getGres();
   // magnetometer resolution is 1 microTesla/16 counts or 1/1.6 milliGauss/count
   mRes = 1./1.6;
   trimBMX055();  // read the magnetometer calibration data
   display.clearDisplay();
   display.setCursor(0, 0); display.print("BMX055 Res");
   display.setCursor(0,10); display.print("ACC ");  display.setCursor(50,10); display.print(1000.*aRes, 2);
   display.setCursor(0,20); display.print("GYRO "); display.setCursor(50,20); display.print(1000.*gRes, 2);
//   display.setCursor(0,30); display.print("MAG ");  display.setCursor(50,30); display.print((int)dig_x1);
   display.display();
   delay(1000); 
   
   
  fastcompaccelBMX055(accelBias);
  Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
  Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
 
  magcalBMX055(magBias);
  Serial.println("mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
   
  }
  else
  {
    Serial.print("Could not connect to BMX055: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}
}


void loop()
{  
  if(!passThru) {
    
  // Check event status register, way to chech data ready by polling rather than interrupt
  uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register
  
   // Check for errors
  if(eventStatus & 0x02) { // error detected, what is it?
  
  uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
  if(!errorStatus) {
  Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);
  if(errorStatus & 0x11) Serial.print("Magnetometer failure!");
  if(errorStatus & 0x12) Serial.print("Accelerometer failure!");
  if(errorStatus & 0x14) Serial.print("Gyro failure!");
  if(errorStatus & 0x21) Serial.print("Magnetometer initialization failure!");
  if(errorStatus & 0x22) Serial.print("Accelerometer initialization failure!");
  if(errorStatus & 0x24) Serial.print("Gyro initialization failure!");
  if(errorStatus & 0x30) Serial.print("Math error!");
  if(errorStatus & 0x80) Serial.print("Invalid sample rate!");
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
  }
 
  if(passThru) {
  // If intPin goes high, all data registers have new data
//  if (digitalRead(intACC2)) {  // On interrupt, read data
    readAccelData(accelCount);  // Read the x/y/z adc values
 
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes; // + accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes; // + accelBias[1];   
    az = (float)accelCount[2]*aRes; // + accelBias[2]; 
 // } 
//  if (digitalRead(intGYRO2)) {  // On interrupt, read data
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;  
    gz = (float)gyroCount[2]*gRes;   
 // }
//  if (digitalRead(intDRDYM)) {  // On interrupt, read data
    readMagData(magCount);  // Read the x/y/z adc values
    
    // Calculate the magnetometer values in milliGauss
    // Temperature-compensated magnetic field is in 16 LSB/microTesla
    mx = (float)magCount[0]*mRes - magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes - magBias[1];  
    mz = (float)magCount[2]*mRes - magBias[2]; 
 //   }
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
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  -my,  mx, mz);
//  if(passThru)MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -my, mx, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)mx); 
    Serial.print(" my = "); Serial.print( (int)my); 
    Serial.print(" mz = "); Serial.print( (int)mz); Serial.println(" mG");
    
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
    }               
 

// tempCount = readTempData();  // Read the gyro adc values
//    temperature = ((float) tempCount) / 333.87 + 21.0; // Gyro chip temperature in degrees Centigrade
   // Print temperature in degrees Centigrade      
//    Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
   if(passThru) {
    D1 = MS5637Read(ADC_D1, OSR);  // get raw pressure value
    D2 = MS5637Read(ADC_D2, OSR);  // get raw temperature value
    dT = D2 - Pcal[5]*pow(2,8);    // calculate temperature difference from reference
    OFFSET = Pcal[2]*pow(2, 17) + dT*Pcal[4]/pow(2,6);
    SENS = Pcal[1]*pow(2,16) + dT*Pcal[3]/pow(2,7);
 
    Temperature = (2000 + (dT*Pcal[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
//
// Second order corrections
    if(Temperature > 20) 
    {
      T2 = 5*dT*dT/pow(2, 38); // correction for high temperatures
      OFFSET2 = 0;
      SENS2 = 0;
    }
    if(Temperature < 20)                   // correction for low temperature
    {
      T2      = 3*dT*dT/pow(2, 33); 
      OFFSET2 = 61*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
      SENS2   = 29*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
    } 
    if(Temperature < -15)                      // correction for very low temperature
    {
      OFFSET2 = OFFSET2 + 17*(100*Temperature + 1500)*(100*Temperature + 1500);
      SENS2 = SENS2 + 9*(100*Temperature + 1500)*(100*Temperature + 1500);
    }
 // End of second order corrections
 //
     Temperature = Temperature - T2/100;
     OFFSET = OFFSET - OFFSET2;
     SENS = SENS - SENS2;
 
     Pressure = (((D1*SENS)/pow(2, 21) - OFFSET)/pow(2, 15))/100;  // Pressure in mbar or kPa

     float altitude = 145366.45*(1. - pow((Pressure/1013.25), 0.190284));
   
     if(SerialDebug) {
     Serial.print("Digital temperature value = "); Serial.print( (float)Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius
     Serial.print("Digital temperature value = "); Serial.print(9.*(float) Temperature/5. + 32., 2); Serial.println(" F"); // temperature in degrees Fahrenheit
     Serial.print("Digital pressure value = "); Serial.print((float) Pressure, 2);  Serial.println(" mbar");// pressure in millibar
     Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
    }
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

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 125 DPS (100), 250 DPS (011), 500 DPS (010), 1000 DPS (001), and 2000 DPS (000). 
    case GFS_125DPS:
          gRes = 124.87/32768.0; // per data sheet, not exactly 125!?
          break;
    case GFS_250DPS:
          gRes = 249.75/32768.0;
          break;
    case GFS_500DPS:
          gRes = 499.5/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 999.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 1998.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (0011), 4 Gs (0101), 8 Gs (1000), and 16 Gs  (1100). 
        // BMX055 ACC data is signed 12 bit
    case AFS_2G:
          aRes = 2.0/2048.0;
          break;
    case AFS_4G:
          aRes = 4.0/2048.0;
          break;
    case AFS_8G:
          aRes = 8.0/2048.0;
          break;
    case AFS_16G:
          aRes = 16.0/2048.0;
          break;
  }
}

float uint32_reg_to_float (uint8_t *buf)
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

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(BMX055_ACC_ADDRESS, BMX055_ACC_D_X_LSB, 6, &rawData[0]);       // Read the six raw data registers into data array
  if((rawData[0] & 0x01) && (rawData[2] & 0x01) && (rawData[4] & 0x01)) {  // Check that all 3 axes have new data
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 4;  // Turn the MSB and LSB into a signed 12-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 4;  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 4; 
  }
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BMX055_GYRO_ADDRESS, BMX055_GYRO_RATE_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

void readMagData(int16_t * magData)
{
  int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
  uint16_t data_r = 0;
  uint8_t rawData[8];  // x/y/z hall magnetic field data, and Hall resistance data
  readBytes(BMX055_MAG_ADDRESS, BMX055_MAG_XOUT_LSB, 8, &rawData[0]);  // Read the eight raw data registers sequentially into data array
    if(rawData[6] & 0x01) { // Check if data ready status bit is set
    mdata_x = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 3;  // 13-bit signed integer for x-axis field
    mdata_y = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 3;  // 13-bit signed integer for y-axis field
    mdata_z = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 1;  // 15-bit signed integer for z-axis field
    data_r = (uint16_t) (((uint16_t)rawData[7] << 8) | rawData[6]) >> 2;  // 14-bit unsigned integer for Hall resistance
   
   // calculate temperature compensated 16-bit magnetic fields
   temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
   magData[0] = ((int16_t)((((int32_t)mdata_x) *
				((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
			     (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
			   ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_x2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
			(((int16_t)dig_x1) << 3);

   temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
   magData[1] = ((int16_t)((((int32_t)mdata_y) *
				((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) + 
			     (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
		           ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_y2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
			(((int16_t)dig_y1) << 3);
   magData[2] = (((((int32_t)(mdata_z - dig_z4)) << 15) - ((((int32_t)dig_z3) * ((int32_t)(((int16_t)data_r) -
	((int16_t)dig_xyz1))))>>2))/(dig_z2 + ((int16_t)(((((int32_t)dig_z1) * ((((int16_t)data_r) << 1)))+(1<<15))>>16))));
    }
  }

int16_t readACCTempData()
{
  uint8_t c =  readByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_TEMP);  // Read the raw data register 
  return ((int16_t)((int16_t)c << 8)) >> 8 ;  // Turn the byte into a signed 8-bit integer
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
 
 
 
 
void trimBMX055()  // get trim values for magnetometer sensitivity
{ 
  uint8_t rawData[2];  //placeholder for 2-byte trim data
  dig_x1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X1);
  dig_x2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X2);
  dig_y1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y1);
  dig_y2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y2);
  dig_xy1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY1);
  dig_xy2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY2);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z1_LSB, 2, &rawData[0]);   
  dig_z1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);  
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z2_LSB, 2, &rawData[0]);   
  dig_z2 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z3_LSB, 2, &rawData[0]);   
  dig_z3 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z4_LSB, 2, &rawData[0]);   
  dig_z4 = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_XYZ1_LSB, 2, &rawData[0]);   
  dig_xyz1 = (uint16_t) (((uint16_t)rawData[1] << 8) | rawData[0]);  
}


void initBMX055()
{  
   // start with all sensors in default mode with all registers reset
   writeByte(BMX055_ACC_ADDRESS,  BMX055_ACC_BGW_SOFTRESET, 0xB6);  // reset accelerometer
   delay(1000); // Wait for all registers to reset 

   // Configure accelerometer
   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_RANGE, Ascale & 0x0F); // Set accelerometer full range
   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_BW, ACCBW & 0x0F);     // Set accelerometer bandwidth
   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_HBW, 0x00);              // Use filtered data

//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_EN_1, 0x10);           // Enable ACC data ready interrupt
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_OUT_CTRL, 0x04);       // Set interrupts push-pull, active high for INT1 and INT2
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_MAP_1, 0x02);        // Define INT1 (intACC1) as ACC data ready interrupt
//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_INT_MAP_1, 0x80);          // Define INT2 (intACC2) as ACC data ready interrupt

//   writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_BGW_SPI3_WDT, 0x06);       // Set watchdog timer for 50 ms
 
 // Configure Gyro
 // start by resetting gyro, better not since it ends up in sleep mode?!
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BGW_SOFTRESET, 0xB6); // reset gyro
// delay(100);
 // Three power modes, 0x00 Normal, 
 // set bit 7 to 1 for suspend mode, set bit 5 to 1 for deep suspend mode
 // sleep duration in fast-power up from suspend mode is set by bits 1 - 3
 // 000 for 2 ms, 111 for 20 ms, etc.
//  writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM1, 0x00);  // set GYRO normal mode
//  set GYRO sleep duration for fast power-up mode to 20 ms, for duty cycle of 50%
//  writeByte(BMX055_ACC_ADDRESS, BMX055_GYRO_LPM1, 0x0E);  
 // set bit 7 to 1 for fast-power-up mode,  gyro goes quickly to normal mode upon wake up
// can set external wake-up interrupts on bits 5 and 4
// auto-sleep wake duration set in bits 2-0, 001 4 ms, 111 40 ms
//  writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_LPM2, 0x00);  // set GYRO normal mode
// set gyro to fast wake up mode, will sleep for 20 ms then run normally for 20 ms 
// and collect data for an effective ODR of 50 Hz, other duty cycles are possible but there
// is a minimum wake duration determined by the bandwidth duration, e.g.,  > 10 ms for 23Hz gyro bandwidth
//  writeByte(BMX055_ACC_ADDRESS, BMX055_GYRO_LPM2, 0x87);   

 writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_RANGE, Gscale);  // set GYRO FS range
 writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BW, GODRBW);     // set GYRO ODR and Bandwidth

// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_EN_0, 0x80);  // enable data ready interrupt
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_EN_1, 0x04);  // select push-pull, active high interrupts
// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_INT_MAP_1, 0x80); // select INT3 (intGYRO1) as GYRO data ready interrupt 

// writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BGW_SPI3_WDT, 0x06); // Enable watchdog timer for I2C with 50 ms window


// Configure magnetometer 
writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x82);  // Softreset magnetometer, ends up in sleep mode
delay(100);
writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // Wake up magnetometer
delay(100);

writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3); // Normal mode
//writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2, MODR << 3 | 0x02); // Forced mode

//writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_INT_EN_2, 0x84); // Enable data ready pin interrupt, active high

// Set up four standard configurations for the magnetometer
  switch (Mmode)
  {
    case lowPower:
         // Low-power
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x01);  // 3 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x02);  // 3 repetitions (oversampling)
          break;
    case Regular:
          // Regular
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x04);  //  9 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x16);  // 15 repetitions (oversampling)
          break;
    case enhancedRegular:
          // Enhanced Regular
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x07);  // 15 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x22);  // 27 repetitions (oversampling)
          break;
    case highAccuracy:
          // High Accuracy
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY, 0x17);  // 47 repetitions (oversampling)
          writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,  0x51);  // 83 repetitions (oversampling)
          break;
  }
}

void fastcompaccelBMX055(float * dest1) 
{
  writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x80); // set all accel offset compensation registers to zero
  writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_SETTING, 0x20);  // set offset targets to 0, 0, and +1 g for x, y, z axes
  writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x20); // calculate x-axis offset

  byte c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  while(!(c & 0x10)) {   // check if fast calibration complete
  c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  delay(10);
}
  writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x40); // calculate y-axis offset

  c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  while(!(c & 0x10)) {   // check if fast calibration complete
  c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  delay(10);
}
  writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL, 0x60); // calculate z-axis offset

  c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  while(!(c & 0x10)) {   // check if fast calibration complete
  c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
  delay(10);
}

  int8_t compx = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_X);
  int8_t compy = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Y);
  int8_t compz = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Z);

  dest1[0] = (float) compx/128.; // accleration bias in g
  dest1[1] = (float) compy/128.; // accleration bias in g
  dest1[2] = (float) compz/128.; // accleration bias in g
}
/*
// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalBMX055(float * dest1, float * dest2)
{  
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t samples, ii;
  
  Serial.println("Calibrating gyro...");
 
  // First get gyro bias
  byte c = readByte(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G);
  writeByte(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G, c | 0x40);     // Enable gyro FIFO  
  delay(200);                                                       // Wait for change to take effect
  writeByte(BMX055G_ADDRESS, BMX055G_FIFO_CTRL_REG_G, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples
  
  samples = (readByte(BMX055G_ADDRESS, BMX055G_FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    int16_t gyro_temp[3] = {0, 0, 0};
    readBytes(BMX055G_ADDRESS, BMX055G_OUT_X_L_G, 6, &data[0]);
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
  
  c = readByte(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G);
  writeByte(BMX055G_ADDRESS, BMX055G_CTRL_REG5_G, c & ~0x40);   //Disable gyro FIFO  
  delay(200);
  writeByte(BMX055G_ADDRESS, BMX055G_FIFO_CTRL_REG_G, 0x00);  // Enable gyro bypass mode
 
   Serial.println("Calibrating accel...");
 
  // now get the accelerometer bias
  c = readByte(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM);
  writeByte(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM, c | 0x40);     // Enable gyro FIFO  
  delay(200);                                                       // Wait for change to take effect
  writeByte(BMX055XM_ADDRESS, BMX055XM_FIFO_CTRL_REG, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples
  
  samples = (readByte(BMX055XM_ADDRESS, BMX055XM_FIFO_SRC_REG) & 0x1F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(BMX055XM_ADDRESS, BMX055XM_OUT_X_L_A, 6, &data[0]);
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
  
  c = readByte(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM);
  writeByte(BMX055XM_ADDRESS, BMX055XM_CTRL_REG0_XM, c & ~0x40);   //Disable accel FIFO  
  delay(200);
  writeByte(BMX055XM_ADDRESS, BMX055XM_FIFO_CTRL_REG, 0x00);  // Enable accel bypass mode
}
*/
void magcalBMX055(float * dest1) 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};
 
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
   sample_count = 128;
   for(ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readMagData(mag_temp);
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
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
  writeByte(BMX055M_ADDRESS, BMX055M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);
 */
   Serial.println("Mag Calibration done!");
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





// I2C communication with the MS5637 is a little different from that with the MPU9250 and most other sensors
// For the MS5637, we write commands, and the MS5637 sends data in response, rather than directly reading
// MS5637 registers

        void MS5637Reset()
        {
        Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
	Wire.write(MS5637_RESET);                // Put reset command in Tx buffer
	Wire.endTransmission();                  // Send the Tx buffer
        }
        
        void MS5637PromRead(uint16_t * destination)
        {
        uint8_t data[2] = {0,0};
        for (uint8_t ii = 0; ii < 7; ii++) {
          Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
          Wire.write(0xA0 | ii << 1);              // Put PROM address in Tx buffer
          Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
	  uint8_t i = 0;
          Wire.requestFrom(MS5637_ADDRESS, 2);   // Read two bytes from slave PROM address 
	  while (Wire.available()) {
          data[i++] = Wire.read(); }               // Put read results in the Rx buffer
          destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); // construct PROM data for return to main program
        }
        }

        uint32_t MS5637Read(uint8_t CMD, uint8_t OSR)  // temperature data read
        {
        uint8_t data[3] = {0,0,0};
        Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
        Wire.write(CMD | OSR);                  // Put pressure conversion command in Tx buffer
        Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
        
        switch (OSR)
        {
          case ADC_256: delay(1); break;  // delay for conversion to complete
          case ADC_512: delay(3); break;
          case ADC_1024: delay(4); break;
          case ADC_2048: delay(6); break;
          case ADC_4096: delay(10); break;
          case ADC_8192: delay(20); break;
        }
       
        Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
        Wire.write(0x00);                        // Put ADC read command in Tx buffer
        Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
        Wire.requestFrom(MS5637_ADDRESS, 3);     // Read three bytes from slave PROM address 
	while (Wire.available()) {
        data[i++] = Wire.read(); }               // Put read results in the Rx buffer
        return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program
        }



unsigned char MS5637checkCRC(uint16_t * n_prom)  // calculate checksum from PROM register contents
{
  int cnt;
  unsigned int n_rem = 0;
  unsigned char n_bit;
  
  n_prom[0] = ((n_prom[0]) & 0x0FFF);  // replace CRC byte by 0 for checksum calculation
  n_prom[7] = 0;
  for(cnt = 0; cnt < 16; cnt++)
  {
    if(cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
    else         n_rem ^= (unsigned short)  (n_prom[cnt>>1]>>8);
    for(n_bit = 8; n_bit > 0; n_bit--)
    {
        if(n_rem & 0x8000)    n_rem = (n_rem<<1) ^ 0x3000;
        else                  n_rem = (n_rem<<1);
    }
  }
  n_rem = ((n_rem>>12) & 0x000F);
  return (n_rem ^ 0x00);
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


// I2C read/write functions for the MPU9250 and AK8963 sensors

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
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
