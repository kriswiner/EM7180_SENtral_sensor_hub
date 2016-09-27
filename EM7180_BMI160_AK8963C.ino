* EM7180_BMI160_AK8963C_t3 Basic Example Code
 by: Kris Winer
 date: April 20, 2016
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 The EM7180 SENtral sensor hub is not a motion sensor, but rather takes raw sensor data from a variety of motion sensors,
 in this case the BMI160 and AK8963C, and does sensor fusion with quaternions as its output. The SENtral loads firmware from the
 on-board M24512DFC 512 kbit EEPROM upon startup, configures and manages the sensors on its dedicated master I2C bus,
 and outputs scaled sensor data (accelerations, rotation rates, and magnetic fields) as well as quaternions and
 heading/pitch/roll, if selected.
 
 This sketch demonstrates basic EM7180 SENtral functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms to compare with the hardware sensor fusion results.
 Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is specifically for the Teensy 3.1 Mini Add-On shield with the EM7180 SENtral sensor hub as master,
 the BMI160+AK8963C 9-axis motion sensor (accel/gyro/mag) as slave, and an M24512DFC
 512kbit (64 kByte) EEPROM as slave all connected via I2C. 
 
 This sketch uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 
 SDA and SCL has external pull-up resistors (to 3.3V).
 
 Hardware setup:
 EM7180 Mini Add-On ------- Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- 17
 SCL ----------------------- 16
 GND ---------------------- GND
 
 Note: All the sensors on this board are I2C sensor and uses the Teensy 3.1 i2c_t3.h Wire library. 
 Because the sensors are not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 */
#include <i2c_t3.h>
#include <SPI.h>

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU6500 and MPU9250 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

// Bosch BMI160 accel/gyro
// see https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI160-DS000-07.pdf
//
#define BMI160_CHIP_ID        0x00
#define BMI160_ERR_REG        0x02
#define BMI160_PMU_STATUS     0x03
#define BMI160_GYRO_DATA      0x0C // 0x04 - 0x17 0x0C is start of gyro  data
#define BMI160_ACCEL_DATA     0x12 // 0x04 - 0x17 0x12 is start of accel data
#define BMI160_SENSORTIME     0x18 // 0x18 - 0x1A
#define BMI160_STATUS         0x1B
#define BMI160_INT_STATUS     0x1C // 0x1C - 0x1F
#define BMI160_TEMPERATURE    0x20 // 0x20 - 0x21
#define BMI160_FIFO_LENGTH    0x22 // 0x22 - 0x23
#define BMI160_FIFO_DATA      0x24 // 1024 bytes
#define BMI160_ACC_CONF       0x40
#define BMI160_ACC_RANGE      0x41
#define BMI160_GYR_CONF       0x42
#define BMI160_GYR_RANGE      0x43
#define BMI160_MAG_CONF       0x44
#define BMI160_FIFO_DOWNS     0x45
#define BMI160_FIFO_CONFIG    0x46 // 0x46 - 0x47
#define BMI160_MAG_IF         0x4B // 0x4B - 0x4F
#define BMI160_INT_EN         0x50 // 0x50 - 0x52
#define BMI160_INT_OUT_CTRL   0x53
#define BMI160_INT_LATCH      0x54
#define BMI160_INT_MAP        0x55 // 0x55 - 0x57
#define BMI160_INT_DATA       0x58 // 0x58 - 0x59
#define BMI160_INT_LOWHIGH    0x5A // 0x5A - 0x5E
#define BMI160_INT_MOTION     0x5F // 0x5F - 0x62
#define BMI160_INT_TAP        0x63 // 0x63 - 0x64
#define BMI160_INT_ORIENT     0x65 // 0x65 - 0x66
#define BMI160_INT_FLAT       0x67 // 0x67 - 0x68
#define BMI160_FOC_CONF       0x69
#define BMI160_CONF           0x6A
#define BMI160_IF_CONF        0x6B
#define BMI160_PMU_TRIGGER    0x6C
#define BMI160_SELF_TEST      0x6D
#define BMI160_NV_CONF        0x70
#define BMI160_OFFSET         0x71 // 0x71 - 0x77
#define BMI160_OFFSET_CONF    0x77
#define BMI160_STEP_CNT       0x78 // 0x78 - 0x79
#define BMI160_STEP_CONF      0x7A // 0x7A - 0x7B
#define BMI160_CMD            0x7E


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
#define EM7180_ADDRESS      0x28   // Address of the EM7180 SENtral sensor hub
#define M24512DFM_DATA_ADDRESS   0x50   // Address of the 500 page M24512DFM EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
#define M24512DFM_IDPAGE_ADDRESS 0x58   // Address of the single M24512DFM lockable EEPROM ID page
#define BMI160_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
#define  AFS_2G 0x03
#define  AFS_4G 0x05
#define  AFS_8G 0x08
#define  AFS_16G 0x0C

enum AODR {
 Arate0 = 0,
 Arate1Hz,  // really 25/32
 Arate2Hz,  // really 25/16
 Arate3Hz, // really 25/8
 Arate6_25Hz,  
 Arate12_5Hz,
 Arate25Hz,
 Arate50Hz,
 Arate100Hz,
 Arate200Hz,
 Arate400Hz,
 Arate800Hz,
 Arate1600Hz
};

enum ABW {
  ABW_4X = 0, // 4 times oversampling ~ 10% of ODR
  ABW_2X,     // 2 times oversampling ~ 20% of ODR
  ABW_1X      // 1 times oversampling ~ 40% of ODR
};

enum Gscale {
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS
};

enum GODR {
  Grate25Hz = 6,
  Grate50Hz,
  Grate100Hz,
  Grate200Hz,
  Grate400Hz,
  Grate800Hz,
  Grate1600Hz,
  Grate3200Hz
};

enum GBW {
  GBW_4X = 0, // 4 times oversampling ~ 10% of ODR
  GBW_2X,     // 2 times oversampling ~ 20% of ODR
  GBW_1X      // 1 times oversampling ~ 40% of ODR
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

//
// Specify sensor full scale
uint8_t Gscale = GFS_250DPS, GODR = Grate200Hz, GBW = GBW_2X;
uint8_t Ascale = AFS_2G, AODR = Arate200Hz, ABW = ABW_2X;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int myLed     = 13;  // LED on the Teensy 3.1

// BMI160 variables
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float Quat[4] = {0, 0, 0, 0}; // quaternion data register
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
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
uint8_t param[4];                         // used for param transfer
uint16_t EM7180_mag_fs, EM7180_acc_fs, EM7180_gyro_fs; // EM7180 sensor full scale ranges

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

bool passThru = true;

void setup()
{
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
    if(count > 3) break;
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
  
  delay(1000); // give some time to read the screen
  
  // Check sensor status
  uint8_t sensorStatus = readByte(EM7180_ADDRESS, EM7180_SensorStatus);
  Serial.print(" EM7180 sensor status = "); Serial.println(sensorStatus);
  if(sensorStatus & 0x01) Serial.println("Magnetometer not acknowledging!");
  if(sensorStatus & 0x02) Serial.println("Accelerometer not acknowledging!");
  if(sensorStatus & 0x04) Serial.println("Gyro not acknowledging!");
  if(sensorStatus & 0x10) Serial.println("Magnetometer ID not recognized!");
  if(sensorStatus & 0x20) Serial.println("Accelerometer ID not recognized!");
  if(sensorStatus & 0x40) Serial.println("Gyro ID not recognized!");
  
  Serial.print("Actual MagRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualMagRate)); Serial.println(" Hz"); 
  Serial.print("Actual AccelRate = "); Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualAccelRate)); Serial.println(" Hz"); 
  Serial.print("Actual GyroRate = "); Serial.print(10*readByte(EM7180_ADDRESS, EM7180_ActualGyroRate)); Serial.println(" Hz"); 

  delay(1000); // give some time to read the screen
   
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

  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("BMI160 6-axis motion sensor...");
  byte c = readByte(BMI160_ADDRESS, BMI160_CHIP_ID); 
  Serial.print("BMI160 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0xD1, HEX);
   
  delay(1000); 
  
  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

  delay(1000); 

  if ((c == 0xD1) && (d == 0x48) ) // WHO_AM_I should always be ACC/GYRO = 0xD1, MAG = 0x48 
  {  
  Serial.println("BMI160 and AK8963C are online...");

  delay(1000); 
   
  // get sensor resolutions, only need to do this once
   getAres();
   getGres();
   getMres();
   
   Serial.println(" Calibrate gyro and accel");
   accelgyrofastcalBMI160(accelBias, gyroBias); // Calibrate gyro and accelerometers, load biases in bias registers
   Serial.println("accel biases (mg)"); Serial.println(3.9*accelBias[0]); Serial.println(3.9*accelBias[1]); Serial.println(3.9*accelBias[2]);
   Serial.println("gyro biases (dps)"); Serial.println(0.061*gyroBias[0]); Serial.println(0.061*gyroBias[1]); Serial.println(0.061*gyroBias[2]);
   delay(1000);

   initBMI160(); 
   Serial.println("BMI160 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

   // Check power status of BMI160
   uint8_t pwr_status = readByte(BMI160_ADDRESS, BMI160_PMU_STATUS);
   uint8_t acc_pwr = (pwr_status & 0x30) >> 4;
   if (acc_pwr == 0x00) Serial.println("Accel Suspend Mode");
   if (acc_pwr == 0x01) Serial.println("Accel Normal Mode");
   if (acc_pwr == 0x02) Serial.println("Accel Low Power Mode");
   uint8_t gyr_pwr = (pwr_status & 0x0C) >> 2;
   if (gyr_pwr == 0x00) Serial.println("Gyro Suspend Mode");
   if (gyr_pwr == 0x01) Serial.println("Gyro Normal Mode");
   if (gyr_pwr == 0x03) Serial.println("Gyro Fast Start Up Mode");
    delay(1000);

   
   BMI160SelfTest(SelfTest); // Start by performing self test and reporting values
   Serial.println("result of self test should be at least 2 g!");
   Serial.print("x-axis self test: acceleration delta is : "); Serial.print(SelfTest[0],1); Serial.println(" g");
   Serial.print("y-axis self test: acceleration delta is : "); Serial.print(SelfTest[1],1); Serial.println(" g");
   Serial.print("z-axis self test: acceleration delta is : "); Serial.print(SelfTest[2],1); Serial.println(" g");
   delay(1000);

   AK8963SelfTest();
   
  // Get magnetometer calibration from AK8963 ROM
   initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
   if(SerialDebug) {
   Serial.println("Calibration values: ");
   Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
   Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
   Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
  }
  
   magcalAK8963(magBias);
   Serial.println("mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]); 
   delay(1000); // add delay to see results before serial spew of data
  }
  else
  {
    Serial.print("Could not connect to BMI160: 0x");
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
  }
 
  if(passThru) {
  if (readByte(BMI160_ADDRESS, BMI160_STATUS) & 0x40) {  // check if new gyro data
    readAccelGyroData(gyroCount, accelCount);  // Read the x/y/z adc values
 
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes;  
    az = (float)accelCount[2]*aRes;  
 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;  
    gz = (float)gyroCount[2]*gRes;   
  }
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) {  // Check magnetometer data ready bit
    readMagData(magCount);  // Read the x/y/z adc values
    
    // Calculate the magnetometer values in milliGauss
    // Temperature-compensated magnetic field is in 16 LSB/microTesla
    mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
    mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2]; 
    }
  } 
 
 
 // keep track of rates
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  // Check to make sure the EM7180 is running properly
 // uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus);
 // if(eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
 // if(eventStatus & 0x02) Serial.println(" EM7180 Error");
//  if(eventStatus & 0x04) Serial.println(" EM7180 new quaternion result");
 // if(eventStatus & 0x08) Serial.println(" EM7180 new mag result");
 // if(eventStatus & 0x10) Serial.println(" EM7180 new accel result");
 // if(eventStatus & 0x20) Serial.println(" EM7180 new gyro result"); 


  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Use NED orientaton convention where N is direction of arrow on the board
  // AK8963C has N = -y, E = -x, and D = -z if top of board == North
  // BMI160 has N = x, E - -y, and D - -z for the gyro, and opposite by convention for the accel 
  // This rotation can be modified to allow any convenient orientation convention.
  // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  -my,  -mx, -mz);
//  if(passThru)MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  -my,  -mx, -mz);

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
    Serial.print("Q0 = "); Serial.print(Quat[0]);
    Serial.print(" Qx = "); Serial.print(Quat[1]); 
    Serial.print(" Qy = "); Serial.print(Quat[2]); 
    Serial.print(" Qz = "); Serial.println(Quat[3]); 
    }               
 

 tempCount = readTempData();  // Read the gyro adc values
    temperature = ((float) tempCount) / 512.0 + 23.0; // Gyro chip temperature in degrees Centigrade
   // Print temperature in degrees Centigrade      
    Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
   
    
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
    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / PI;
    //Hardware AHRS:
    Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);   
    Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
    Roll  = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
    Pitch *= 180.0f / PI;
    Yaw   *= 180.0f / PI; 
    Yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
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
 
    digitalWrite(myLed, !digitalRead(myLed));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
    }

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

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


void readSENtralQuatData(float * destination)
{
  uint8_t rawData[16];  // x/y/z quaternion register data stored here
  readBytes(EM7180_ADDRESS, EM7180_QX, 16, &rawData[0]);       // Read the sixteen raw data registers into data array
  destination[1] = uint32_reg_to_float (&rawData[0]);
  destination[2] = uint32_reg_to_float (&rawData[4]);
  destination[3] = uint32_reg_to_float (&rawData[8]);
  destination[0] = uint32_reg_to_float (&rawData[12]);  // SENtral stores quats as qx, qy, qz, q0!

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

void getMres() {
  switch (Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4219./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4219./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_125DPS:
          gRes = 125.0/32768.0;
          break;
     case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
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
	// 2 Gs (00), 4 Gs (05), 8 Gs (08), and 16 Gs  (0C). 
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void readAccelGyroData(int16_t * dest1, int16_t * dest2)
{
  uint8_t rawData[12];  // x/y/z accel register data stored here
  readBytes(BMI160_ADDRESS, BMI160_GYRO_DATA, 12, &rawData[0]);  // Read the twelve raw data registers into data array
  dest1[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn gyro MSB and LSB into a signed 16-bit value
  dest1[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  dest1[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 

  dest2[0] = ((int16_t)rawData[7] << 8) | rawData[6] ;  // Turn accel MSB and LSB into a signed 16-bit value
  dest2[1] = ((int16_t)rawData[9] << 8) | rawData[8] ;  
  dest2[2] = ((int16_t)rawData[11] << 8) | rawData[10] ; 
}


void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
   }
  }
}


int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(BMI160_ADDRESS, BMI160_TEMPERATURE, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a 16-bit value
}
  
       
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z mag calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(20);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(20);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(20);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(20);
}

void AK8963SelfTest()
{
  int16_t SelfTestData[3];  // x/y/z self test result stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(20);

  writeByte(AK8963_ADDRESS, AK8963_ASTC, 0x40); // Enable self test
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x18); // enter self test mode, use 16-bit data

  while(!(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)); // wait for data ready bit
  readMagData(SelfTestData);
  
  writeByte(AK8963_ADDRESS, AK8963_ASTC, 0x00); // Disable self test
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(20);

  Serial.print("x-axis self test = "); Serial.print(SelfTestData[0]); Serial.println(" should be +/- 200");
  Serial.print("y-axis self test = "); Serial.print(SelfTestData[1]); Serial.println(" should be +/- 200");
  Serial.print("z-axis self test = "); Serial.print(SelfTestData[2]); Serial.println(" should be -3200 to -800");
}


void initBMI160()
{  
 // configure accel and gyro
  writeByte(BMI160_ADDRESS, BMI160_CMD, 0x11); // Set accel in normal mode operation
  delay(50); // Wait for accel to reset 
  writeByte(BMI160_ADDRESS, BMI160_CMD, 0x15); // Set gyro in normal mode operation
  delay(100); // Wait for gyro to reset 
 // Define accel full scale and sample rate
  writeByte(BMI160_ADDRESS, BMI160_ACC_RANGE, Ascale);
  writeByte(BMI160_ADDRESS, BMI160_ACC_CONF, ABW << 4 | AODR);
 // Define gyro full scale and sample rate
  writeByte(BMI160_ADDRESS, BMI160_GYR_RANGE, Gscale);
  writeByte(BMI160_ADDRESS, BMI160_GYR_CONF, GBW << 4 | GODR);
}


void accelgyrofastcalBMI160(float * dest1, float * dest2)
{
  uint8_t rawData[7] = {0, 0, 0, 0, 0, 0, 0};
  
  writeByte(BMI160_ADDRESS, BMI160_FOC_CONF, 0x4 | 0x30 | 0x0C | 0x01); // Enable gyro cal and accel cal with 0, 0, 1 as reference
  delay(20);
  writeByte(BMI160_ADDRESS, BMI160_CMD, 0x03); // start fast calibration
  delay(50);
  if(readByte(BMI160_ADDRESS, BMI160_ERR_REG) & 0x40) {  // check if dropped command
    Serial.println("Dropped fast offset compensation command!");
    return;
  }
 
  while(!(readByte(BMI160_ADDRESS, BMI160_STATUS) & 0x08)); // wait for fast compensation data ready bit
  if((readByte(BMI160_ADDRESS, BMI160_STATUS) & 0x08)) {
   
    readBytes(BMI160_ADDRESS, BMI160_OFFSET, 7, &rawData[0]);  // Read the seven raw data registers into data array
    dest1[0] = (float)(((int16_t)(rawData[0] << 8 ) | 0x00) >> 8);  // Turn accel offset into a signed 8-bit value
    dest1[1] = (float)(((int16_t)(rawData[1] << 8 ) | 0x00) >> 8);  
    dest1[2] = (float)(((int16_t)(rawData[2] << 8 ) | 0x00) >> 8); 

    dest2[0] = (float)((((int16_t)(rawData[7] & 0x03) << 8) | rawData[3]) >> 6);  // Turn gyro offset MSB and LSB into a signed 10-bit value
    dest2[1] = (float)((((int16_t)(rawData[7] & 0x0C) << 8) | rawData[4]) >> 6);  
    dest2[2] = (float)((((int16_t)(rawData[7] & 0x30) << 8) | rawData[5]) >> 6); 

    writeByte(BMI160_ADDRESS, BMI160_OFFSET_CONF, 0x40);  // enable use of offset registers in data output
  }
  else {
    Serial.println("couldn't get offsets!");
  }
}



void magcalAK8963(float * dest1) 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0X8000, 0X8000, 0X8000}, mag_min[3] = {0X7FFF, 0X7FFF, 0X7FFF}, mag_temp[3] = {0, 0, 0};
 
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
   sample_count = 512;
   for(ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(11);  // at 100 Hz ODR, new mag data is available every 10 ms
   }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];          

   Serial.println("Mag Calibration done!");
}


void BMI160SelfTest(float * destination)
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint16_t selfTestp[3], selfTestm[3];
     
// Enable Gyro Self Test
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x10); // Enable Gyro Self Test
  delay(100);
  uint8_t result = readByte(BMI160_ADDRESS, BMI160_STATUS);
  if(result & 0x02) {
    Serial.println("Gyro Self test passed!");
  }
  else {
     Serial.println("Gyro Self test failed!");  
  }
   // disable self test
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x00 ); // disable Self Test  
  delay(100);
  
// Configure for Accel Self test
   writeByte(BMI160_ADDRESS, BMI160_ACC_RANGE, AFS_8G); // set range to 8 g
   writeByte(BMI160_ADDRESS, BMI160_ACC_CONF, 0x2C ); // Configure accel for Self Test

// Enable Accel Self Test-positive deflection
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x08 | 0x04 | 0x01 ); // Enable accel Self Test positive
  delay(100);
  readBytes(BMI160_ADDRESS, BMI160_ACCEL_DATA, 6, &rawData[0]);  // Read the six raw data registers into data array
  selfTestp[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn accel MSB and LSB into a signed 16-bit value
  selfTestp[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  selfTestp[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 

  // disable self test
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x00 ); // disable Self Test  
  delay(100);

// Enable Accel Self Test-negative deflection
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x08 | 0x01 ); // Enable accel Self Test negative
  delay(100);
  readBytes(BMI160_ADDRESS, BMI160_ACCEL_DATA, 6, &rawData[0]);  // Read the six raw data registers into data array
  selfTestm[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn accel MSB and LSB into a signed 16-bit value
  selfTestm[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  selfTestm[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 

//  Serial.println(selfTestp[0]);Serial.println(selfTestp[1]);Serial.println(selfTestp[2]);
 // Serial.println(selfTestm[0]);Serial.println(selfTestm[1]);Serial.println(selfTestm[2]);
   
  destination[0] = (float)(selfTestp[0] - selfTestm[0])*8./32768.; //construct differences between positve and negative defection, should be ~2 g
  destination[1] = (float)(selfTestp[1] - selfTestm[1])*8./32768.;
  destination[2] = (float)(selfTestp[2] - selfTestm[2])*8./32768.;

 // disable self test
  writeByte(BMI160_ADDRESS, BMI160_SELF_TEST, 0x00 ); // disable Self Test  
  delay(100);

  writeByte(BMI160_ADDRESS, BMI160_ACC_RANGE, Ascale); // set range to original value
  writeByte(BMI160_ADDRESS, BMI160_ACC_CONF, ABW << 4 | AODR); // return accel to original configuration
// End self tests
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


// I2C read/write functions for the MPU6500 and AK8963 sensors

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
