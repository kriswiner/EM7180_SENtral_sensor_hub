/* EM7180_MPU9250_BMP280_t3 Basic Example Code
 by: Kris Winer
 date: September 11, 2015
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 The EM7180 SENtral sensor hub is not a motion sensor, but rather takes raw sensor data from a variety of motion sensors,
 in this case the MPU9250 (with embedded MPU9250 + AK8963C), and does sensor fusion with quaternions as its output. The SENtral loads firmware from the
 on-board M24512DRC 512 kbit EEPROM upon startup, configures and manages the sensors on its dedicated master I2C bus,
 and outputs scaled sensor data (accelerations, rotation rates, and magnetic fields) as well as quaternions and
 heading/pitch/roll, if selected.
 
 This sketch demonstrates basic EM7180 SENtral functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms to compare with the hardware sensor fusion results.
 Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is specifically for the Teensy 3.1 Mini Add-On shield with the EM7180 SENtral sensor hub as master,
 the MPU9250 9-axis motion sensor (accel/gyro/mag) as slave, a BMP280 pressure/temperature sensor, and an M24512DRC
 512kbit (64 kByte) EEPROM as slave all connected via I2C. The SENtral can use the pressure data in the sensor fusion
 yet and there is a driver for the BMP280 in the SENtral firmware. 
 
 This sketch uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The BMP280 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4k7 resistors are on the EM7180+MPU9250+BMP280+M24512DRC Mini Add-On board for Teensy 3.1.
 
 Hardware setup:
 EM7180 Mini Add-On ------- Teensy 3.1
 VDD ---------------------- 3.3V
 SDA ----------------------- 17
 SCL ----------------------- 16
 GND ---------------------- GND
 INT------------------------ 8
 
 Note: All the sensors n this board are I2C sensor and uses the Teensy 3.1 i2c_t3.h Wire library. 
 Because the sensors are not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 */
 
#include "Arduino.h"
#include "Globals.h"
#include <i2c_t3.h>
#include <SPI.h>

#define SerialDebug true
//#define SerialDebug false

bool passThru = false;
//bool passThru = true;

void setup()
{  
  // Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(100);

  delay(100);
  Serial.begin(115200);
  delay(5000);
      
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);

  // should detect SENtral at 0x28
  I2Cscan();
  
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

  // Give some time to read the screen
  delay(1000);

  // Check which sensors can be detected by the EM7180
  uint8_t featureflag = readByte(EM7180_ADDRESS, EM7180_FeatureFlags);
  if(featureflag & 0x01)  Serial.println("A barometer is installed");
  if(featureflag & 0x02)  Serial.println("A humidity sensor is installed");
  if(featureflag & 0x04)  Serial.println("A temperature sensor is installed");
  if(featureflag & 0x08)  Serial.println("A custom sensor is installed");
  if(featureflag & 0x10)  Serial.println("A second custom sensor is installed");
  if(featureflag & 0x20)  Serial.println("A third custom sensor is installed");

  // Give some time to read the screen
  delay(1000);

  // Check SENtral status, make sure EEPROM upload of firmware was accomplished
  byte STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
  if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
  if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
  if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
  if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
  if(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
  int count = 0;
  while(!STAT)
  {
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
  if(!passThru)
  {
    // Take user input to choose Warm Start or not...
    // "1" from the keboard is ASCII "1" which gives integer value 49
    // "0" from the keboard is ASCII "0" which gives integer value 48
    Serial.println("Send '1' for Warm Start, '0' for no Warm Start");
    serial_input = Serial.read();
    while(!(serial_input == 49) && !(serial_input == 48))
    {
      serial_input = Serial.read();
      delay(500);
    }
    if(serial_input == 49)
    {
      warm_start = 1;
    } else
    {
      warm_start = 0;
    }
    if(warm_start)
    {
      Serial.println("!!!Warm Start active!!!");

      // Put the Sentral in pass-thru mode
      WS_PassThroughMode();

      // Fetch the WarmStart data from the M24512DFM I2C EEPROM
      readSenParams();

      // Take Sentral out of pass-thru mode and re-start algorithm
      WS_Resume();
    } else
    {
      Serial.println("***No Warm Start***");
    }
    // Take user input to choose Warm Start or not...
    // "2" from the keboard is ASCII "1" which gives integer value 50
    // "0" from the keboard is ASCII "0" which gives integer value 48
    Serial.println("Send '2' to apply Accelerometer Cal, '0' to not apply Accelerometer Cal");
    serial_input = Serial.read();
    while(!(serial_input == 50) && !(serial_input == 48))
    {
      serial_input = Serial.read();
      delay(500);
    }
    if(serial_input == 50)
    {
      accel_cal = 1;
    } else
    {
      accel_cal = 0;
    }
    if(accel_cal)
    {
      Serial.println("!!!Accel Cal Active!!!");

      // Put the Sentral in pass-thru mode
      WS_PassThroughMode();

      // Fetch the WarmStart data from the M24512DFM I2C EEPROM
      readAccelCal();
      Serial.print("X-acc max: "); Serial.println(global_conf.accZero_max[0]);
      Serial.print("Y-acc max: "); Serial.println(global_conf.accZero_max[1]);
      Serial.print("Z-acc max: "); Serial.println(global_conf.accZero_max[2]);
      Serial.print("X-acc min: "); Serial.println(global_conf.accZero_min[0]);
      Serial.print("Y-acc min: "); Serial.println(global_conf.accZero_min[1]);
      Serial.print("Z-acc min: "); Serial.println(global_conf.accZero_min[2]);

      // Take Sentral out of pass-thru mode and re-start algorithm
      WS_Resume();
    } else
    {
      Serial.println("***No Accel Cal***");
    }
  }

  // Give some time to read the screen
  delay(1000);
    
  // Set up the SENtral as sensor bus in normal operating mode
  if(!passThru)
  {
    // Set SENtral in initialized state to configure registers
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); 
    
    // Load Accel Cal
    if(accel_cal)
    {
      EM7180_acc_cal_upload();
    }
    
    // Force initialize
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01);

    // Load Warm Start parameters
    if(warm_start)
    {
      EM7180_set_WS_params();
    }

    // Set SENtral in initialized state to configure registers
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); 
  
    //Setup LPF bandwidth (BEFORE setting ODR's)
    writeByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, 0x03); // 41Hz
    writeByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, 0x01); // 184Hz
    
    // Set accel/gyro/mage desired ODR rates
    writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, 0x02); // 100 Hz
    writeByte(EM7180_ADDRESS, EM7180_MagRate, 0x64); // 100 Hz
    writeByte(EM7180_ADDRESS, EM7180_AccelRate, 0x14); // 200/10 Hz
    writeByte(EM7180_ADDRESS, EM7180_GyroRate, 0x14); // 200/10 Hz
    writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | 0x32);  // set enable bit and set Baro rate to 25 Hz

    // Configure operating mode
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data
    
    // Enable interrupt to host upon certain events
    // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
    // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
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
    while(!(param_xfer==0x4A))
    {
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
    while(!(param_xfer==0x4B))
    {
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
    while(!(param_xfer==0x4A))
    {
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
    while(!(param_xfer==0x4B))
    {
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

    // Give some time to read the screen
    delay(1000);
  
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
    Serial.print("Actual BaroRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualBaroRate)); Serial.println(" Hz");
    Serial.println(""); Serial.println("*******************************************");
    Serial.println("Send '1' to store Warm Start configuration");
    Serial.println("*******************************************"); Serial.println("");

    // Give some time to read the screen
    delay(1000);
  }
 
  // If pass through mode desired, set it up here
  if(passThru)
  {
    // Put EM7180 SENtral into pass-through mode
    SENtralPassThroughMode();
    delay(1000);

    // should see all the devices on the I2C bus including two from the EEPROM (ID page and data pages)
    I2Cscan();
 
    // Read first page of EEPROM
    uint8_t data[128];
    M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x00, 0x00, 128, data);
    Serial.println("EEPROM Signature Byte"); 
    Serial.print(data[0], HEX); Serial.println("  Should be 0x2A");
    Serial.print(data[1], HEX); Serial.println("  Should be 0x65");
    for (int i = 0; i < 128; i++)
    {
      Serial.print(data[i], HEX); Serial.print(" ");
    }

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    // Read the WHO_AM_I register, this is a good test of communication
    Serial.println("MPU9250 9-axis motion sensor...");
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
    if (c == 0x71) // WHO_AM_I should always be 0x71
    {  
      Serial.println("MPU9250 is online...");
      MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
      Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
      Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
      Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
      Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
      Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
      Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
      
      delay(1000);
    
     // get sensor resolutions, only need to do this once
     getAres();
     getGres();
     getMres();
    
     Serial.println(" Calibrate gyro and accel");
     accelgyrocalMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
     Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
     Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

     delay(1000);  
   
     initMPU9250(); 
     Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
     writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22); 
     I2Cscan(); // should see all the devices on the I2C bus including two from the EEPROM (ID page and data pages)
 
     // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
     byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
     Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
 
     delay(1000); 
  
     // Get magnetometer calibration from AK8963 ROM
     // Initialize device for active mode read of magnetometer
     initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode....");
  
     magcalMPU9250(magBias, magScale);
     Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]); 
     Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
     delay(2000); // add delay to see results before serial spew of data
   
     if(SerialDebug)
     {
       Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
       Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
       Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
     }
  
     delay(1000);  
  
     // Read the WHO_AM_I register of the BMP280 this is a good test of communication
     // Read WHO_AM_I register for BMP280
     byte f = readByte(BMP280_ADDRESS, BMP280_ID);
     Serial.print("BMP280 "); 
     Serial.print("I AM "); 
     Serial.print(f, HEX); 
     Serial.print(" I should be "); 
     Serial.println(0x58, HEX);
     Serial.println(" ");
  
     delay(1000); 

     // reset BMP280 before initilization
     writeByte(BMP280_ADDRESS, BMP280_RESET, 0xB6);
     
     delay(100);

     // Initialize BMP280 altimeter
     BMP280Init();
     Serial.println("Calibration coeficients:");
     Serial.print("dig_T1 ="); 
     Serial.println(dig_T1);
     Serial.print("dig_T2 ="); 
     Serial.println(dig_T2);
     Serial.print("dig_T3 ="); 
     Serial.println(dig_T3);
     Serial.print("dig_P1 ="); 
     Serial.println(dig_P1);
     Serial.print("dig_P2 ="); 
     Serial.println(dig_P2);
     Serial.print("dig_P3 ="); 
     Serial.println(dig_P3);
     Serial.print("dig_P4 ="); 
     Serial.println(dig_P4);
     Serial.print("dig_P5 ="); 
     Serial.println(dig_P5);
     Serial.print("dig_P6 ="); 
     Serial.println(dig_P6);
     Serial.print("dig_P7 ="); 
     Serial.println(dig_P7);
     Serial.print("dig_P8 ="); 
     Serial.println(dig_P8);
     Serial.print("dig_P9 ="); 
     Serial.println(dig_P9);
  
     delay(1000);
   }
     else
   {
     Serial.print("Could not connect to MPU9250: 0x");
     Serial.println(c, HEX);
     while(1) ; // Loop forever if communication doesn't happen
   }
  }
}

void loop()
{  
  if(!passThru)
  {
    serial_input = Serial.read();
    if (serial_input == 49)
    {
      delay(100);
      EM7180_get_WS_params();
      
      // Put the Sentral in pass-thru mode
      WS_PassThroughMode();

      // Store WarmStart data to the M24512DFM I2C EEPROM
      writeSenParams();

      // Take Sentral out of pass-thru mode and re-start algorithm
      WS_Resume();
      warm_start_saved = 1;
    }
    if (serial_input == 50)
    {
      calibratingA = 512;
    }
      
    // Check event status register, way to chech data ready by polling rather than interrupt
    uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register
  
    // Check for errors
    // Error detected, what is it?
    if(eventStatus & 0x02)
    { 
      uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
      if(!errorStatus)
      {
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
    // new acceleration data available
    if(eventStatus & 0x10)
    { 
      readSENtralAccelData(accelCount);
  
      // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0]*0.000488;  // get actual g value
      ay = (float)accelCount[1]*0.000488;    
      az = (float)accelCount[2]*0.000488;
       
      // Manages accelerometer calibration; is active when calibratingA > 0
      Accel_cal_check();
    }
  
    if(eventStatus & 0x20)
    {
      // new gyro data available  
      readSENtralGyroData(gyroCount);
  
      // Now we'll calculate the gyro value into actual dps's
      gx = (float)gyroCount[0]*0.153;  // get actual dps value
      gy = (float)gyroCount[1]*0.153;    
      gz = (float)gyroCount[2]*0.153;  
    }
    if(eventStatus & 0x08)
    {
      // new mag data available
      readSENtralMagData(magCount);
  
      // Now we'll calculate the mag value into actual G's
      // get actual G value
      mx = (float)magCount[0]*0.305176;
      my = (float)magCount[1]*0.305176;    
      mz = (float)magCount[2]*0.305176;  
    }

    if(eventStatus & 0x04)
    {
      readSENtralQuatData(Quat);
    }

    // get BMP280 pressure
    // new baro data available
    if(eventStatus & 0x40)
    { 
      rawPressure = readSENtralBaroData();
      pressure = (float)rawPressure*0.01f +1013.25f; // pressure in mBar

      // get BMP280 temperature
      rawTemperature = readSENtralTempData();  
      temperature = (float) rawTemperature*0.01;  // temperature in degrees C
    }
  }
 
  if(passThru)
  {
    // If intPin goes high, all data registers have new data
    readAccelData(accelCount);  // Read the x/y/z adc values
 
    // Now we'll calculate the acceleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2];

    // Read the x/y/z adc values
    readGyroData(gyroCount);

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;  
    gz = (float)gyroCount[2]*gRes;   
    readMagData(magCount);  // Read the x/y/z adc values
    
    // Calculate the magnetometer values in milliGauss
    mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
    mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
  } 
 
  // keep track of rates
  Now = micros();

  // set integration time by time elapsed since last filter update
  deltat = ((Now - lastUpdate)/1000000.0f);
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
//  if(passThru)MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -my, mx, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;

    // update LCD once per half-second independent of read rate
    if (delt_t > 500)
    { 

      if(SerialDebug)
      {
        Serial.print("ax = "); Serial.print((int)1000*ax);  
        Serial.print(" ay = "); Serial.print((int)1000*ay); 
        Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
        Serial.print("gx = "); Serial.print( gx, 2); 
        Serial.print(" gy = "); Serial.print( gy, 2); 
        Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
        Serial.print("mx = "); Serial.print( (int)mx); 
        Serial.print(" my = "); Serial.print( (int)my); 
        Serial.print(" mz = "); Serial.print( (int)mz); Serial.println(" mG");
        Serial.println("Software quaternions (ENU):"); 
        Serial.print("q0 = "); Serial.print(q[0]);
        Serial.print(" qx = "); Serial.print(q[1]); 
        Serial.print(" qy = "); Serial.print(q[2]); 
        Serial.print(" qz = "); Serial.println(q[3]); 
        Serial.println("Hardware quaternions (NED):"); 
        Serial.print("Q0 = "); Serial.print(Quat[0]);
        Serial.print(" Qx = "); Serial.print(Quat[1]); 
        Serial.print(" Qy = "); Serial.print(Quat[2]); 
        Serial.print(" Qz = "); Serial.println(Quat[3]); 
      }               
      if(passThru)
      {
        rawPress =  readBMP280Pressure();
        pressure = (float) bmp280_compensate_P(rawPress)/25600.; // Pressure in mbar
        rawTemp =   readBMP280Temperature();
        temperature = (float) bmp280_compensate_T(rawTemp)/100.;
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
     
      // Or define output variable according to the Android system, where heading (0 to 360) is defined by the angle between the y-axis 
      // and True North, pitch is rotation about the x-axis (-180 to +180), and roll is rotation about the y-axis (-90 to +90)
      // In this systen, the z-axis is pointing away from Earth, the +y-axis is at the "top" of the device (cellphone) and the +x-axis
      // points toward the right of the device.
    
      if(SerialDebug)
      {
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
        Serial.println("BMP280:");
        Serial.print("Altimeter temperature = "); 
        Serial.print( temperature, 2); 
        Serial.println(" C"); // temperature in degrees Celsius
        Serial.print("Altimeter temperature = "); 
        Serial.print(9.*temperature/5. + 32., 2); 
        Serial.println(" F"); // temperature in degrees Fahrenheit
        Serial.print("Altimeter pressure = "); 
        Serial.print(pressure, 2);  
        Serial.println(" mbar");// pressure in millibar
        altitude = 145366.45f*(1.0f - pow((pressure/1013.25f), 0.190284f));
        Serial.print("Altitude = "); 
        Serial.print(altitude, 2); 
        Serial.println(" feet");
        Serial.println(" ");
        if(!passThru)
        {
          if(warm_start_saved)
          {
            Serial.println("Warm Start configuration saved!");
          } else
          {
            Serial.println("Send '1' to store Warm Start configuration");
          }
          if(accel_cal_saved > 0)
          {
            Serial.print("Accel Cals Complete:"); Serial.println(accel_cal_saved);
          } else
          {
            Serial.println("Send '2' to store Accel Cal");
          }
        }
      }
      Serial.print(millis()/1000.0, 1);Serial.print(",");
      Serial.print(yaw); Serial.print(",");Serial.print(pitch); Serial.print(",");Serial.print(roll); Serial.print(",");
      Serial.print(Yaw); Serial.print(",");Serial.print(Pitch); Serial.print(",");Serial.println(Roll);  
      digitalWrite(myLed, !digitalRead(myLed));
      count = millis(); 
      sumCount = 0;
      sum = 0;    
    }
}

//===================================================================================================================
//====== Sentral parameter management functions
//===================================================================================================================

void EM7180_set_gyro_FS (uint16_t gyro_fs)
{
  uint8_t bytes[4], STAT;
  bytes[0] = gyro_fs & (0xFF);
  bytes[1] = (gyro_fs >> 8) & (0xFF);
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Unused
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Unused

  // Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
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

  // Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA);

  //Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==0xCA)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_integer_param (uint8_t param, uint32_t param_val)
{
  uint8_t bytes[4], STAT;
  bytes[0] = param_val & (0xFF);
  bytes[1] = (param_val >> 8) & (0xFF);
  bytes[2] = (param_val >> 16) & (0xFF);
  bytes[3] = (param_val >> 24) & (0xFF);
 
  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==param)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_float_param (uint8_t param, float param_val) {
  uint8_t bytes[4], STAT;
  float_to_bytes (param_val, &bytes[0]);

  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==param)) {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void EM7180_set_WS_params()
{
  uint8_t param = 1;
  uint8_t STAT;
  
  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, WS_params.Sen_param[0][0]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, WS_params.Sen_param[0][1]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, WS_params.Sen_param[0][2]);
  writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, WS_params.Sen_param[0][3]);
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);

  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==param))
  {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  for(uint8_t i=1; i<35; i++)
  {
    param = (i+1) | 0x80;
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte0, WS_params.Sen_param[i][0]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte1, WS_params.Sen_param[i][1]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte2, WS_params.Sen_param[i][2]);
    writeByte(EM7180_ADDRESS, EM7180_LoadParamByte3, WS_params.Sen_param[i][3]);
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
    
    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(STAT==param))
    {
      STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);
}

void EM7180_get_WS_params()
{
  uint8_t param = 1;
  uint8_t STAT;

  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
  delay(10);
  
  // Request parameter transfer procedure
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80);
  delay(10);

   // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  while(!(STAT==param))
  {
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  
  // Parameter is the decimal value with the MSB set low (default) to indicate a paramter read processs
  WS_params.Sen_param[0][0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
  WS_params.Sen_param[0][1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
  WS_params.Sen_param[0][2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
  WS_params.Sen_param[0][3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);

  for(uint8_t i=1; i<35; i++)
  {
    param = (i+1);
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
    delay(10);
    
    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(STAT==param))
    {
      STAT = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    WS_params.Sen_param[i][0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    WS_params.Sen_param[i][1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    WS_params.Sen_param[i][2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    WS_params.Sen_param[i][3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
  }
  // Parameter request = 0 to end parameter transfer process
  writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00);

  // Re-start algorithm
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00);
}

void EM7180_acc_cal_upload()
{
  int64_t big_cal_num;
  union
  {
    int16_t cal_num;
    unsigned char cal_num_byte[2];
  };
  
  if(!accel_cal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    big_cal_num = (4096000000/(global_conf.accZero_max[0] - global_conf.accZero_min[0])) - 1000000;
    cal_num = (int16_t)big_cal_num;
  }
  writeByte(EM7180_ADDRESS, EM7180_GP36, cal_num_byte[0]);
  writeByte(EM7180_ADDRESS, EM7180_GP37, cal_num_byte[1]);
  
  if(!accel_cal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    big_cal_num = (4096000000/(global_conf.accZero_max[1] - global_conf.accZero_min[1])) - 1000000;
    cal_num = (int16_t)big_cal_num;
  }
  writeByte(EM7180_ADDRESS, EM7180_GP38, cal_num_byte[0]);
  writeByte(EM7180_ADDRESS, EM7180_GP39, cal_num_byte[1]);  

  if(!accel_cal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    big_cal_num = (4096000000/(global_conf.accZero_max[2] - global_conf.accZero_min[2])) - 1000000;
    cal_num = (int16_t)big_cal_num;
  }
  writeByte(EM7180_ADDRESS, EM7180_GP40, cal_num_byte[0]);
  writeByte(EM7180_ADDRESS, EM7180_GP50, cal_num_byte[1]);

  if(!accel_cal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    big_cal_num = (((2048 - global_conf.accZero_max[0]) + (-2048 - global_conf.accZero_min[0]))*100000)/4096;
    cal_num = (int16_t)big_cal_num;
  }
  writeByte(EM7180_ADDRESS, EM7180_GP51, cal_num_byte[0]);
  writeByte(EM7180_ADDRESS, EM7180_GP52, cal_num_byte[1]);

  if(!accel_cal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    big_cal_num = (((2048 - global_conf.accZero_max[1]) + (-2048 - global_conf.accZero_min[1]))*100000)/4096;
    cal_num = (int16_t)big_cal_num;
  }
  writeByte(EM7180_ADDRESS, EM7180_GP53, cal_num_byte[0]);
  writeByte(EM7180_ADDRESS, EM7180_GP54, cal_num_byte[1]);

  if(!accel_cal)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    big_cal_num = (((2048 - global_conf.accZero_max[2]) + (-2048 - global_conf.accZero_min[2]))*100000)/4096;
    cal_num = -(int16_t)big_cal_num;
  }
  writeByte(EM7180_ADDRESS, EM7180_GP55, cal_num_byte[0]);
  writeByte(EM7180_ADDRESS, EM7180_GP56, cal_num_byte[1]);
}

void WS_PassThroughMode()
{
  uint8_t stat = 0;
  
  // First put SENtral in standby mode
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x01);
  delay(5);
  
  // Place SENtral in pass-through mode
  writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x01);
  delay(5);
  stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
  while(!(stat & 0x01))
  {
    stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
    delay(5);
  }
}

void WS_Resume()
{
  uint8_t stat = 0;
  
  // Cancel pass-through mode
  writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00);
  delay(5);
  stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
  while((stat & 0x01))
  {
    stat = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
    delay(5);
  }

  // Re-start algorithm
  writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00);
  delay(5);
  stat = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
  while((stat & 0x01))
  {
    stat = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
    delay(5);
  }
}

void readSenParams()
{
  uint8_t data[140];
  uint8_t paramnum;
  M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 12, &data[128]); // Page 255
  delay(100);
  M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
  for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
  {
    for (uint8_t i= 0; i < 4; i++)
    {
      WS_params.Sen_param[paramnum][i] = data[(paramnum*4 + i)];
    }
  }
}

void writeSenParams()
{
  uint8_t data[140];
  uint8_t paramnum;
  for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
  {
    for (uint8_t i= 0; i < 4; i++)
    {
      data[(paramnum*4 + i)] = WS_params.Sen_param[paramnum][i];
    }
  }
  M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x80, 12, &data[128]); // Page 255
  delay(100);
  M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x00, 128, &data[0]); // Page 254
}

void Accel_cal_check()
{
  static int64_t a[3] = {0, 0, 0}, b[3] = {0, 0, 0};

  if (calibratingA > 0)
  {
    for (uint8_t axis = 0; axis < 3; axis++)
    {
      if (accelCount[axis] > 1024)
      {
        // Sum up 512 readings
        a[axis] += accelCount[axis];
      }
      if (accelCount[axis] < -1024)
      {
        b[axis] += accelCount[axis];
      }
      // Clear global variables for next reading
      accelCount[axis] = 0;
    }
    
    // Calculate averages, and store values in EEPROM at end of calibration
    if (calibratingA == 1)
    {
      for (uint8_t axis = 0; axis < 3; axis++)
      {
        if (a[axis]>>9 > 1024)
        {
          global_conf.accZero_max[axis] = a[axis]>>9;
        }
        if (b[axis]>>9 < -1024)
        {
          global_conf.accZero_min[axis] = b[axis]>>9;
        }
        a[axis] = 0;
        b[axis] = 0;
      }

      //Write accZero to EEPROM
      delay(100);
      
      // Put the Sentral in pass-thru mode
      WS_PassThroughMode();

      // Store accelerometer calibration data to the M24512DFM I2C EEPROM
      writeAccCal();


      // Take Sentral out of pass-thru mode and re-start algorithm
      WS_Resume();
      accel_cal_saved++;
      if(accel_cal_saved > 6) accel_cal_saved = 0;
    }
    calibratingA--;
  }
}

void readAccelCal()
{
  uint8_t data[12];
  uint8_t axis;

  M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x8c, 12, data); // Page 255
  for (axis = 0; axis < 3; axis++)
  {
    global_conf.accZero_max[axis] = ((int16_t)(data[(2*axis + 1)]<<8) | data[2*axis]);
    global_conf.accZero_min[axis] = ((int16_t)(data[(2*axis + 7)]<<8) | data[(2*axis + 6)]);
  }
}

void writeAccCal()
{
  uint8_t data[12];
  uint8_t axis;
  for (axis = 0; axis < 3; axis++)
  {
    data[2*axis] = (global_conf.accZero_max[axis] & 0xff);
    data[(2*axis + 1)] = (global_conf.accZero_max[axis] >> 8);
    data[(2*axis + 6)] = (global_conf.accZero_min[axis] & 0xff);
    data[(2*axis + 7)] = (global_conf.accZero_min[axis] >> 8);
  }
  M24512DFMwriteBytes(M24512DFM_DATA_ADDRESS, 0x7f, 0x8c, 12, data); // Page 255
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
  destination[0] = uint32_reg_to_float (&rawData[0]);
  destination[1] = uint32_reg_to_float (&rawData[4]);
  destination[2] = uint32_reg_to_float (&rawData[8]);
  destination[3] = uint32_reg_to_float (&rawData[12]);  // SENtral stores quats as qx, qy, qz, q0!

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
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
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
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
  // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
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


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
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
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}
       
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
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


void initMPU9250()
{  
 // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
// Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x02; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
  
 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalMPU9250(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
 // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);                                    

// Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
  
// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFE;
  data[1] = (accel_bias_reg[0])      & 0xFE;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFE;
  data[3] = (accel_bias_reg[1])      & 0xFE;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFE;
  data[5] = (accel_bias_reg[2])      & 0xFE;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void magcalMPU9250(float * dest1, float * dest2) 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {0xFF, 0xFF, 0xFF}, mag_min[3] = {0x7F, 0x7F, 0x7F}, mag_temp[3] = {0, 0, 0};
 
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
   if(Mmode == 0x02) sample_count = 128;
   if(Mmode == 0x06) sample_count = 1500;
   for(ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
   }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
  
   Serial.println("Mag Calibration done!");
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
   float factoryTrim[6];
   uint8_t FS = 0;
   
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
   }
   
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
  else
  {
    Serial.println("ERROR! SENtral not in pass-through mode!");
  }
}

int32_t readBMP280Temperature()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BMP280_ADDRESS, BMP280_TEMP_MSB, 3, &rawData[0]);  
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

int32_t readBMP280Pressure()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BMP280_ADDRESS, BMP280_PRESS_MSB, 3, &rawData[0]);  
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

void BMP280Init()
{
  // Configure the BMP280
  // Set T and P oversampling rates and sensor mode
  writeByte(BMP280_ADDRESS, BMP280_CTRL_MEAS, Tosr << 5 | Posr << 2 | Mode);
  // Set standby time interval in normal mode and bandwidth
  writeByte(BMP280_ADDRESS, BMP280_CONFIG, SBy << 5 | IIRFilter << 2);
  // Read and store calibration data
  uint8_t calib[24];
  readBytes(BMP280_ADDRESS, BMP280_CALIB00, 24, &calib[0]);
  dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
  dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
  dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
  dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
  dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
  dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
  dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
  dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
  dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
  dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
  dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
  dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// “5123” equals 51.23 DegC.
int32_t bmp280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
//fractional bits).
//Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P(int32_t adc_P)
{
  long long var1, var2, p;
  var1 = ((long long)t_fine) - 128000;
  var2 = var1 * var1 * (long long)dig_P6;
  var2 = var2 + ((var1*(long long)dig_P5)<<17);
  var2 = var2 + (((long long)dig_P4)<<35);
  var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
  var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
  if(var1 == 0)
  {
    return 0;
    // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125)/var1;
  var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((long long)dig_P8) * p)>> 19;
  p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
  return (uint32_t)p;
}

//===================================================================================================================
//====== I2C Communication Support Functions
//===================================================================================================================

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
  if(count > 128)
  {
    count = 128;
    Serial.print("Page count cannot be more than 128 bytes!");
  }
  Wire.beginTransmission(device_address);   // Initialize the Tx buffer
  Wire.write(data_address1);                // Put slave register address in Tx buffer
  Wire.write(data_address2);                // Put slave register address in Tx buffer
  for(uint8_t i=0; i < count; i++)
  {
    Wire.write(dest[i]);                    // Put data in Tx buffer
  }
  Wire.endTransmission();                   // Send the Tx buffer
}

uint8_t M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(device_address);         // Initialize the Tx buffer
  Wire.write(data_address1);                      // Put slave register address in Tx buffer
  Wire.write(data_address2);                      // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);               // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(device_address, (size_t)1);   // Read one byte from slave register address 
  data = Wire.read();                             // Fill Rx buffer with result
  return data;                                    // Return data read from slave register
}

void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(device_address);            // Initialize the Tx buffer
  Wire.write(data_address1);                         // Put slave register address in Tx buffer
  Wire.write(data_address2);                         // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);                  // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(device_address, (size_t)count);  // Read bytes from slave register address 
  while (Wire.available())
  {
    dest[i++] = Wire.read();
  }                                                   // Put read results in the Rx buffer
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
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);            // Initialize the Tx buffer
  Wire.write(subAddress);                     // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);           // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
  while (Wire.available())
  {
    dest[i++] = Wire.read();
  }                                           // Put read results in the Rx buffer
}
