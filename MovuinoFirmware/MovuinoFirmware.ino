/* Copyright (c) Sandeep Mistry. All rights reserved.
   Licensed under the MIT license. See LICENSE file in the project root for full license information.
   Modified by Chiara Ruggeri <chiara@arduino.org>

   Serial Port over BLE
   Create UART service compatible with Nordic's *nRF Toolbox* and Adafruit's *Bluefruit LE* iOS/Android apps.
   You can also use another board with serial_client example in File->Examples->BLE->Central menu to set up
   a BLE serial connection between two different boards.

   BLESerial class implements same protocols as Arduino's built-in Serial class and can be used as it's wireless
   replacement. Data transfers are routed through a BLE service with TX and RX characteristics. To make the
   service discoverable all UUIDs are NUS (Nordic UART Service) compatible.

   Please note that TX and RX characteristics use Notify and WriteWithoutResponse, so there's no guarantee
   that the data will make it to the other end. However, under normal circumstances and reasonable signal
   strengths everything works well.

   In this example BLE_LED shows the status of the board. It will blink every 200 ms when the board is advertising.
   It will be on when the board is connected to a central. It will be off when the board is disconnected.
*/

#include <BLESerial.h>
#include <Wire.h>
#include "ArduinoLowPower.h"


// create ble serial instance
BLESerial bleSerial = BLESerial();

// I2C address of the MPU-6050
const int MPU_addr = 0x69;

// TODO: I2C address of the BME680
// const int BME_addr; // TODO

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Battery Monitoring
float BatteryLevelAvg = 0.0;
int BatteryLevelArray[3];
uint8_t BatteryLevelCursor = 0;
uint8_t IsBatteryLevelValid = 0;

// Timestamps
int LastSamplingTime = 0;
int LastBatteryReadingTime = 0;


void setup() {
  
  // Custom services and characteristics can be added as well
  bleSerial.setLocalName("MOVUINO");

  // Wake up of MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write((uint8_t)0x00); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  // Initialize BLE led (not used)
  pinMode(BLE_LED, OUTPUT);
  digitalWrite(BLE_LED, HIGH);
  
  // Battery
  pinMode(A2, INPUT);

  // Start BLE
  bleSerial.begin();
}


void loop() {
  
  // Monitoring of battery level
  if (millis() >= LastBatteryReadingTime + 1000)
  {
    // reading timestamp
    LastBatteryReadingTime = millis();

    // update battery informations
    BATTERY_ReadLevel();
  }

  // Is battery low ?
  if (BATTERY_IsLowLevel())
  {
    // 10 sec low power mode
    POWER_SetLowPowerMode(10000);
  }
  // Battery OK, are we advertising or connected ?
  else if (bleSerial.status() != ADVERTISING) 
  {
    if (millis() >= LastSamplingTime + 28)
    {
      // sampling timestamp
      LastSamplingTime = millis();
         
      // reading and sending 
      MPU6050_ReadData();
      BLE_SendData();
    
      // no activity for 23 ms
      LowPower.idle(23);
    }
  }
  else
  {
    // no activity for 10 ms
    LowPower.idle(10);    
  }
}

// Function to read battery level
void BATTERY_ReadLevel() {
  
  // read of analog value
  BatteryLevelArray[BatteryLevelCursor] = analogRead(A2);

  // is average valid ?
  BatteryLevelCursor++;
  if (BatteryLevelCursor >= 3)
  {
    IsBatteryLevelValid = 1;
    BatteryLevelCursor = 0;
  }

  // making decision
  if (IsBatteryLevelValid)
  {
    BatteryLevelAvg = 0.20142831 * (BatteryLevelArray[0] + BatteryLevelArray[1] + BatteryLevelArray[2]) / 3;
    BatteryLevelAvg *= 4.3;
  }
}


// Funciton to detect low battery level
uint8_t BATTERY_IsLowLevel()
{
  uint8_t IsLowLevel = 0;

  if ((1 == IsBatteryLevelValid) && (3500.0 > BatteryLevelAvg))
  {
    IsLowLevel = 1;
  }

  return IsLowLevel;
}


// Function to set movuino system in low power mode
void POWER_SetLowPowerMode(int SleepTime) {

  // Sleep of MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x40);
  Wire.endTransmission(true);

  // end of ble communication
  bleSerial.end();
  
  // Triggers a SleepTime ms sleep (the device will be woken up only by the registered wakeup sources and by internal RTC)
  LowPower.sleep(SleepTime);  

  // Reset of timestamps
  LastSamplingTime = 0;
  LastBatteryReadingTime = 0;

  // start of ble communication
  bleSerial.begin();
}


// Read accelerometer and gyroscope data
void MPU6050_ReadData() {
  
  Wire.beginTransmission(MPU_addr);

  // Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.write(0x3B);
  Wire.endTransmission(false);

  // Request a total of 14 registers
  Wire.requestFrom(MPU_addr, 14, true);

  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

// Periodically sent time stamps
void BLE_SendData() {
  if (bleSerial) 
  {
    bleSerial.poll();
    
    bleSerial.write("L");
    bleSerial.write(AcX >> 8);
    bleSerial.write(AcX);
    bleSerial.write(AcY >> 8);
    bleSerial.write(AcY);
    bleSerial.write(AcZ >> 8);
    bleSerial.write(AcZ);
    bleSerial.write(GyX >> 8);
    bleSerial.write(GyX);
    bleSerial.write(GyY >> 8);
    bleSerial.write(GyY);
    bleSerial.write(GyZ >> 8);
    bleSerial.write(GyZ);
    
    bleSerial.flush();
  }
}
