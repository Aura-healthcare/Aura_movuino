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
#include <ArduinoLowPower.h>

// create ble serial instance
BLESerial bleSerial = BLESerial();

// I2C address of the MPU-6050
const int MPU_addr = 0x69;

// TODO: I2C address of the BME680
// const int BME_addr; // TODO

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void setup() {
  // Custom services and characteristics can be added as well
  bleSerial.setLocalName("MOVUINO");
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register

  // Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  Serial.write("hello from Movuino!");

  // Initialize BLE led
  pinMode(BLE_LED, OUTPUT);
  // Bettery
  pinMode(A2, INPUT);

  // Start BLE
  // TODO: only if enough battery
  bleSerial.begin();

  // TODO: Shutdown BME680 needs to get the i2c addr --> using the i2c scanner (need serial to work)
  // Wire.beginTransmission(BME_addr);
  // Wire.write(0x74);
  // Wire.write(0x00);
  // Wire.endTransmission();
}

void read_mpu() {
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

void loop() {
  // Battery value and battery voltage
  int BatteryValue = analogRead(A2);
  float voltage = 0.0002014283 * BatteryValue;
  float batteryVoltage= voltage*4.3;
  
  // TODO: only if enough battery
  bleSerial.poll();
  read_mpu();
  spam();

  // Handle the BLE led. Blink when advertising
  if (bleSerial.status() == ADVERTISING) {
    digitalWrite(BLE_LED, LOW);
    delay(200);
    digitalWrite(BLE_LED, HIGH);
    delay(200);
  } else { // If we are not advertising, we are connected
    digitalWrite(BLE_LED, HIGH);
  }

  // TODO: only if not enough battery
  // Triggers a 2000 ms sleep (the device will be woken up only by the registered wakeup sources and by internal RTC)
  LowPower.sleep(2000);
}


// Forward received from Serial to BLESerial and vice versa
void forward() {
  if (bleSerial && Serial) {
    int byte;
    while ((byte = bleSerial.read()) > 0) Serial.write((char)byte);
    while ((byte = Serial.read()) > 0) bleSerial.write((char)byte);
  }
}

// Echo all received data back
void loopback() {
  if (bleSerial) {
    int byte;
    while ((byte = bleSerial.read()) > 0) bleSerial.write(byte);
  }
}

// Periodically sent time stamps
void spam() {
  if (bleSerial) {
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

    delay(35);
  }
}
