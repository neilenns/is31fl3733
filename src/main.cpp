#include <Arduino.h>
#include <Wire.h>

#include "is31fl3733.hpp"

// Arduino pin for the SDB line, which is set high to enable the IS31FL3733 chip.
#define SDB_PIN 4

// Write a buffer to I2C.
uint8_t i2c_write_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t count)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  byte bytesWritten = Wire.write(buffer, count);
  Wire.endTransmission();

  return bytesWritten;
}

// Read a buffer from I2C.
uint8_t i2c_read_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t count)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();
  byte bytesRead = Wire.requestFrom(i2c_addr, count);
  for (int i = 0; i < bytesRead && i < count; i++)
  {
    buffer[i] = Wire.read();
  }
  return bytesRead;
}

void setup()
{
  IS31FL3733 driver(IS31FL3733_ADDR::ADDR_GND, IS31FL3733_ADDR::ADDR_GND, &i2c_read_reg, &i2c_write_reg);

  // Enable the chip by setting the SDB pin high
  pinMode(SDB_PIN, OUTPUT);
  digitalWrite(SDB_PIN, HIGH);

  // Initialize I2C
  Wire.begin();

  while (!Serial)
    ; // Waiting for Serial Monitor

  Serial.println("\nIS31FL3733B test. Waiting 5 seconds to begin.");
  Serial.print("Device address: 0x");

  // This gives enough time to start up a connected logic analyzer
  delay(5000);

  Serial.println("Initializing");
  driver.Init();
  Serial.println("Set global current control");
  driver.SetGCC(255);
  Serial.println("Setting PWM state for all LEDs");
  driver.SetLEDPWM(IS31FL3733_CS, IS31FL3733_SW, 255);
  Serial.println("Turning on all LEDs");
  driver.SetLEDState(0, 0, IS31FL3733_LED_STATE_ON);
}

void loop()
{
}