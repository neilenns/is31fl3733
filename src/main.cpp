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
  // Enable the chip by setting the SDB pin high
  pinMode(SDB_PIN, OUTPUT);
  digitalWrite(SDB_PIN, HIGH);

  // Initialize I2C
  Wire.begin();

  // Wait for serial to connect so the debug output can be seen.
  while (!Serial)
    ; // Waiting for Serial Monitor

  // Create a new driver with the address pins tied to ground, and provide the I2C read and write
  // functions.
  IS31FL3733 driver(IS31FL3733_ADDR_GND, IS31FL3733_ADDR_GND, &i2c_read_reg, &i2c_write_reg);

  Serial.print("\nIS31FL3733B test of driver at address 0x");
  Serial.println(driver.GetI2CAddress(), HEX);
  Serial.println("Waiting 5 seconds to begin.");

  // This gives enough time to start up a connected logic analyzer
  delay(5000);

  Serial.println("Initializing");
  driver.Init();

  Serial.println("Setting global current control to half");
  driver.SetGCC(127);

  Serial.println("Setting PWM state for all LEDs to full power");
  driver.SetLEDPWM(IS31FL3733_CS, IS31FL3733_SW, 255);

  Serial.println("Turning on all LEDs");
  driver.SetLEDState(IS31FL3733_CS, IS31FL3733_SW, IS31FL3733_LED_STATE_ON);

  Serial.println("Configure all LEDs for ABM1");
  driver.SetLEDMode(IS31FL3733_CS, IS31FL3733_SW, IS31FL3733_LED_MODE_ABM1);

  IS31FL3733_ABM ABM1;

  ABM1.T1 = IS31FL3733_ABM_T1_840MS;
  ABM1.T2 = IS31FL3733_ABM_T2_840MS;
  ABM1.T3 = IS31FL3733_ABM_T3_840MS;
  ABM1.T4 = IS31FL3733_ABM_T4_840MS;
  ABM1.Tbegin = IS31FL3733_ABM_LOOP_BEGIN_T4;
  ABM1.Tend = IS31FL3733_ABM_LOOP_END_T3;
  ABM1.Times = IS31FL3733_ABM_LOOP_FOREVER;

  // Write ABM structure parameters to device registers.
  driver.ConfigABM(IS31FL3733_ABM_NUM_1, &ABM1);
  // Start ABM mode operation.
  driver.StartABM();
}

void loop()
{
}