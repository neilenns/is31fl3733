/**
 * @file abm_arduino.cpp
 * @author Neil Enns (neile@live.com)
 * @brief Demonstrates how to use the IS31FL3733 chip to enable ABM mode for all LEDs using an Arduino
 * with interrupt detection of when ABM completes.
 * @version 1.0.3
 * @date 2021-11-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Arduino.h>
#include <Wire.h>

#include "is31fl3733.hpp"

using namespace IS31FL3733;

// Arduino pin for the SDB line which is set high to enable the IS31FL3733 chip.
const uint8_t SDB_PIN = 4;
// Arduino pin for the IS13FL3733 interrupt pin.
const uint8_t INTB_PIN = 3;

// Function prototypes for the read and write functions defined later in the file.
uint8_t i2c_read_reg(const uint8_t i2c_addr, const uint8_t reg_addr, uint8_t *buffer, const uint8_t length);
uint8_t i2c_write_reg(const uint8_t i2c_addr, const uint8_t reg_addr, const uint8_t *buffer, const uint8_t count);

// Create a new driver with the address pins tied to ground, and provide the I2C read and write
// functions.
IS31FL3733Driver driver(ADDR::GND, ADDR::GND, &i2c_read_reg, &i2c_write_reg);

/**
 * @brief Finite state machine states for the LEDs.
 * 
 */
enum LedState
{
  ABMNotStarted, //< Before ABM starts running.
  ABMRunning,    //< While ABM is running.
  ABMComplete,   //< After the ABM complete interrupt fires.
  LEDOn          //< ABM is complete and LEDs are on.
};

/**
 * @brief Current state of the finite state machine.
 * 
 */
volatile auto ledState = LedState::ABMNotStarted;

/**
 * @brief Read a buffer of data from the specified register.
 * 
 * @param i2c_addr I2C address of the device to read the data from.
 * @param reg_addr Address of the register to read from.
 * @param buffer Buffer to read the data into.
 * @param length Length of the buffer.
 * @return uint8_t 
 */
uint8_t i2c_read_reg(const uint8_t i2c_addr, const uint8_t reg_addr, uint8_t *buffer, const uint8_t length)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();
  byte bytesRead = Wire.requestFrom(i2c_addr, length);
  for (int i = 0; i < bytesRead && i < length; i++)
  {
    buffer[i] = Wire.read();
  }
  return bytesRead;
}

/**
 * @brief Writes a buffer to the specified register. It is up to the caller to ensure the count of
 * bytes to write doesn't exceed 31, which is the Arduino's write buffer size (32) minus one byte for
 * the register address.
 * 
 * @param i2c_addr I2C address of the device to write the data to.
 * @param reg_addr Address of the register to write to.
 * @param buffer Pointer to an array of bytes to write.
 * @param count Number of bytes in the buffer.
 * @return uint8_t 0 if success, non-zero on error.
 */
uint8_t i2c_write_reg(const uint8_t i2c_addr, const uint8_t reg_addr, const uint8_t *buffer, const uint8_t count)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.write(buffer, count);
  return Wire.endTransmission();
}

/**
 * @brief Interrupt handler for when ABM finishes.
 * 
 */
void abm_completed()
{
  ledState = LedState::ABMComplete;
}

/**
 * @brief Arduino initialization.
 * 
 */
void setup()
{
  // Initialize serial and I2C.
  Serial.begin(115200);
  Wire.begin();

  // Enable the IS31FL3733 chip by setting the SDB pin high.
  pinMode(SDB_PIN, OUTPUT);
  digitalWrite(SDB_PIN, HIGH);

  // Register for interrupts when ABM completes.
  pinMode(INTB_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTB_PIN), abm_completed, CHANGE);

  // Wait for serial to connect so the debug output can be seen.
  while (!Serial)
    ; // Waiting for Serial Monitor

  Serial.print("\nIS31FL3733B test of driver at address 0x");
  Serial.println(driver.GetI2CAddress(), HEX);
  Serial.println("Waiting 5 seconds to begin.");

  // This gives enough time to start up a connected logic analyzer
  delay(5000);

  Serial.println("Initializing");
  driver.Init();

  Serial.println("Setting global current control to half");
  driver.SetGCC(127);

  Serial.println("Setting PWM state for all LEDs to full power in 5 seconds...");
  delay(5000);
  driver.SetLEDMatrixPWM(255);
  Serial.println("Done");
  delay(5000);

  Serial.println("Setting PWM state for all LEDs to half power in 5 seconds...");
  delay(5000);
  driver.SetLEDMatrixPWM(128);
  Serial.println("Done");
  delay(5000);

  Serial.println("Turning on all LEDs");
  driver.SetLEDMatrixState(LED_STATE::ON);

  Serial.println("Configure all LEDs for ABM1");
  driver.SetLEDMode(CS_LINES, SW_LINES, LED_MODE::ABM1);

  ABM_CONFIG ABM1;

  ABM1.T1 = ABM_T1::T1_840MS;
  ABM1.T2 = ABM_T2::T2_840MS;
  ABM1.T3 = ABM_T3::T3_840MS;
  ABM1.T4 = ABM_T4::T4_840MS;
  ABM1.Tbegin = ABM_LOOP_BEGIN::LOOP_BEGIN_T4;
  ABM1.Tend = ABM_LOOP_END::LOOP_END_T3;
  ABM1.Times = 2;

  // Write ABM structure parameters to device registers.
  driver.ConfigABM(ABM_NUM::NUM_1, &ABM1);

  // Enable interrupts when ABM completes and auto-clear them after 8ms
  driver.WriteCommonReg(COMMONREGISTER::IMR, IMR_IAB);

  // Start ABM mode operation.
  driver.StartABM();
  ledState = LedState::ABMRunning;
}

/**
 * @brief Arduino loop.
 * 
 */
void loop()
{
  // Simple finite state machine to switch LEDs on after ABM finishes running.
  switch (ledState)
  {
  case LedState::ABMNotStarted:
  {
    Serial.println("ABM not started");
    delay(500);
    break;
  }
  case LedState::ABMRunning:
  {
    Serial.println("ABM running");
    delay(500);
    break;
  }
  case LedState::ABMComplete:
  {
    // Read the interrupt status to force the interrupt to clear.
    uint8_t interruptStatus = driver.ReadCommonReg(COMMONREGISTER::ISR);

    // Check and see if ABM1 is the ABM that finished.
    if (interruptStatus & ISR_ABM1)
    {
      Serial.println("ABM1 completed");
      Serial.println("Configure all LEDs for full on");
      driver.SetLEDMode(CS_LINES, SW_LINES, LED_MODE::PWM);

      ledState = LedState::LEDOn;
    }
    break;
  }
  case LedState::LEDOn:
  {
    Serial.println("LEDs on");
    delay(500);
    break;
  }
  }
}