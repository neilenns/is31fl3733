#include "is31fl3733.hpp"

namespace IS31FL3733
{
  IS31FL3733Driver::IS31FL3733Driver(const ADDR addr1, const ADDR addr2, const i2c_read_function read_function, const i2c_write_function write_function)
  {
#ifdef ARDUINO
    // Arduino uses 7 bit I2C addresses without the R/W bit in position 0.
    address = ((I2C_BASE_ADDR) | ((addr2) << 3) | ((addr1) << 1)) >> 1;
    // Arduino's Wire.write() function is limited to 32 bytes in one go.
    // This is set to 31 because one of the bytes gets consumed by the register address
    // getting written to.
    maxI2CWriteBufferSize = 31;
#else
    address = ((I2C_BASE_ADDR) | ((addr2) << 3) | ((addr1) << 1));
    // For all other platforms assume it is possible to write a full page at once.
    maxI2CWriteBufferSize = LED_COUNT;
#endif

    i2c_read_reg = read_function;
    i2c_write_reg = write_function;
  }

  void IS31FL3733Driver::_setFullPagedRegister(const PAGEDREGISTER reg, uint8_t value)
  {
    // On Arduino the maximum buffer size for an I2C write is 32 bytes so this is
    // really wasting RAM on the Arduino. Ideally on Arduino this would only be 32
    // bytes and would get sent repeatedly until LED_COUNT's worth of bytes are sent.
    // The downside to that approach is this code is no longer portable across
    // platforms. So instead this is the full amount of bytes that have to get sent
    // and splitting it into appropriate chunks is left to WritePagedRegs().
    uint8_t values[LED_COUNT];

    memset(values, value, LED_COUNT * sizeof(uint8_t));

    WritePagedRegs(reg, values, LED_COUNT);
  }

  uint8_t IS31FL3733Driver::ReadCommonReg(const COMMONREGISTER reg)
  {
    uint8_t reg_value;

    // Read value from register.
    i2c_read_reg(address, reg, &reg_value, sizeof(uint8_t));
    // Return register value.
    return reg_value;
  }

  void IS31FL3733Driver::WriteCommonReg(const COMMONREGISTER reg, const uint8_t reg_value)
  {
    // Write value to register.
    i2c_write_reg(address, reg, &reg_value, sizeof(uint8_t));
  }

  void IS31FL3733Driver::SelectPageForRegister(const PAGEDREGISTER reg)
  {
    // Unlock Command Register.
    WriteCommonReg(COMMONREGISTER::PSWL, PSWL_OPTIONS::PSWL_ENABLE);
    // Select requested page in Command Register. The requested page is the
    // high byte of reg.
    WriteCommonReg(COMMONREGISTER::PSR, (uint8_t)(reg >> 8));
  }

  uint8_t IS31FL3733Driver::ReadPagedReg(const PAGEDREGISTER reg)
  {
    return ReadPagedReg(reg, 0);
  }

  uint8_t IS31FL3733Driver::ReadPagedReg(const PAGEDREGISTER reg, const uint8_t offset)
  {
    uint8_t reg_value;

    // Select register page.
    SelectPageForRegister(reg);
    // Read value from register.
    // The register is the low eight bits of the reg_addr.
    i2c_read_reg(address, (uint8_t)(reg) + offset, &reg_value, sizeof(uint8_t));
    // Return register value.
    return reg_value;
  }

  void IS31FL3733Driver::WritePagedReg(const PAGEDREGISTER reg, const uint8_t reg_value)
  {
    WritePagedReg(reg, 0, reg_value);
  }

  void IS31FL3733Driver::WritePagedReg(const PAGEDREGISTER reg, const uint8_t offset, const uint8_t reg_value)
  {
    // Select register page.
    SelectPageForRegister(reg);
    // Write value to register.
    // The register is the low eight bits of the reg.
    i2c_write_reg(address, (uint8_t)(reg) + offset, &reg_value, sizeof(uint8_t));
  }

  void IS31FL3733Driver::WritePagedRegs(const PAGEDREGISTER reg, const uint8_t *values, const uint8_t count)
  {
    WritePagedRegs(reg, 0, values, count);
  }

  void IS31FL3733Driver::WritePagedRegs(const PAGEDREGISTER reg, const uint8_t offset, const uint8_t *values, const uint8_t count)
  {
    uint8_t bytesRemaining = count;
    uint8_t bytesToWrite = maxI2CWriteBufferSize;
    uint8_t bytesWritten = 0;

    // Depending on the platform it may not be possible to send the entire list of values at once.
    while (bytesRemaining > 0)
    {
      if (bytesRemaining < maxI2CWriteBufferSize)
      {
        bytesToWrite = bytesRemaining;
      }

      // Select register page.
      SelectPageForRegister(reg);

      // Write values to registers.
      // The register is the low eight bits of the reg_addr.
      i2c_write_reg(address, (uint8_t)(reg) + offset + bytesWritten, &values[bytesWritten], bytesToWrite);

      bytesRemaining -= bytesToWrite;
      bytesWritten += bytesToWrite;
    }
  }

  void IS31FL3733Driver::Init()
  {
    // Read reset register to reset device.
    ReadPagedReg(PAGEDREGISTER::RESET);
    // Clear software reset in configuration register.
    WritePagedReg(PAGEDREGISTER::CR, CR_OPTIONS::CR_SSD);
    // Clear state of all LEDs in internal buffer and sync buffer to device.
    SetLEDMatrixState(LED_STATE::OFF);
  }

  byte IS31FL3733Driver::GetI2CAddress()
  {
    return address;
  }

  void IS31FL3733Driver::SetGCC(const uint8_t gcc)
  {
    // Write gcc value to Global Current Control (GCC) register.
    WritePagedReg(PAGEDREGISTER::GCC, gcc);
  }

  void IS31FL3733Driver::SetSWPUR(const RESISTOR resistor)
  {
    // Write resistor value to SWPUR register.
    WritePagedReg(PAGEDREGISTER::SWPUR, resistor);
  }

  void IS31FL3733Driver::SetCSPDR(const RESISTOR resistor)
  {
    // Write resistor value to CSPDR register.
    WritePagedReg(PAGEDREGISTER::CSPDR, resistor);
  }

  void IS31FL3733Driver::SetLEDSingleState(const uint8_t cs, uint8_t sw, const LED_STATE state)
  {
    uint8_t offset;

    // Set state of individual LED.
    // Calculate LED bit offset.
    offset = (sw << 1) + (cs / 8);

    // Update state of LED in internal buffer.
    if (state == LED_STATE::OFF)
    {
      // Clear bit for selected LED.
      leds[offset] &= ~(0x01 << (cs % 8));
    }
    else
    {
      // Set bit for selected LED.
      leds[offset] |= 0x01 << (cs % 8);
    }

    // Write updated LED state to device register.
    WritePagedReg(PAGEDREGISTER::LEDONOFF, offset, leds[offset]);
  }

  void IS31FL3733Driver::SetLEDRowState(const uint8_t sw, const LED_STATE state)
  {
    uint8_t offset;

    // Set state of full row selected by SW.
    // Calculate row offset.
    offset = sw << 1;
    // Update state of row LEDs in internal buffer.
    if (state == LED_STATE::OFF)
    {
      // Clear 16 bits for selected row LEDs.
      leds[offset] = 0x00;
      leds[offset + 1] = 0x00;
    }
    else
    {
      // Set 16 bits for selected row LEDs.
      leds[offset] = 0xFF;
      leds[offset + 1] = 0xFF;
    }
    // Write updated LEDs state to device registers.
    WritePagedRegs(PAGEDREGISTER::LEDONOFF, offset, &leds[offset], CS_LINES / 8);
  }

  void IS31FL3733Driver::SetLEDColumnState(const uint8_t cs, const LED_STATE state)
  {
    uint8_t offset;

    // Set state of full column selected by CS.
    for (uint8_t row = 0; row < SW_LINES; row++)
    {
      // Calculate LED bit offset.
      offset = (row << 1) + (cs / 8);
      // Update state of LED in internal buffer.
      if (state == LED_STATE::OFF)
      {
        // Clear bit for selected LED.
        leds[offset] &= ~(0x01 << (cs % 8));
      }
      else
      {
        // Set bit for selected LED.
        leds[offset] |= 0x01 << (cs % 8);
      }
      // Write updated LED state to device register.
      WritePagedReg(PAGEDREGISTER::LEDONOFF, offset, leds[offset]);
    }
  }

  void IS31FL3733Driver::SetLEDMatrixState(const LED_STATE state)
  {
    for (uint8_t row = 0; row < SW_LINES; row++)
    {
      // Update state of all LEDs in internal buffer.
      if (state == LED_STATE::OFF)
      {
        // Clear all bits.
        leds[(row << 1)] = 0x00;
        leds[(row << 1) + 1] = 0x00;
      }
      else
      {
        // Set all bits.
        leds[(row << 1)] = 0xFF;
        leds[(row << 1) + 1] = 0xFF;
      }
    }
    // Write updated LEDs state to device registers.
    WritePagedRegs(PAGEDREGISTER::LEDONOFF, leds, SW_LINES * CS_LINES / 8);
  }

  void IS31FL3733Driver::SetLEDSinglePWM(uint8_t cs, uint8_t sw, const uint8_t value)
  {
    // Write LED PWM value to device register.
    WritePagedReg(PAGEDREGISTER::LEDPWM, sw * CS_LINES + cs, value);
  }

  void IS31FL3733Driver::SetLEDRowPWM(uint8_t sw, const uint8_t value)
  {
    uint8_t values[CS_LINES];

    memset(values, value, CS_LINES * sizeof(uint8_t));

    // Write LED PWM value to device register.
    WritePagedRegs(PAGEDREGISTER::LEDPWM, sw * CS_LINES, values, CS_LINES);
  }

  void IS31FL3733Driver::SetLEDColumnPWM(uint8_t cs, const uint8_t value)
  {
    uint8_t offset;

    // Set PWM of full column selected by CS.
    for (uint8_t row = 0; row < SW_LINES; row++)
    {
      // Calculate LED offset.
      offset = row * CS_LINES + cs;

      // Write LED PWM value to device register.
      WritePagedReg(PAGEDREGISTER::LEDPWM, offset, value);
    }
  }

  void IS31FL3733Driver::SetLEDMatrixPWM(const uint8_t value)
  {
    _setFullPagedRegister(PAGEDREGISTER::LEDPWM, value);
  }

  LED_STATUS IS31FL3733Driver::GetLEDStatus(uint8_t cs, uint8_t sw)
  {
    uint8_t offset;

    // Check CS and SW boundaries.
    if ((cs < CS_LINES) && (sw < SW_LINES))
    {
      // Calculate LED bit offset.
      offset = (sw << 1) + (cs / 8);
      // Get Open status from device register.
      if (ReadPagedReg(PAGEDREGISTER::LEDOPEN, offset) & (0x01 << (cs % 8)))
      {
        return LED_STATUS::OPEN;
      }
      // Get Short status from device register.
      if (ReadPagedReg(PAGEDREGISTER::LEDSHORT, offset) & (0x01 << (cs % 8)))
      {
        return LED_STATUS::SHORT;
      }
    }
    else
    {
      // Unknown status for nonexistent LED.
      return LED_STATUS::UNKNOWN;
    }
    return LED_STATUS::NORMAL;
  }

  void IS31FL3733Driver::SetState(const uint8_t *states)
  {
    uint8_t sw;
    uint8_t cs;
    uint8_t offset;

    // Set state of all LEDs.
    for (sw = 0; sw < SW_LINES; sw++)
    {
      for (cs = 0; cs < CS_LINES; cs++)
      {
        // Calculate LED bit offset.
        offset = (sw << 1) + (cs / 8);
        // Update state of LED in internal buffer.
        if (states[sw * CS_LINES + cs] == 0)
        {
          // Clear bit for selected LED.
          leds[offset] &= ~(0x01 << (cs % 8));
        }
        else
        {
          // Set bit for selected LED.
          leds[offset] |= 0x01 << (cs % 8);
        }
      }
    }
    // Write updated LEDs state to device registers.
    WritePagedRegs(PAGEDREGISTER::LEDONOFF, leds, SW_LINES * CS_LINES / 8);
  }

  void IS31FL3733Driver::SetPWM(const uint8_t *values)
  {
    // Write LED PWM values to device registers.
    WritePagedRegs(PAGEDREGISTER::LEDPWM, values, SW_LINES * CS_LINES);
  }

  void IS31FL3733Driver::SetLEDSingleMode(uint8_t cs, uint8_t sw, const LED_MODE mode)
  {
    WritePagedReg(PAGEDREGISTER::LEDABM, sw * CS_LINES + cs, mode);
  }

  void IS31FL3733Driver::SetLEDRowMode(const uint8_t sw, const LED_MODE mode)
  {
    uint8_t values[CS_LINES];

    memset(values, mode, CS_LINES * sizeof(uint8_t));

    // Write LED PWM value to device register.
    WritePagedRegs(PAGEDREGISTER::LEDABM, sw * CS_LINES, values, CS_LINES);
  }

  void IS31FL3733Driver::SetLEDColumnMode(uint8_t cs, const LED_MODE mode)
  {
    uint8_t offset;

    // Set PWM of full column selected by CS.
    for (uint8_t row = 0; row < SW_LINES; row++)
    {
      // Calculate LED offset.
      offset = row * CS_LINES + cs;

      // Write LED PWM value to device register.
      WritePagedReg(PAGEDREGISTER::LEDABM, offset, mode);
    }
  }

  void IS31FL3733Driver::SetLEDMatrixMode(const LED_MODE mode)
  {
    _setFullPagedRegister(PAGEDREGISTER::LEDABM, mode);
  }

  void IS31FL3733Driver::ConfigABM(const ABM_NUM n, const ABM_CONFIG *config)
  {
    // Set fade in and fade out time.
    WritePagedReg((PAGEDREGISTER)n, config->T1 | config->T2);
    // Set hold and off time.
    WritePagedReg((PAGEDREGISTER)n, 1, config->T3 | config->T4);
    // Set loop begin/end time and high part of loop times.
    WritePagedReg((PAGEDREGISTER)n, 2, config->Tend | config->Tbegin | ((config->Times >> 8) & 0x0F));
    // Set low part of loop times.
    WritePagedReg((PAGEDREGISTER)n, 3, config->Times & 0xFF);
  }

  void IS31FL3733Driver::StartABM()
  {
    // Clear B_EN bit in configuration register.
    WritePagedReg(PAGEDREGISTER::CR, CR_OPTIONS::CR_SSD);
    // Set B_EN bit in configuration register.
    WritePagedReg(PAGEDREGISTER::CR, CR_OPTIONS::CR_BEN | CR_OPTIONS::CR_SSD);
    // Write 0x00 to Time Update Register to update ABM settings.
    WritePagedReg(PAGEDREGISTER::TUR, 0x00);
  }
}