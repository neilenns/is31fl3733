#include "is31fl3733.hpp"

namespace IS31FL3733
{
  IS31FL3733Driver::IS31FL3733Driver(ADDR addr1, ADDR addr2, i2c_function read_function, i2c_function write_function)
  {
#ifdef ARDUINO
    // Arduino uses 7 bit I2C addresses without the R/W bit in position 0.
    address = ((I2C_BASE_ADDR) | ((addr2) << 3) | ((addr1) << 1)) >> 1;
#else
    address = ((I2C_BASE_ADDR) | ((addr2) << 3) | ((addr1) << 1));
#endif

    i2c_read_reg = read_function;
    i2c_write_reg = write_function;
  }

  uint8_t IS31FL3733Driver::ReadCommonReg(uint8_t reg_addr)
  {
    uint8_t reg_value;

    // Read value from register.
    i2c_read_reg(address, reg_addr, &reg_value, sizeof(uint8_t));
    // Return register value.
    return reg_value;
  }

  void IS31FL3733Driver::WriteCommonReg(uint8_t reg_addr, uint8_t reg_value)
  {
    // Write value to register.
    i2c_write_reg(address, reg_addr, &reg_value, sizeof(uint8_t));
  }

  void IS31FL3733Driver::SelectPageForRegister(uint16_t reg_addr)
  {
    // Unlock Command Register.
    WriteCommonReg(REGISTERS::PSWL, PSWL_OPTIONS::PSWL_ENABLE);
    // Select requested page in Command Register. The requested page is the
    // top eight bits of the reg_addr.
    WriteCommonReg(REGISTERS::PSR, (uint8_t)(reg_addr >> 8));
  }

  uint8_t IS31FL3733Driver::ReadPagedReg(uint16_t reg_addr)
  {
    uint8_t reg_value;

    // Select register page.
    SelectPageForRegister(reg_addr);
    // Read value from register.
    // The register is the low eight bits of the reg_addr.
    i2c_read_reg(address, (uint8_t)(reg_addr), &reg_value, sizeof(uint8_t));
    // Return register value.
    return reg_value;
  }

  void IS31FL3733Driver::WritePagedReg(uint16_t reg_addr, uint8_t reg_value)
  {
    // Select register page.
    SelectPageForRegister(reg_addr);
    // Write value to register.
    // The register is the low eight bits of the reg_addr.
    i2c_write_reg(address, (uint8_t)(reg_addr), &reg_value, sizeof(uint8_t));
  }

  void IS31FL3733Driver::WritePagedRegs(uint16_t reg_addr, uint8_t *values, uint8_t count)
  {
    // Select registers page.
    SelectPageForRegister(reg_addr);
    // Write values to registers.
    // The register is the low eight bits of the reg_addr.
    i2c_write_reg(address, (uint8_t)(reg_addr), values, count);
  }

  void IS31FL3733Driver::Init()
  {
    // Read reset register to reset device.
    ReadPagedReg(REGISTERS::RESET);
    // Clear software reset in configuration register.
    WritePagedReg(REGISTERS::CR, CR_OPTIONS::CR_SSD);
    // Clear state of all LEDs in internal buffer and sync buffer to device.
    SetLEDState(CS_LINES, SW_LINES, LED_STATE::OFF);
  }

  byte IS31FL3733Driver::GetI2CAddress()
  {
    return address;
  }

  void IS31FL3733Driver::SetGCC(uint8_t gcc)
  {
    // Write gcc value to Global Current Control (GCC) register.
    WritePagedReg(REGISTERS::GCC, gcc);
  }

  void IS31FL3733Driver::SetSWPUR(RESISTOR resistor)
  {
    // Write resistor value to SWPUR register.
    WritePagedReg(REGISTERS::SWPUR, resistor);
  }

  void IS31FL3733Driver::SetCSPDR(RESISTOR resistor)
  {
    // Write resistor value to CSPDR register.
    WritePagedReg(REGISTERS::CSPDR, resistor);
  }

  void IS31FL3733Driver::SetLEDState(uint8_t cs, uint8_t sw, LED_STATE state)
  {
    uint8_t offset;

    // Check SW boundaries.
    if (sw < SW_LINES)
    {
      // Check CS boundaries.
      if (cs < CS_LINES)
      {
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
        WritePagedReg(REGISTERS::LEDONOFF + offset, leds[offset]);
      }
      else
      {
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
        WritePagedRegs(REGISTERS::LEDONOFF + offset, &leds[offset], CS_LINES / 8);
      }
    }
    else
    {
      // Check CS boundaries.
      if (cs < CS_LINES)
      {
        // Set state of full column selected by CS.
        for (sw = 0; sw < SW_LINES; sw++)
        {
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
          WritePagedReg(REGISTERS::LEDONOFF + offset, leds[offset]);
        }
      }
      else
      {
        // Set state of all LEDs.
        for (sw = 0; sw < SW_LINES; sw++)
        {
          // Update state of all LEDs in internal buffer.
          if (state == LED_STATE::OFF)
          {
            // Clear all bits.
            leds[(sw << 1)] = 0x00;
            leds[(sw << 1) + 1] = 0x00;
          }
          else
          {
            // Set all bits.
            leds[(sw << 1)] = 0xFF;
            leds[(sw << 1) + 1] = 0xFF;
          }
        }
        // Write updated LEDs state to device registers.
        WritePagedRegs(REGISTERS::LEDONOFF, leds, SW_LINES * CS_LINES / 8);
      }
    }
  }

  void IS31FL3733Driver::SetLEDPWM(uint8_t cs, uint8_t sw, uint8_t value)
  {
    uint8_t offset;

    // Check SW boundaries.
    if (sw < SW_LINES)
    {
      // Check CS boundaries.
      if (cs < CS_LINES)
      {
        // Set PWM of individual LED.
        // Calculate LED offset.
        offset = sw * CS_LINES + cs;
        // Write LED PWM value to device register.
        WritePagedReg(REGISTERS::LEDPWM + offset, value);
      }
      else
      {
        // Set PWM of full row selected by SW.
        for (cs = 0; cs < CS_LINES; cs++)
        {
          // Calculate LED offset.
          offset = sw * CS_LINES + cs;
          // Write LED PWM value to device register.
          WritePagedReg(REGISTERS::LEDPWM + offset, value);
        }
      }
    }
    else
    {
      // Check CS boundaries.
      if (cs < CS_LINES)
      {
        // Set PWM of full column selected by CS.
        for (sw = 0; sw < SW_LINES; sw++)
        {
          // Calculate LED offset.
          offset = sw * CS_LINES + cs;
          // Write LED PWM value to device register.
          WritePagedReg(REGISTERS::LEDPWM + offset, value);
        }
      }
      else
      {
        // Set PWM of all LEDs.
        for (sw = 0; sw < SW_LINES; sw++)
        {
          for (cs = 0; cs < CS_LINES; cs++)
          {
            // Calculate LED offset.
            offset = sw * CS_LINES + cs;
            // Write LED PWM value to device register.
            WritePagedReg(REGISTERS::LEDPWM + offset, value);
          }
        }
      }
    }
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
      if (ReadPagedReg(REGISTERS::LEDOPEN + offset) & (0x01 << (cs % 8)))
      {
        return LED_STATUS::OPEN;
      }
      // Get Short status from device register.
      if (ReadPagedReg(REGISTERS::LEDSHORT + offset) & (0x01 << (cs % 8)))
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

  void IS31FL3733Driver::SetState(uint8_t *states)
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
    WritePagedRegs(REGISTERS::LEDONOFF, leds, SW_LINES * CS_LINES / 8);
  }

  void IS31FL3733Driver::SetPWM(uint8_t *values)
  {
    // Write LED PWM values to device registers.
    WritePagedRegs(REGISTERS::LEDPWM, values, SW_LINES * CS_LINES);
  }

#ifdef ENABLE_ABM
  void IS31FL3733Driver::SetLEDMode(uint8_t cs, uint8_t sw, LED_MODE mode)
  {
    uint8_t offset;

    // Check SW boundaries.
    if (sw < SW_LINES)
    {
      // Check CS boundaries.
      if (cs < CS_LINES)
      {
        // Set mode of individual LED.
        // Calculate LED offset.
        offset = sw * CS_LINES + cs;
        // Write LED mode to device register.
        WritePagedReg(REGISTERS::LEDABM + offset, mode);
      }
      else
      {
        // Set mode of full row selected by SW.
        for (cs = 0; cs < CS_LINES; cs++)
        {
          // Calculate LED offset.
          offset = sw * CS_LINES + cs;
          // Write LED mode to device register.
          WritePagedReg(REGISTERS::LEDABM + offset, mode);
        }
      }
    }
    else
    {
      // Check CS boundaries.
      if (cs < CS_LINES)
      {
        // Set mode of full column selected by CS.
        for (sw = 0; sw < SW_LINES; sw++)
        {
          // Calculate LED offset.
          offset = sw * CS_LINES + cs;
          // Write LED mode to device register.
          WritePagedReg(REGISTERS::LEDABM + offset, mode);
        }
      }
      else
      {
        // Set mode of all LEDs.
        for (sw = 0; sw < SW_LINES; sw++)
        {
          for (cs = 0; cs < CS_LINES; cs++)
          {
            // Calculate LED offset.
            offset = sw * CS_LINES + cs;
            // Write LED mode to device register.
            WritePagedReg(REGISTERS::LEDABM + offset, mode);
          }
        }
      }
    }
  }

  void IS31FL3733Driver::ConfigABM(ABM_NUM n, ABM_CONFIG *config)
  {
    // Set fade in and fade out time.
    WritePagedReg(n, config->T1 | config->T2);
    // Set hold and off time.
    WritePagedReg(n + 1, config->T3 | config->T4);
    // Set loop begin/end time and high part of loop times.
    WritePagedReg(n + 2, config->Tend | config->Tbegin | ((config->Times >> 8) & 0x0F));
    // Set low part of loop times.
    WritePagedReg(n + 3, config->Times & 0xFF);
  }

  void IS31FL3733Driver::StartABM()
  {
    // Clear B_EN bit in configuration register.
    WritePagedReg(REGISTERS::CR, CR_OPTIONS::CR_SSD);
    // Set B_EN bit in configuration register.
    WritePagedReg(REGISTERS::CR, CR_OPTIONS::CR_BEN | CR_OPTIONS::CR_SSD);
    // Write 0x00 to Time Update Register to update ABM settings.
    WritePagedReg(REGISTERS::TUR, 0x00);
  }
#endif
}