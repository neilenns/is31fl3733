#include "is31fl3733.hpp"

IS31FL3733::IS31FL3733(uint8_t addr1, uint8_t addr2, i2c_function read_function, i2c_function write_function)
{
  address = IS31FL3733_I2C_ADDR(addr1, addr1);
  i2c_read_reg = read_function;
  i2c_write_reg = write_function;
}

uint8_t IS31FL3733::ReadCommonReg(uint8_t reg_addr)
{
  uint8_t reg_value;

  // Read value from register.
  i2c_read_reg(address, reg_addr, &reg_value, sizeof(uint8_t));
  // Return register value.
  return reg_value;
}

void IS31FL3733::WriteCommonReg(uint8_t reg_addr, uint8_t reg_value)
{
  // Write value to register.
  i2c_write_reg(address, reg_addr, &reg_value, sizeof(uint8_t));
}

void IS31FL3733::SelectPage(uint8_t page)
{
  // Unlock Command Register.
  WriteCommonReg(IS31FL3733_PSWL, IS31FL3733_PSWL_ENABLE);
  // Select requested page in Command Register.
  WriteCommonReg(IS31FL3733_PSR, page);
}

uint8_t IS31FL3733::ReadPagedReg(uint16_t reg_addr)
{
  uint8_t reg_value;

  // Select register page.
  SelectPage(IS31FL3733_GET_PAGE(reg_addr));
  // Read value from register.
  i2c_read_reg(address, IS31FL3733_GET_ADDR(reg_addr), &reg_value, sizeof(uint8_t));
  // Return register value.
  return reg_value;
}

void IS31FL3733::WritePagedReg(uint16_t reg_addr, uint8_t reg_value)
{
  // Select register page.
  SelectPage(IS31FL3733_GET_PAGE(reg_addr));
  // Write value to register.
  i2c_write_reg(address, IS31FL3733_GET_ADDR(reg_addr), &reg_value, sizeof(uint8_t));
}

void IS31FL3733::WritePagedRegs(uint16_t reg_addr, uint8_t *values, uint8_t count)
{
  // Select registers page.
  SelectPage(IS31FL3733_GET_PAGE(reg_addr));
  // Write values to registers.
  i2c_write_reg(address, IS31FL3733_GET_ADDR(reg_addr), values, count);
}

void IS31FL3733::Init()
{
  // Read reset register to reset device.
  ReadPagedReg(IS31FL3733_RESET);
  // Clear software reset in configuration register.
  WritePagedReg(IS31FL3733_CR, IS31FL3733_CR_SSD);
  // Clear state of all LEDs in internal buffer and sync buffer to device.
  SetLEDState(IS31FL3733_CS, IS31FL3733_SW, IS31FL3733_LED_STATE_OFF);
}

void IS31FL3733::SetGCC(uint8_t gcc)
{
  // Write gcc value to Global Current Control (GCC) register.
  WritePagedReg(IS31FL3733_GCC, gcc);
}

void IS31FL3733::SetSWPUR(IS31FL3733_RESISTOR resistor)
{
  // Write resistor value to SWPUR register.
  WritePagedReg(IS31FL3733_SWPUR, resistor);
}

void IS31FL3733::SetCSPDR(IS31FL3733_RESISTOR resistor)
{
  // Write resistor value to CSPDR register.
  WritePagedReg(IS31FL3733_CSPDR, resistor);
}

void IS31FL3733::SetLEDState(uint8_t cs, uint8_t sw, IS31FL3733_LED_STATE state)
{
  uint8_t offset;

  // Check SW boundaries.
  if (sw < IS31FL3733_SW)
  {
    // Check CS boundaries.
    if (cs < IS31FL3733_CS)
    {
      // Set state of individual LED.
      // Calculate LED bit offset.
      offset = (sw << 1) + (cs / 8);
      // Update state of LED in internal buffer.
      if (state == IS31FL3733_LED_STATE_OFF)
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
      WritePagedReg(IS31FL3733_LEDONOFF + offset, leds[offset]);
    }
    else
    {
      // Set state of full row selected by SW.
      // Calculate row offset.
      offset = sw << 1;
      // Update state of row LEDs in internal buffer.
      if (state == IS31FL3733_LED_STATE_OFF)
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
      WritePagedRegs(IS31FL3733_LEDONOFF + offset, &leds[offset], IS31FL3733_CS / 8);
    }
  }
  else
  {
    // Check CS boundaries.
    if (cs < IS31FL3733_CS)
    {
      // Set state of full column selected by CS.
      for (sw = 0; sw < IS31FL3733_SW; sw++)
      {
        // Calculate LED bit offset.
        offset = (sw << 1) + (cs / 8);
        // Update state of LED in internal buffer.
        if (state == IS31FL3733_LED_STATE_OFF)
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
        WritePagedReg(IS31FL3733_LEDONOFF + offset, leds[offset]);
      }
    }
    else
    {
      // Set state of all LEDs.
      for (sw = 0; sw < IS31FL3733_SW; sw++)
      {
        // Update state of all LEDs in internal buffer.
        if (state == IS31FL3733_LED_STATE_OFF)
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
      WritePagedRegs(IS31FL3733_LEDONOFF, leds, IS31FL3733_SW * IS31FL3733_CS / 8);
    }
  }
}

void IS31FL3733::SetLEDPWM(uint8_t cs, uint8_t sw, uint8_t value)
{
  uint8_t offset;

  // Check SW boundaries.
  if (sw < IS31FL3733_SW)
  {
    // Check CS boundaries.
    if (cs < IS31FL3733_CS)
    {
      // Set PWM of individual LED.
      // Calculate LED offset.
      offset = sw * IS31FL3733_CS + cs;
      // Write LED PWM value to device register.
      WritePagedReg(IS31FL3733_LEDPWM + offset, value);
    }
    else
    {
      // Set PWM of full row selected by SW.
      for (cs = 0; cs < IS31FL3733_CS; cs++)
      {
        // Calculate LED offset.
        offset = sw * IS31FL3733_CS + cs;
        // Write LED PWM value to device register.
        WritePagedReg(IS31FL3733_LEDPWM + offset, value);
      }
    }
  }
  else
  {
    // Check CS boundaries.
    if (cs < IS31FL3733_CS)
    {
      // Set PWM of full column selected by CS.
      for (sw = 0; sw < IS31FL3733_SW; sw++)
      {
        // Calculate LED offset.
        offset = sw * IS31FL3733_CS + cs;
        // Write LED PWM value to device register.
        WritePagedReg(IS31FL3733_LEDPWM + offset, value);
      }
    }
    else
    {
      // Set PWM of all LEDs.
      for (sw = 0; sw < IS31FL3733_SW; sw++)
      {
        for (cs = 0; cs < IS31FL3733_CS; cs++)
        {
          // Calculate LED offset.
          offset = sw * IS31FL3733_CS + cs;
          // Write LED PWM value to device register.
          WritePagedReg(IS31FL3733_LEDPWM + offset, value);
        }
      }
    }
  }
}

IS31FL3733_LED_STATUS IS31FL3733::GetLEDStatus(uint8_t cs, uint8_t sw)
{
  uint8_t offset;

  // Check CS and SW boundaries.
  if ((cs < IS31FL3733_CS) && (sw < IS31FL3733_SW))
  {
    // Calculate LED bit offset.
    offset = (sw << 1) + (cs / 8);
    // Get Open status from device register.
    if (ReadPagedReg(IS31FL3733_LEDOPEN + offset) & (0x01 << (cs % 8)))
    {
      return IS31FL3733_LED_STATUS_OPEN;
    }
    // Get Short status from device register.
    if (ReadPagedReg(IS31FL3733_LEDSHORT + offset) & (0x01 << (cs % 8)))
    {
      return IS31FL3733_LED_STATUS_SHORT;
    }
  }
  else
  {
    // Unknown status for nonexistent LED.
    return IS31FL3733_LED_STATUS_UNKNOWN;
  }
  return IS31FL3733_LED_STATUS_NORMAL;
}

void IS31FL3733::SetState(uint8_t *states)
{
  uint8_t sw;
  uint8_t cs;
  uint8_t offset;

  // Set state of all LEDs.
  for (sw = 0; sw < IS31FL3733_SW; sw++)
  {
    for (cs = 0; cs < IS31FL3733_CS; cs++)
    {
      // Calculate LED bit offset.
      offset = (sw << 1) + (cs / 8);
      // Update state of LED in internal buffer.
      if (states[sw * IS31FL3733_CS + cs] == 0)
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
  WritePagedRegs(IS31FL3733_LEDONOFF, leds, IS31FL3733_SW * IS31FL3733_CS / 8);
}

void IS31FL3733::SetPWM(uint8_t *values)
{
  // Write LED PWM values to device registers.
  WritePagedRegs(IS31FL3733_LEDPWM, values, IS31FL3733_SW * IS31FL3733_CS);
}
