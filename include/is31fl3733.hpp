/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Neil Enns. All rights reserved.
 *  Licensed under the MIT License. See LICENSE in the project root for license information.
 *--------------------------------------------------------------------------------------------*/
#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace IS31FL3733
{
  const uint8_t CS_LINES = 16;                   ///< Number of CS lines on the chip.
  const uint8_t SW_LINES = 12;                   ///< Number of SW lines on the chip.
  const uint8_t LED_COUNT = CS_LINES * SW_LINES; ///< Total number of LEDs in the matrix.

  /// @brief Addresses for the common registers.
  enum COMMONREGISTER
  {
    PSR = 0xFD,  ///< Common: page select register. Write only.
    PSWL = 0xFE, ///< Common: page select register write lock. Read/write.
    IMR = 0xF0,  ///< Common: interrupt mask register. Write only.
    ISR = 0xF1,  ///< Common: interrupt status register. Read only.
  };

  /// @brief Addresses for the paged registers. The high byte is the page
  /// for the register. The low byte is the register address.
  enum PAGEDREGISTER
  {
    LEDONOFF = 0x0000, ///< Page 0: On or off state control for each LED. Write only.
    LEDOPEN = 0x0018,  ///< Page 0: Open state for each LED. Read only.
    LEDSHORT = 0x0030, ///< Page 0: Short state for each LED. Read only.
    LEDPWM = 0x0100,   ///< Page 1: PWM duty cycle for each LED. Write only.
    LEDABM = 0x0200,   ///< Page 2: Auto breath mode for each LED. Write only.
    CR = 0x0300,       ///< Page 3: Configuration register. Write only.
    GCC = 0x0301,      ///< Page 3: Global current control register. Write only.
    ABM1CR = 0x0302,   ///< Page 3: Auto breath control register for ABM-1. Write only.
    ABM2CR = 0x0306,   ///< Page 3: Auto breath control register for ABM-2. Write only.
    ABM3CR = 0x030A,   ///< Page 3: Auto breath control register for ABM-3. Write only.
    TUR = 0x030E,      ///< Page 3: Time update register. Write only.
    SWPUR = 0x030F,    ///< Page 3: SWy pull-up resistor selection register. Write only.
    CSPDR = 0x0310,    ///< Page 3: CSx pull-down resistor selection register. Read only.
    RESET = 0x0311,    ///< Page 3: Reset register. Read only.
  };

  /// @brief Bits to control the PSWL register options.
  enum PSWL_OPTIONS
  {
    PSWL_DISABLE = 0x00, ///< Disable write to page select register.
    PSWL_ENABLE = 0xC5   ///< Enable write to page select register.
  };

  /// @brief Bits to control the IMR register options.
  enum IMR_OPTIONS
  {
    IMR_IAC = 0x08, ///< Auto clear interrupt bit.
    IMR_IAB = 0x04, ///< Auto breath interrupt bit.
    IMR_IS = 0x02,  ///< Dot short interrupt bit.
    IMR_IO = 0x01,  ///< Dot open interrupt bit.
  };

  /// @brief Bits to control the ISR register options.
  enum ISR_OPTIONS
  {
    ISR_ABM3 = 0x10, ///< Auto breath mode 3 finish bit
    ISR_ABM2 = 0x08, ///< Auto Breath Mode 2 Finish Bit.
    ISR_ABM1 = 0x04, ///< Auto Breath Mode 1 Finish Bit.
    ISR_SB = 0x02,   ///< Short bit.
    ISR_OB = 0x01,   ///< Open bit.
  };

  /// @brief Bits to control the CR register options.
  enum CR_OPTIONS
  {
    CR_SYNC_MASTER = 0x40, ///< Configure as clock master device.
    CR_SYNC_SLAVE = 0x80,  ///< Configure as clock slave device.
    CR_OSD = 0x04,         ///< Open/short detection enable bit.
    CR_BEN = 0x02,         ///< Auto breath mode enable bit.
    CR_SSD = 0x01,         ///< Software shutdown bit.
  };

  /// @brief The valid connection points for ADDR1 and ADDR2 pins.
  enum ADDR
  {
    GND = 0x00, //< Pin connected to GND.
    SCL = 0x01, //< Pin connected to SCL.
    SDA = 0x02, //< Pin connected to SDA.
    VCC = 0x03  //< Pin connected to VCC.
  };

  /// @brief Values for setting an LED off and on.
  enum LED_STATE
  {
    OFF = 0x00, //< LED is off.
    ON = 0x01   //< LED is on.
  };

  /// @brief The valid LED status states.
  enum LED_STATUS
  {
    NORMAL = 0x00, //< Normal LED status.
    OPEN = 0x01,   //< LED is open.
    SHORT = 0x02,  //< LED is short.
    UNKNOWN = 0x03 //< Unknown LED status.
  };

  /// @brief Pull-up or pull-down resistor values.
  enum RESISTOR
  {
    RESISTOR_OFF = 0x00, //< No resistor.
    RESISTOR_500 = 0x01, //< 0.5 kOhm pull-up resistor.
    RESISTOR_1K = 0x02,  //< 1.0 kOhm pull-up resistor.
    RESISTOR_2K = 0x03,  //< 2.0 kOhm pull-up resistor.
    RESISTOR_4K = 0x04,  //< 4.0 kOhm pull-up resistor.
    RESISTOR_8K = 0x05,  //< 8.0 kOhm pull-up resistor.
    RESISTOR_16K = 0x06, //< 16 kOhm pull-up resistor.
    RESISTOR_32K = 0x07  //< 32 kOhm pull-up resistor.
  };

  /// @brief Maximum number of ABM loop times.
  const int ABM_LOOP_TIMES_MAX = 0x0FFF;

  /// @brief Loop ABM forever.
  const int ABM_LOOP_FOREVER = 0x0000;

  /// @brief Configures the LED mode when using ABM.
  enum LED_MODE
  {
    PWM = 0x00,  ///< PWM control mode.
    ABM1 = 0x01, ///< Auto Breath Mode 1.
    ABM2 = 0x02, ///< Auto Breath Mode 2.
    ABM3 = 0x03  ///< Auto Breath Mode 3.
  };

  /// @brief ABM T1 period time in milliseconds.
  enum ABM_T1
  {
    T1_210MS = 0x00,   ///< 210 milliseconds
    T1_420MS = 0x20,   ///< 420 milliseconds
    T1_840MS = 0x40,   ///< 840 milliseconds
    T1_1680MS = 0x60,  ///< 1680 milliseconds
    T1_3360MS = 0x80,  ///< 3360 milliseconds
    T1_6720MS = 0xA0,  ///< 6720 milliseconds
    T1_13440MS = 0xC0, ///< 13440 milliseconds
    T1_26880MS = 0xE0  ///< 26880 milliseconds
  };

  /// @brief ABM T2 period time in milliseconds.
  enum ABM_T2
  {
    T2_0MS = 0x00,     ///< 0 milliseconds
    T2_210MS = 0x02,   ///< 210 milliseconds
    T2_420MS = 0x04,   ///< 420 milliseconds
    T2_840MS = 0x06,   ///< 840 milliseconds
    T2_1680MS = 0x08,  ///< 1680 milliseconds
    T2_3360MS = 0x0A,  ///< 3360 milliseconds
    T2_6720MS = 0x0C,  ///< 6720 milliseconds
    T2_13440MS = 0x0E, ///< 13440 milliseconds
    T2_26880MS = 0x10  ///< 26880 milliseconds
  };

  /// @brief ABM T3 period time in milliseconds.
  enum ABM_T3
  {
    T3_210MS = 0x00,   ///< 210 milliseconds
    T3_420MS = 0x20,   ///< 420 milliseconds
    T3_840MS = 0x40,   ///< 840 milliseconds
    T3_1680MS = 0x60,  ///< 1680 milliseconds
    T3_3360MS = 0x80,  ///< 3360 milliseconds
    T3_6720MS = 0xA0,  ///< 6720 milliseconds
    T3_13440MS = 0xC0, ///< 13440 milliseconds
    T3_26880MS = 0xE0  ///< 26880 milliseconds
  };

  /// @brief ABM T3 period time in milliseconds.
  enum ABM_T4
  {
    T4_0MS = 0x00,     ///< 0 milliseconds
    T4_210MS = 0x02,   ///< 210 milliseconds
    T4_420MS = 0x04,   ///< 420 milliseconds
    T4_840MS = 0x06,   ///< 840 milliseconds
    T4_1680MS = 0x08,  ///< 1680 milliseconds
    T4_3360MS = 0x0A,  ///< 3360 milliseconds
    T4_6720MS = 0x0C,  ///< 6720 milliseconds
    T4_13440MS = 0x0E, ///< 13440 milliseconds
    T4_26880MS = 0x10, ///< 26880 milliseconds
    T4_53760MS = 0x12, ///< 53760 milliseconds
    T4_107520MS = 0x14 ///< 107520 milliseconds
  };

  /// @brief ABM loop beginning time.
  enum ABM_LOOP_BEGIN
  {
    LOOP_BEGIN_T1 = 0x00, ///< Loop begin from T1.
    LOOP_BEGIN_T2 = 0x10, ///< Loop begin from T2.
    LOOP_BEGIN_T3 = 0x20, ///< Loop begin from T3.
    LOOP_BEGIN_T4 = 0x30  ///< Loop begin from T4.
  };

  /// @brief ABM loop end time.
  enum ABM_LOOP_END
  {
    LOOP_END_T3 = 0x00, ///< Loop end at end of T3.
    LOOP_END_T1 = 0x40  ///< Loop end at end of T1.
  };

  /// @brief ABM function number, also used as register offset.
  enum ABM_NUM
  {
    NUM_1 = PAGEDREGISTER::ABM1CR,
    NUM_2 = PAGEDREGISTER::ABM2CR,
    NUM_3 = PAGEDREGISTER::ABM3CR
  };

  /// @brief Structure for providing ABM configuration options.
  struct ABM_CONFIG
  {
    ABM_T1 T1;             ///< Fade in time.
    ABM_T2 T2;             ///< Hold time.
    ABM_T3 T3;             ///< Fade out time.
    ABM_T4 T4;             ///< Off time.
    ABM_LOOP_BEGIN Tbegin; ///< Position in sequence where loop begins.
    ABM_LOOP_END Tend;     ///< Position in sequence where loop ends.
    uint16_t Times;        ///< Number of times to loop. Set to ABM_LOOP_FOREVER to loop forever.
  };

  /// @brief Function definition for reading and writing the registers.
  typedef uint8_t (*i2c_read_function)(const uint8_t i2c_addr, const uint8_t reg_addr, uint8_t *buffer, const uint8_t count);
  typedef uint8_t (*i2c_write_function)(const uint8_t i2c_addr, const uint8_t reg_addr, const uint8_t *buffer, const uint8_t count);

  /// @brief Driver for interacting with IS31FL3733 chips.
  class IS31FL3733Driver
  {
  private:
    const uint8_t I2C_BASE_ADDR = 0xA0; ///< Base I2C address of the chip.
    uint8_t maxI2CWriteBufferSize;      ///< Maximum number of bytes that can be written by i2c_write_buffer in a single call.

    uint8_t address;                       ///< Address on I2C bus.
    i2c_read_function i2c_read_reg;        ///< Pointer to I2C read register function.
    i2c_write_function i2c_write_reg;      ///< Pointer to the i2C write register function.
    uint8_t leds[SW_LINES * CS_LINES / 8]; ///< State of individual LEDs. Bitmask that can't be read back from IS31FL3733.

    /// @brief Writes every byte in the specified column of the paged register with the specified value.
    /// @param reg The register to write to.
    /// @param sw The column to write to. Origin 0, for example pass 0 to write to cs1.
    /// @param value The value to write.
    void _setColumnPagedRegister(const PAGEDREGISTER reg, const uint8_t cs, uint8_t value);

    /// @brief Writes every byte in the specified paged register with the specified value.
    /// @param reg The register to write to.
    /// @param value The value to write.
    void _setFullPagedRegister(const PAGEDREGISTER reg, const uint8_t value);

    /// @brief Writes every byte in the specified row of the paged register with the specified value.
    /// @param reg The register to write to.
    /// @param sw The row to write to. Origin 0, for example pass 0 to write to sw1.
    /// @param value The value to write.
    void _setRowPagedRegister(const PAGEDREGISTER reg, const uint8_t sw, uint8_t value);

    /// @brief Sets the state of an LED in a single byte of the led array.
    /// @param offset The position in the array for the LED byte.
    /// @param cs The LED's column position. Origin 0, for example pass 0 to control cs1.
    /// @param state The LED_STATE to set the LED to.
    void _setLEDState(const uint8_t offset, const uint8_t cs, const LED_STATE state);

  public:
    /// @brief Construct a new IS31FL3733 object
    /// @param addr1 The ADDR1 pin connection. Must be a value from the ADDR enum.
    /// @param addr2 The ADDR2 pin connection. Must be a value from the ADDR enum.
    /// @param read_function Pointer to the I2C read function. Must implement an i2c_function.
    /// @param write_function Pointer to the I2C write function. Must implement an i2c_function.
    IS31FL3733Driver(const ADDR addr1, const ADDR addr2, const i2c_read_function read_function, const i2c_write_function write_function);

    /// @brief Gets the I2C address for the IS31FL3733.
    /// @return byte The 7-bit I2C address.
    byte GetI2CAddress();

    /// @brief Read from common register.
    /// @param reg The common register to read from.
    uint8_t ReadCommonReg(const COMMONREGISTER reg);

    /// @brief Write to common register.
    /// @param reg The common register to write to.
    /// @param reg_value The value to write.
    void WriteCommonReg(const COMMONREGISTER reg, const uint8_t reg_value);

    /// @brief Select the associated page given a register.
    /// @param reg The register to activate the page for.
    void SelectPageForRegister(const PAGEDREGISTER reg);

    /// @brief Read from paged register.
    /// @param reg The paged register to read from.
    uint8_t ReadPagedReg(const PAGEDREGISTER reg);

    /// @brief Read from paged register.
    /// @param reg The paged register to read from.
    /// @param offset The offset in the paged register to read from.
    uint8_t ReadPagedReg(const PAGEDREGISTER reg, const uint8_t offset);

    /// @brief Write to paged register.
    /// @param reg The paged register to write to.
    /// @param reg_value The value to write.
    void WritePagedReg(const PAGEDREGISTER reg, const uint8_t reg_value);

    /// @brief Write to paged register.
    /// @param reg The paged register to write to.
    /// @param offset The offset in the paged register to write to.
    /// @param reg_value The value to write.
    void WritePagedReg(const PAGEDREGISTER reg, const uint8_t offset, const uint8_t reg_value);

    /// @brief Write array to sequentially allocated paged registers starting from specified address.
    /// @param reg The paged register to write to.
    /// @param values The array of values to write.
    /// @param count The number of values in the array.
    void WritePagedRegs(const PAGEDREGISTER reg, const uint8_t *values, const uint8_t count);

    /// @brief Write array to sequentially allocated paged registers starting from specified address.
    /// @param reg The paged register to write to.
    /// @param reg The offset in the paged register to write to.
    /// @param values The array of values to write.
    /// @param count The number of values in the array.
    void WritePagedRegs(const PAGEDREGISTER reg, const uint8_t offset, const uint8_t *values, const uint8_t count);

    /// @brief Initialize the IS31FL3733 chip.
    void Init();

    /// @brief Set global current control register.
    /// @param gcc The current control value to set.
    void SetGCC(const uint8_t gcc);

    /// @brief Set the SW pull-up register.
    /// @param resistor The value of the pull-up resistor to use.
    void SetSWPUR(const RESISTOR resistor);

    /// @brief Set the CS pull-down register.
    /// @param resistor The value of the pull-down resistor to use.
    void SetCSPDR(const RESISTOR resistor);

    /// @brief Set a single LED state to either ON or OFF.
    /// @param cs The LED's column position. Origin 0, for example pass 0 to control cs1.
    /// @param sw The LED's row position. Origin 0, for example pass 0 to control sw1.
    /// @param state The LED_STATE to set the LED to.
    void SetLEDSingleState(const uint8_t cs, const uint8_t sw, const LED_STATE state);

    /// @brief Sets all LEDs in the specified column to either ON or OFF.
    /// @param cs The column to set. Origin 0, for example pass 0 to control cs1.
    /// @param state The LED_STATE to set the LEDs to.
    void SetLEDColumnState(uint8_t cs, const LED_STATE state);

    /// @brief Sets all LEDs in the specified row to either ON or OFF.
    /// @param sw The row to set. Origin 0, for example pass 0 to control sw1.
    /// @param state The LED_STATE to set the LEDs to.
    void SetLEDRowState(uint8_t sw, const LED_STATE state);

    /// @brief Sets all LEDs to either ON or OFF.
    /// @param cs The column to set. Origin 0, for example pass 0 to control cs1.    /// @param state The LED_STATE to set the LEDs to.
    void SetLEDMatrixState(const LED_STATE state);

    /// @brief Set the PWM duty value for a single LED.
    /// @param cs The LED's column position. Origin 0, for example pass 0 to control cs1.
    /// @param sw The LED's row position. Origin 0, for example pass 0 to control sw1.
    /// @param value The PWM duty cycle to set the LED to.
    void SetLEDSinglePWM(const uint8_t cs, const uint8_t sw, const uint8_t value);

    /// @brief Set the PWM duty value for all LEDs in a column.
    /// @param cs The column to set. Origin 0, for example pass 0 to control cs1.
    /// @param value The PWM duty cycle to set the LEDs to.
    void SetLEDColumnPWM(const uint8_t cs, const uint8_t value);

    /// @brief Set the PWM duty value for all LEDs in a row.
    /// @param sw The row to set. Origin 0, for example pass 0 to control sw1.
    /// @param value The PWM duty cycle to set the LEDs to.
    void SetLEDRowPWM(const uint8_t sw, const uint8_t value);

    /// @brief Set the PWM duty value for all LEDs.
    /// @param value The PWM duty cycle to set the LEDs to.
    void SetLEDMatrixPWM(const uint8_t value);

    /// @brief Set the LED state for all LED's from buffer.
    /// @param states An array of LED states for all 192 LEDs.
    void SetState(const LED_STATE *states);

    /// @brief Get the status of an LED.
    /// @param CS The LED's column position.
    /// @param SW THe LED's row position.
    /// @returns The status of the LED.
    LED_STATUS GetLEDStatus(const uint8_t cs, const uint8_t sw);

    /// @brief Set the LED PWN values for all LED's from buffer.
    /// @param states An array of PWM values for all 192 LEDs.
    void SetPWM(const uint8_t *values);

    /// @brief Sets the LED operating mode for a single LED.
    /// @param cs The LED's column position. Origin 0, for example pass 0 to control cs1.
    /// @param sw The LED's row position. Origin 0, for example pass 0 to control sw1.
    /// @param value The LED_MODE to set.
    void SetLEDSingleMode(uint8_t cs, uint8_t sw, const LED_MODE mode);

    /// @brief Set the LED operating mode for all LEDs in a row.
    /// @param sw The row to set. Origin 0, for example pass 0 to control sw1.
    /// @param value The LED_MODE to set.
    void SetLEDRowMode(const uint8_t sw, const LED_MODE mode);

    /// @brief Set the LED operating mode for all LEDs in a column.
    /// @param sw The column to set. Origin 0, for example pass 0 to control cs1.
    /// @param value The LED_MODE to set.
    void SetLEDColumnMode(const uint8_t cs, const LED_MODE mode);

    /// @brief Set the LED operating mode for all LEDs in the matrix.
    /// @param value The LED_MODE to set.
    void SetLEDMatrixMode(const LED_MODE mode);

    /// @brief Configures the ABM mode options.
    /// @param n The ABM to configure.
    /// @param config The configuration to set.
    void ConfigABM(const ABM_NUM n, const ABM_CONFIG *config);

    /// @brief Starts ABM operation.
    void StartABM();
  };
}
