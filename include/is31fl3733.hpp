/** ISSI IS31FL3733 Pulse Width Mode (PWM) control.
  */
#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include <stdint.h>

namespace IS31FL3733
{
  const uint8_t CS_LINES = 16; ///< Number of CS lines on the chip.
  const uint8_t SW_LINES = 12; ///< Number of SW lines on the chip.

  /// @brief Addresses for all the registers.
  enum REGISTERS
  {
    PSR = 0x00FD,      ///< Common: page select register. Write only.
    PSWL = 0x00FE,     ///< Common: page select register write lock. Read/write.
    IMR = 0x00F0,      ///< Common: interrupt mask register. Write only.
    ISR = 0x00F1,      ///< Common: interrupt status register. Read only.
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

// ****************************************************************
// Start ABM support
#ifdef ENABLE_ABM
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
    NUM_1 = REGISTERS::ABM1CR,
    NUM_2 = REGISTERS::ABM2CR,
    NUM_3 = REGISTERS::ABM3CR
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

#endif
  // End ABW support
  // ****************************************************************

  /// @brief Function definition for reading and writing the registers.
  typedef uint8_t (*i2c_function)(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t count);

  /// @brief Driver for interacting with IS31FL3733 chips.
  class IS31FL3733Driver
  {
  private:
    const uint8_t I2C_BASE_ADDR = 0xA0; ///< Base I2C address of the chip.

    uint8_t address;                       ///< Address on I2C bus.
    i2c_function i2c_read_reg;             ///< Pointer to I2C read register function.
    i2c_function i2c_write_reg;            ///< Pointer to the i2C write register function.
    uint8_t leds[SW_LINES * CS_LINES / 8]; ///< State of individual LEDs. Bitmask that can't be read back from IS31FL3733.

  public:
    /// @brief Construct a new IS31FL3733 object
    /// @param addr1 The ADDR1 pin connection. Must be a value from the ADDR enum.
    /// @param addr2 The ADDR2 pin connection. Must be a value from the ADDR enum.
    /// @param read_function Pointer to the I2C read function. Must implement an i2c_function.
    /// @param write_function Pointer to the I2C write function. Must implement an i2c_function.
    IS31FL3733Driver(ADDR addr1, ADDR addr2, i2c_function read_function, i2c_function write_function);

    /// @brief Gets the I2C address for the IS31FL3733.
    /// @return byte The 7-bit I2C address.
    byte GetI2CAddress();

    /// @brief Read from common register.
    /// @param reg_addr The common register to read from.
    uint8_t ReadCommonReg(uint8_t reg_addr);

    /// @brief Write to common register.
    void WriteCommonReg(uint8_t reg_addr, uint8_t reg_value);

    /// @brief Select the associated page given a register.
    /// @param reg_addr The register to activate the page for.
    void SelectPageForRegister(uint16_t reg_addr);

    /// @brief Read from paged register.
    /// @param reg_addr The paged register to read from.
    uint8_t ReadPagedReg(uint16_t reg_addr);

    /// @brief Write to paged register.
    /// @param reg_addr The paged register to write to.
    /// @param reg_value The value to write.
    void WritePagedReg(uint16_t reg_addr, uint8_t reg_value);

    /// @brief Write array to sequentially allocated paged registers starting from specified address.
    /// @param reg_adder The paged register to write to.
    /// @param values The array of values to write.
    /// @param count The number of values in the array.
    void WritePagedRegs(uint16_t reg_addr, uint8_t *values, uint8_t count);

    /// @brief Initialize the IS31FL3733 chip.
    void Init();

    /// @brief Set global current control register.
    /// @param gcc The current control value to set.
    void SetGCC(uint8_t gcc);

    /// @brief Set the SW pull-up register.
    /// @param resistor The value of the pull-up resistor to use.
    void SetSWPUR(RESISTOR resistor);

    /// @brief Set the CS pull-down register.
    /// @param resistor The value of the pull-down resistor to use.
    void SetCSPDR(RESISTOR resistor);

    /// @brief Set LED state to either ON or OFF.
    /// @param cs The LED's column position. Use CS_COUNT to set all LEDs in the column.
    /// @param sw The LED's row position. Use SW_COUNT to set all LEDs in the row.
    /// @param state The LED_STATE to set the LED to.
    void SetLEDState(uint8_t cs, uint8_t sw, LED_STATE state);

    /// @brief Set the LED PWM duty value.
    /// @param cs The LED's column position. Use CS_COUNT to set all LEDs in the column.
    /// @param sw The LED's row position. Use SW_COUNT to set all LEDs in the row.
    /// @param value The PWM duty cycle to set the LED to.
    void SetLEDPWM(uint8_t cs, uint8_t sw, uint8_t value);

    /// @brief Get the status of an LED.
    /// @param CS The LED's column position.
    /// @param SW THe LED's row position.
    /// @returns The status of the LED.
    LED_STATUS GetLEDStatus(uint8_t cs, uint8_t sw);

    /// @brief Set the LED state for all LED's from buffer.
    /// @param states An array of LED states for all 192 LEDs.
    void SetState(uint8_t *states);

    /// @brief Set the LED PWN values for all LED's from buffer.
    /// @param states An array of PWM values for all 192 LEDs.
    void SetPWM(uint8_t *values);

// ABM functions
#ifdef ENABLE_ABM
    /// @brief Sets the LED operating mode for an LED.
    /// @param cs The LED's column position. Use CS_COUNT to set all LEDs in the column.
    /// @param sw The LED's row position. Use SW_COUNT to set all LEDs in the row.
    /// @param value The LED_MODE to set.
    void SetLEDMode(uint8_t cs, uint8_t sw, LED_MODE mode);

    /// @brief Configures the ABM mode options.
    /// @param n The ABM to configure.
    /// @param config The configuration to set.
    void ConfigABM(ABM_NUM n, ABM_CONFIG *config);

    /// @brief Starts ABM operation.
    void StartABM();
#endif
  };
}
