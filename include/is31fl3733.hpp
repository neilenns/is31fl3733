/** ISSI IS31FL3733 Pulse Width Mode (PWM) control.
  */
#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include <stdint.h>

namespace IS31FL3733
{
  /**
   * @brief Number of CS lines on the chip.
   * 
   */
  const uint8_t CS_LINES = 16;

  /**
 * @brief Number of SW lines on the chip.
 * 
 */
  const uint8_t SW_LINES = 12;

  /** IS31FL3733 base address on I2C bus.
  */
  const uint8_t I2C_BASE_ADDR = 0xA0;

/** IS31FL3733 common registers.
  */
#define IS31FL3733_PSR (0xFD)  ///< Page select register. Write only.
#define IS31FL3733_PSWL (0xFE) ///< Page select register write lock. Read/Write.
#define IS31FL3733_IMR (0xF0)  ///< Interrupt mask register. Write only.
#define IS31FL3733_ISR (0xF1)  ///< Interrupt status register. Read only.

/** Registers in Page 0.
  */
#define IS31FL3733_LEDONOFF (0x0000) /// ON or OFF state control for each LED. Write only.
#define IS31FL3733_LEDOPEN (0x0018)  /// Open state for each LED. Read only.
#define IS31FL3733_LEDSHORT (0x0030) /// Short state for each LED. Read only.

/** Registers in Page 1.
  */
#define IS31FL3733_LEDPWM (0x0100) /// PWM duty for each LED. Write only.

/** Registers in Page 2.
  */
#define IS31FL3733_LEDABM (0x0200) /// Auto breath mode for each LED. Write only.

/** Registers in Page 3.
  */
#define IS31FL3733_CR (0x0300)    /// Configuration Register. Write only.
#define IS31FL3733_GCC (0x0301)   /// Global Current Control register. Write only.
#define IS31FL3733_ABM1 (0x0302)  /// Auto breath control register for ABM-1. Write only.
#define IS31FL3733_ABM2 (0x0306)  /// Auto breath control register for ABM-2. Write only.
#define IS31FL3733_ABM3 (0x030A)  /// Auto breath control register for ABM-3. Write only.
#define IS31FL3733_TUR (0x030E)   /// Time update register. Write only.
#define IS31FL3733_SWPUR (0x030F) /// SWy Pull-Up Resistor selection register. Write only.
#define IS31FL3733_CSPDR (0x0310) /// CSx Pull-Down Resistor selection register. Write only.
#define IS31FL3733_RESET (0x0311) /// Reset register. Read only.

/// Get register page.
#define IS31FL3733_GET_PAGE(reg_addr) (uint8_t)((reg_addr) >> 8)
/// Get register 8-bit address.
#define IS31FL3733_GET_ADDR(reg_addr) (uint8_t)(reg_addr)

/// PSWL register bits.
#define IS31FL3733_PSWL_DISABLE (0x00) /// Disable write to Page Select register.
#define IS31FL3733_PSWL_ENABLE (0xC5)  /// Enable write to Page select register.

/// IMR register bits.
#define IS31FL3733_IMR_IAC (0x08) /// Auto Clear Interrupt bit.
#define IS31FL3733_IMR_IAB (0x04) /// Auto Breath Interrupt bit.
#define IS31FL3733_IMR_IS (0x02)  /// Dot Short Interrupt bit.
#define IS31FL3733_IMR_IO (0x01)  /// Dot Open Interrupt bit.

/// ISR register bits.
#define IS31FL3733_ISR_ABM3 (0x10) /// Auto Breath Mode 3 Finish Bit.
#define IS31FL3733_ISR_ABM2 (0x08) /// Auto Breath Mode 2 Finish Bit.
#define IS31FL3733_ISR_ABM1 (0x04) /// Auto Breath Mode 1 Finish Bit.
#define IS31FL3733_ISR_SB (0x02)   /// Short Bit.
#define IS31FL3733_ISR_OB (0x01)   /// Open Bit.

/// CR register bits.
#define IS31FL3733_CR_SYNC_MASTER (0x40) /// Configure as clock master device.
#define IS31FL3733_CR_SYNC_SLAVE (0x80)  /// Configure as clock slave device.
#define IS31FL3733_CR_OSD (0x04)         /// Open/Short detection enable bit.
#define IS31FL3733_CR_BEN (0x02)         /// Auto breath mode enable bit.
#define IS31FL3733_CR_SSD (0x01)         /// Software shutdown bit.

  /**
 * @brief Defines the valid connection points for ADDR1 and ADDR2 pins.
 * 
 */
  enum ADDR
  {
    /**
   * @brief Pin connected to GND.
   * 
   */
    GND = 0x00,
    /**
   * @brief Pin connected to SCL.
   * 
   */
    SCL = 0x01,
    /**
   * @brief Pin connected to SDA.
   * 
   */
    SDA = 0x02,
    /**
   * @brief Pin connected to VCC.
   * 
   */
    VCC = 0x03
  };

  /// LED state enumeration.
  enum LED_STATE
  {
    OFF = 0x00, ///< LED is off.
    ON = 0x01   ///< LED is on.
  };

  /// LED status enumeration.
  enum LED_STATUS
  {
    NORMAL = 0x00, ///< Normal LED status.
    OPEN = 0x01,   ///< LED is open.
    SHORT = 0x02,  ///< LED is short.
    UNKNOWN = 0x03 ///< Unknown LED status.
  };

  /// Pull-Up or Pull-Down resistor value.
  enum RESISTOR
  {
    RESISTOR_OFF = 0x00, ///< No resistor.
    RESISTOR_500 = 0x01, ///< 0.5 kOhm pull-up resistor.
    RESISTOR_1K = 0x02,  ///< 1.0 kOhm pull-up resistor.
    RESISTOR_2K = 0x03,  ///< 2.0 kOhm pull-up resistor.
    RESISTOR_4K = 0x04,  ///< 4.0 kOhm pull-up resistor.
    RESISTOR_8K = 0x05,  ///< 8.0 kOhm pull-up resistor.
    RESISTOR_16K = 0x06, ///< 16 kOhm pull-up resistor.
    RESISTOR_32K = 0x07  ///< 32 kOhm pull-up resistor.
  };

// ****************************************************************
// Start ABM support
#ifdef ENABLE_ABM

  /**
 * @brief Maximum number of ABM loop times.
 * 
 */
  const int ABM_LOOP_TIMES_MAX = 0x0FFF;

  /**
   * @brief Loop ABM forever.
   * 
   */
  const int ABM_LOOP_FOREVER = 0x0000;

  /// LED mode enumeration.
  enum LED_MODE
  {
    PWM = 0x00,  ///< PWM control mode.
    ABM1 = 0x01, ///< Auto Breath Mode 1.
    ABM2 = 0x02, ///< Auto Breath Mode 2.
    ABM3 = 0x03  ///< Auto Breath Mode 3.
  };

  /// ABM T1 period time, ms.
  enum ABM_T1
  {
    T1_210MS = 0x00,
    T1_420MS = 0x20,
    T1_840MS = 0x40,
    T1_1680MS = 0x60,
    T1_3360MS = 0x80,
    T1_6720MS = 0xA0,
    T1_13440MS = 0xC0,
    T1_26880MS = 0xE0
  };

  /// ABM T2 period time, ms.
  enum ABM_T2
  {
    T2_0MS = 0x00,
    T2_210MS = 0x02,
    T2_420MS = 0x04,
    T2_840MS = 0x06,
    T2_1680MS = 0x08,
    T2_3360MS = 0x0A,
    T2_6720MS = 0x0C,
    T2_13440MS = 0x0E,
    T2_26880MS = 0x10
  };

  /// ABM T3 period time, ms.
  enum ABM_T3
  {
    T3_210MS = 0x00,
    T3_420MS = 0x20,
    T3_840MS = 0x40,
    T3_1680MS = 0x60,
    T3_3360MS = 0x80,
    T3_6720MS = 0xA0,
    T3_13440MS = 0xC0,
    T3_26880MS = 0xE0
  };

  /// ABM T4 period time, ms.
  enum ABM_T4
  {
    T4_0MS = 0x00,
    T4_210MS = 0x02,
    T4_420MS = 0x04,
    T4_840MS = 0x06,
    T4_1680MS = 0x08,
    T4_3360MS = 0x0A,
    T4_6720MS = 0x0C,
    T4_13440MS = 0x0E,
    T4_26880MS = 0x10,
    T4_53760MS = 0x12,
    T4_107520MS = 0x14
  };

  /// ABM loop beginning time.
  enum ABM_LOOP_BEGIN
  {
    LOOP_BEGIN_T1 = 0x00, ///< Loop begin from T1.
    LOOP_BEGIN_T2 = 0x10, ///< Loop begin from T2.
    LOOP_BEGIN_T3 = 0x20, ///< Loop begin from T3.
    LOOP_BEGIN_T4 = 0x30  ///< Loop begin from T4.
  };

  /// ABM loop end time.
  enum ABM_LOOP_END
  {
    LOOP_END_T3 = 0x00, ///< Loop end at end of T3.
    LOOP_END_T1 = 0x40  ///< Loop end at end of T1.
  };

  /// ABM function number (also used as register offset).
  enum ABM_NUM
  {
    NUM_1 = IS31FL3733_ABM1,
    NUM_2 = IS31FL3733_ABM2,
    NUM_3 = IS31FL3733_ABM3
  };

  /** Auto Breath Mode (ABM) configuration structure.
  *      +----+              +
  *     /      \            / 
  *    /        \          /  
  *   /          \        /   
  *  /            \      /    
  * +              +----+     
  * | T1 | T2 | T3 | T4 | T1 |
  *
  */
  struct ABM
  {
    /// T1 time.
    ABM_T1 T1;
    /// T2 time.
    ABM_T2 T2;
    /// T3 time.
    ABM_T3 T3;
    /// T4 time.
    ABM_T4 T4;
    /// Loop beginning time.
    ABM_LOOP_BEGIN Tbegin;
    /// Loop end time.
    ABM_LOOP_END Tend;
    /// Total loop times.
    uint16_t Times;
  };

#endif
  // End ABW support
  // ****************************************************************

  // Function definitions for reading and writing the registers.
  typedef uint8_t (*i2c_function)(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t count);

  /** IS31FL3733 class.
  */
  class IS31FL3733Driver
  {
  private:
    /// Address on I2C bus.
    uint8_t address;

    /// Pointer to I2C read register function.
    i2c_function i2c_read_reg;

    // Pointer to the i2C write register function.
    i2c_function i2c_write_reg;

    /// State of individual LED's. Bitmask, that can't be read back from IS31FL3733.
    uint8_t leds[SW_LINES * CS_LINES / 8];

  public:
    /**
 * @brief Construct a new IS31FL3733 object
 * 
 * @param addr1 The ADDR1 pin connection. One of: ADDR_GND, ADDR_PWR
 * @param addr2 
 * @param read_function 
 * @param write_function 
 */
    IS31FL3733Driver(ADDR addr1, ADDR addr2, i2c_function read_function, i2c_function write_function);

    /**
   * @brief Gets the I2C address for the IS31FL3733. 
   * 
   * @return byte The 7-bit I2C address.
   */
    byte GetI2CAddress();

    /// Read from common register.
    uint8_t ReadCommonReg(uint8_t reg_addr);
    /// Write to common register.
    void WriteCommonReg(uint8_t reg_addr, uint8_t reg_value);
    /// Select active page.
    void SelectPage(uint8_t page);
    /// Read from paged register.
    uint8_t ReadPagedReg(uint16_t reg_addr);
    /// Write to paged register.
    void WritePagedReg(uint16_t reg_addr, uint8_t reg_value);
    /// Write array to sequentially allocated paged registers starting from specified address.
    void WritePagedRegs(uint16_t reg_addr, uint8_t *values, uint8_t count);
    /// Initialize IS31FL3733 for PWM operation.
    void Init();
    /// Set global current control register.
    void SetGCC(uint8_t gcc);
    /// Set SW Pull-Up register.
    void SetSWPUR(RESISTOR resistor);
    /// Set CS Pull-Down register.
    void SetCSPDR(RESISTOR resistor);
    /// Set LED state: ON/OFF. Could be set ALL / CS / SW.
    void SetLEDState(uint8_t cs, uint8_t sw, LED_STATE state);
    /// Set LED PWM duty value. Could be set ALL / CS / SW.
    void SetLEDPWM(uint8_t cs, uint8_t sw, uint8_t value);
    /// Get status of LED.
    LED_STATUS GetLEDStatus(uint8_t cs, uint8_t sw);
    /// Set LED state for all LED's from buffer.
    void SetState(uint8_t *states);
    /// SET LED PWM duty value for all LED's from buffer.
    void SetPWM(uint8_t *values);

// ABM functions
#ifdef ENABLE_ABM
    /// Set LED operating mode: PWM/ABM1,2,3. Could be set ALL / CS / SW.
    void SetLEDMode(uint8_t cs, uint8_t sw, LED_MODE mode);
    /// Configure ABM Mode.
    void ConfigABM(ABM_NUM n, ABM *config);
    /// Start ABM operation.
    void StartABM();
#endif
  };
}
