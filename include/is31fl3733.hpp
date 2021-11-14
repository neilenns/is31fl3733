/** ISSI IS31FL3733 Pulse Width Mode (PWM) control.
  */
#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include <stdint.h>

/** Number of CS lines.
  */
#define IS31FL3733_CS (16)

/** Number of SW lines.
  */
#define IS31FL3733_SW (12)

/** IS31FL3733 base address on I2C bus.
  */
#define IS31FL3733_I2C_BASE_ADDR (0x50)

/** IS31FL3733 real address on I2C bus, see Table 1 on page 9 in datasheet.
    Example: IS31FL3733_I2C_ADDR(ADDR_SDA, ADDR_VCC) is 0xB6 address on I2C bus.
  */
#define IS31FL3733_I2C_ADDR(ADDR2, ADDR1) ((IS31FL3733_I2C_BASE_ADDR) | ((ADDR2) << 3) | ((ADDR1) << 1))

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
typedef enum
{
  /**
   * @brief Pin connected to GND.
   * 
   */
  IS31FL3733_ADDR_GND = 0x00,
  /**
   * @brief Pin connected to SCL.
   * 
   */
  IS31FL3733_ADDR_SCL = 0x01,
  /**
   * @brief Pin connected to SDA.
   * 
   */
  IS31FL3733_ADDR_SDA = 0x02,
  /**
   * @brief Pin connected to VCC.
   * 
   */
  IS31FL3733_ADDR_VCC = 0x03
} IS31FL3733_ADDR;

/// LED state enumeration.
typedef enum
{
  IS31FL3733_LED_STATE_OFF = 0x00, ///< LED is off.
  IS31FL3733_LED_STATE_ON = 0x01   ///< LED is on.
} IS31FL3733_LED_STATE;

/// LED status enumeration.
typedef enum
{
  IS31FL3733_LED_STATUS_NORMAL = 0x00, ///< Normal LED status.
  IS31FL3733_LED_STATUS_OPEN = 0x01,   ///< LED is open.
  IS31FL3733_LED_STATUS_SHORT = 0x02,  ///< LED is short.
  IS31FL3733_LED_STATUS_UNKNOWN = 0x03 ///< Unknown LED status.
} IS31FL3733_LED_STATUS;

/// Pull-Up or Pull-Down resistor value.
typedef enum
{
  IS31FL3733_RESISTOR_OFF = 0x00, ///< No resistor.
  IS31FL3733_RESISTOR_500 = 0x01, ///< 0.5 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_1K = 0x02,  ///< 1.0 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_2K = 0x03,  ///< 2.0 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_4K = 0x04,  ///< 4.0 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_8K = 0x05,  ///< 8.0 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_16K = 0x06, ///< 16 kOhm pull-up resistor.
  IS31FL3733_RESISTOR_32K = 0x07  ///< 32 kOhm pull-up resistor.
} IS31FL3733_RESISTOR;

// Function definitions for reading and writing the registers.
typedef uint8_t (*i2c_function)(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t count);

/** IS31FL3733 class.
  */
class IS31FL3733
{
private:
  /// Address on I2C bus.
  uint8_t address;

  /// Pointer to I2C read register function.
  i2c_function i2c_read_reg;

  // Pointer to the i2C write register function.
  i2c_function i2c_write_reg;

  /// State of individual LED's. Bitmask, that can't be read back from IS31FL3733.
  uint8_t leds[IS31FL3733_SW * IS31FL3733_CS / 8];

public:
  /**
 * @brief Construct a new IS31FL3733 object
 * 
 * @param addr1 The ADDR1 pin connection. One of: ADDR_GND, ADDR_PWR
 * @param addr2 
 * @param read_function 
 * @param write_function 
 */
  IS31FL3733(IS31FL3733_ADDR addr1, IS31FL3733_ADDR addr2, i2c_function read_function, i2c_function write_function);

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
  void SetSWPUR(IS31FL3733_RESISTOR resistor);
  /// Set CS Pull-Down register.
  void SetCSPDR(IS31FL3733_RESISTOR resistor);
  /// Set LED state: ON/OFF. Could be set ALL / CS / SW.
  void SetLEDState(uint8_t cs, uint8_t sw, IS31FL3733_LED_STATE state);
  /// Set LED PWM duty value. Could be set ALL / CS / SW.
  void SetLEDPWM(uint8_t cs, uint8_t sw, uint8_t value);
  /// Get status of LED.
  IS31FL3733_LED_STATUS GetLEDStatus(uint8_t cs, uint8_t sw);
  /// Set LED state for all LED's from buffer.
  void SetState(uint8_t *states);
  /// SET LED PWM duty value for all LED's from buffer.
  void SetPWM(uint8_t *values);
};
