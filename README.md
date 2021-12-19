# IS31FL3733 C++ library

This library originally forked from [the C library written by kkostyan](https://github.com/kkostyan/is31fl3733).
It is functionally equivalent to the C library but updated to use C++, including namespaces, enums, and constants
instead of #defines. Several methods have been added and renamed for clarity and the I2C communication is optimized
to be as efficient as possible.

Doxygen comments are provided for all methods. It also combines the ABM support into the core
library via a compile time flag.

## Using with PlatformIO

This library is registered with PlatformIO. Include it in your project via the PlatformIO library
manager.

## Using with other toolchains

Include `is31fl3733.hpp` and `is31fl3733.cpp` in your project.

## Common steps

The following code samples illustrate common tasks.

### Creating a new instance of the driver

To work with library in any of this modes you need to declare an instance of IS31FL3733Driver,
providing the connections for the `ADDR1` and `ADDR2` pins and [functions for reading and writing
I2C data](#Implementing_the_read_and_write_register_functions):

```C++
using namespace IS31FL3733
IS31FL3733Driver driver(ADDR::GND, ADDR::GND, &i2c_read_reg, &i2c_write_reg);
```

### Implementing the read and write register functions

The `i2c_read_reg` and `i2c_write_reg` functions must be implmeneted for each platform
and take care of reading and writing data on the I2C bus.

Example implementations for Arduino:

```C++
uint8_t i2c_write_reg(const uint8_t i2c_addr, const uint8_t reg_addr, const uint8_t *buffer, const uint8_t count)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  byte bytesWritten = Wire.write(buffer, count);
  Wire.endTransmission();

  return bytesWritten;
}

uint8_t i2c_read_reg(const uint8_t i2c_addr, const uint8_t reg_addr, uint8_t *buffer, const uint8_t count)
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
```

### Initializing the device

The first step in using the IS31FL3733 is to initialize the chip and set the global current
control. This is done with the `Init()` and `SetGCC()` methods:

```C++
// Reset the chip to a clean startup state.
driver.Init();
// Set global current control to half the max value.
driver.SetGCC(127);
```

### Setting PWM values

PWN values are set using the `SetLEDSinglePWM()` method and LEDs are turned on and off
using the `SetLEDSingleState()` method.

```C++
// Set PWM value for LED at {1, 2} to maximum brightness.
driver.SetLEDSinglePWM(1, 2, 255);
// Turn on LED at position {1, 2}.
driver.SetLEDSingleState(1, 2, LEDSTATE::On);
```

In addition to setting a single LED you can also control all LEDs in a row or column
or all LEDs in the matrix:

```C++
// Set PWM values for all LEDs in the first column to 15.
driver.SetLEDColumnPWM(1, 15);
// Turn on all LEDs in the firs column.
driver.SetLEDColumnState(1, LED_STATE::ON);

// Set PWM values for all LEDs in the seventh row to 55.
driver.SetLEDRowPWM(7, 55);
// Turn on all LEDs in the seventh row.
driver.SetLEDRowState(7, LED_STATE::ON);

// Set PWM values for all LEDs in the matrix to 146.
driver.SetLEDMatrixPWM(146);
// Turn on all LEDs in the matrix
driver.SetLEDMatrixState(LED_STATE::ON);
```

Also you can update all LEDs state and brightness from an array of values, e.g. draw a heart figure:

```C++
// 16x12 heart figure.
constexpr uint8_t heart[] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,
    0x00,0x00,0x00,0xff,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0xff,0x00,0x00,0x00,
    0x00,0x00,0x00,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0x00,0x00,
    0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

// Set LED brightness for all LEDs from an array.
driver.SetPWM(heart);
// Turn on LEDs with non-zero brightness.
driver.SetState(heart);
```

### Initializing ABM mode, setting parameters, and starting ABM

To set up ABM mode first turn on the desired LEDs, then set the LED mode to one of the available
types of ABM:

```C++
// Turn on LEDs for heart figure.
driver.SetState(heart);
// Configure all matrix LEDs to work in ABM1 mode.
driver.SetLEDMatrixMode(LED_MODE::ABM1);
```

Then configure the ABM parameters and start ABM mode:

```C++
ABM_CONFIG ABM1;

ABM1.T1 = ABM_T1::T1_840MS;
ABM1.T2 = ABM_T2::T2_840MS;
ABM1.T3 = ABM_T3::T3_840MS;
ABM1.T4 = ABM_T4::T4_840MS;
ABM1.Tbegin = ABM_LOOP_BEGIN::LOOP_BEGIN_T4;
ABM1.Tend = ABM_LOOP_END::LOOP_END_T3;
ABM1.Times = ABM_LOOP_FOREVER;

// Write ABM structure parameters.
driver.ConfigABM(ABM_NUM::NUM_1, &ABM1);
// Start ABM mode operation.
driver.StartABM();
```
