# VL53L7CX
Arduino library to support the VL53L7CX Time-of-Flight 8x8 multizone ranging sensor with wide field view.

## API

This sensor uses I2C to communicate. And I2C instance is required to access to the sensor.
The APIs provide simple distance measure and multizone detection in both polling and interrupt modes.

## Examples

The examples contained in this library are based on VL53L7CX-SATEL sensor board. You need to connect the VL53L7CX-SATEL sensor board directly to the Nucleo board with wires as explained below:
- pin 1 (GND) of the VL53L7CX satellite connected to GND of the Nucleo board
- pin 2 (IOVDD) of the VL53L7CX satellite connected to 3V3 pin of the Nucleo board
- pin 3 (AVDD) of the VL53L7CX satellite connected to 5V pin of the Nucleo board
- pin 4 (PWREN) of the VL53L7CX satellite connected to pin A5 of the Nucleo board
- pin 5 (LPn) of the VL53L7CX satellite connected to pin A3 of the Nucleo board
- pin 6 (SCL) of the VL53L7CX satellite connected to pin D15 (SCL) of the Nucleo board
- pin 7 (SDA) of the VL53L7CX satellite connected to pin D14 (SDA) of the Nucleo board
- pin 8 (I2C_RST) of the VL53L7CX satellite connected to pin A1 of the Nucleo board
- pin 9 (INT) of the VL53L7CX satellite connected to pin A2 of the Nucleo board 

There are 2 examples with the VL53L7CX library:

* VL53L7CX_HelloWorld: This example code is to show how to get multizone detection and proximity values of the VL53L7CX satellite sensor in polling mode.

* VL53L7CX_ThresholdsDetection: This example code is to show how to configure the thresholds detection of the VL53L7CX satellite sensor.

## Documentation

You can find the source files at
https://github.com/stm32duino/VL53L7CX

The VL53L7CX datasheet is available at
https://www.st.com/en/imaging-and-photonics-solutions/VL53L7CX.html
