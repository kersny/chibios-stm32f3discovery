STM32F3Discovery ChibiOS Demo with Gyro and USB-CDC
===================================================

An amalgamation of ChibiOS demo projects designed to run on the STM32F3Discovery board. Demonstrates reading from the onboard L3GD20 gyroscope over SPI and output through the ChibiOS USB-CDC driver. Once ChibiOS supports I2C on the STM32F3, the goal is to read from the LSM303DLHC and implement IMU/AHRS algorithms.


### Setup

Requires an arm-none-eabi- set of gcc tools, I use [CodeSourcery Lite](http://www.mentor.com/embedded-software/codesourcery). Update the CHIBIOS variable in the Makefile to the location of the (Head, from SVN or Git) ChibiOS repository.


### Build

    make


### Program

I use [stlink](https://github.com/texane/stlink), with the board plugged into the USB ST-LINK port.

    {sudo} st-flash write build/stm32f3discovery-demo.bin 0x8000000

### Use

Connect to the USB USER port.

    {sudo} cat /dev/ttyACM0

You should see 9 floating point values corresponding to the x, y, and z values from the gyroscope, accelerometer, and magnetometer.


### Debug

Again, with [stlink](https://github.com/texane/stlink).

In one terminal:

    {sudo} st-util
    
And another:

    arm-non-eabi-gdb build/stm32f3discovery-demo.elf
    
And then within GDB:

    > target extended-remote :4242
    ...
    > load
    ...
    
And you can debug with GDB as you would expect.
