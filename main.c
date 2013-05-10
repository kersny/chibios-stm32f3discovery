/*
   ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
   2011,2012 Giovanni Di Sirio.

   This file is part of ChibiOS/RT.

   ChibiOS/RT is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   ChibiOS/RT is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
   */

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "spi.h"
#include "i2c.h"
#include "usbcfg.h"
#include "chprintf.h"

#define usb_lld_connect_bus(usbp)
#define usb_lld_disconnect_bus(usbp)

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

static float mdps_per_digit = 8.75;


static const SPIConfig spi1cfg = {
  NULL,
  /* HW dependent part.*/
  GPIOE,
  GPIOE_SPI1_CS,
  SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA,
  0
};

static const I2CConfig i2cconfig = {
  0x00902025, //voodoo magic
  0,
  0
};

static uint8_t readByteSPI(uint8_t reg)
{
	char txbuf[2] = {0x80 | reg, 0xFF};
	char rxbuf[2];
	spiSelect(&SPID1);
	spiExchange(&SPID1, 2, txbuf, rxbuf);
	spiUnselect(&SPID1);
	return rxbuf[1];
}
static uint8_t writeByteSPI(uint8_t reg, uint8_t val)
{
	char txbuf[2] = {reg, val};
	char rxbuf[2];
	spiSelect(&SPID1);
	spiExchange(&SPID1, 2, txbuf, rxbuf);
	spiUnselect(&SPID1);
	return rxbuf[1];
}

static uint8_t readByteI2C(uint8_t addr, uint8_t reg)
{
    uint8_t data;
    i2cAcquireBus(&I2CD1);
    (void)i2cMasterTransmitTimeout(&I2CD1, addr, &reg, 1, &data, 1, TIME_INFINITE);
    i2cReleaseBus(&I2CD1);
    return data;
}
static void writeByteI2C(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t cmd[] = {reg, val};
    i2cAcquireBus(&I2CD1);
    (void)i2cMasterTransmitTimeout(&I2CD1, addr, cmd, 2, NULL, 0, TIME_INFINITE);
    i2cReleaseBus(&I2CD1);
}

static void initGyro(void)
{
    /* see the L3GD20 Datasheet */
    writeByteSPI(0x20, 0x0F);
}
static void initAccel(void)
{
    // Highest speed, enable all axes
    writeByteI2C(0x19,0x20, 0x97);
}
static void initMag(void)
{
    // Highest speed, enable all axes
    writeByteI2C(0x1E,0x00, 0x1C);
    writeByteI2C(0x1E, 0x02, 0x00);
}
static void readGyro(float* data)
{
    /* read from L3GD20 registers and assemble data */
    uint8_t x_low = readByteSPI(0x28);
    uint8_t x_high = readByteSPI(0x29);
    uint8_t y_low = readByteSPI(0x2a);
    uint8_t y_high = readByteSPI(0x2b);
    uint8_t z_low = readByteSPI(0x2c);
    uint8_t z_high = readByteSPI(0x2d);
    int16_t val_x = (x_high << 8) | x_low;
    int16_t val_y = (y_high << 8) | y_low;
    int16_t val_z = (z_high << 8) | z_low;
    data[0] = (((float)val_x) * mdps_per_digit)/1000.0;
    data[1] = (((float)val_y) * mdps_per_digit)/1000.0;
    data[2] = (((float)val_z) * mdps_per_digit)/1000.0;
}
static void readAccel(float* data)
{
    uint8_t x_low = readByteI2C(0x19, 0x28);
    uint8_t x_high = readByteI2C(0x19, 0x29);
    uint8_t y_low = readByteI2C(0x19, 0x2a);
    uint8_t y_high = readByteI2C(0x19, 0x2b);
    uint8_t z_low = readByteI2C(0x19, 0x2c);
    uint8_t z_high = readByteI2C(0x19, 0x2d);
    int16_t val_x = (x_high << 8) | x_low;
    int16_t val_y = (y_high << 8) | y_low;
    int16_t val_z = (z_high << 8) | z_low;
    // Accel scale is +- 2.0g
    data[0] = ((float)val_x)*(4.0/(65535.0))*9.81;
    data[1] = ((float)val_y)*(4.0/(65535.0))*9.81;
    data[2] = ((float)val_z)*(4.0/(65535.0))*9.81;
}
static void readMag(float* data)
{
    uint8_t x_high = readByteI2C(0x1E, 0x03);
    uint8_t x_low = readByteI2C(0x1E, 0x04);
    uint8_t z_high = readByteI2C(0x1E, 0x05);
    uint8_t z_low = readByteI2C(0x1E, 0x06);
    uint8_t y_high = readByteI2C(0x1E, 0x07);
    uint8_t y_low = readByteI2C(0x1E, 0x08);
    int16_t val_x = (x_high << 8) | x_low;
    int16_t val_y = (y_high << 8) | y_low;
    int16_t val_z = (z_high << 8) | z_low;
    data[0] = ((float)val_x)*1.22;
    data[1] = ((float)val_y)*1.22;
    data[2] = ((float)val_z)*1.22;
}

/*
 * Application entry point.
 */
int main(void) {

    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    spiStart(&SPID1, &spi1cfg);
    i2cStart(&I2CD1, &i2cconfig);
    initGyro();
    initAccel();
    initMag();

    while (TRUE) {
	float gyroData[3];
        float accelData[3];
        float magData[3];
	readGyro(gyroData);
	readAccel(accelData);
	readMag(magData);
	chprintf((BaseSequentialStream *)&SDU1, "%f %f %f %f %f %f %f %f %f\n", gyroData[0], gyroData[1], gyroData[2], accelData[0], accelData[1], accelData[2], magData[0], magData[1], magData[2]);
    }
}
