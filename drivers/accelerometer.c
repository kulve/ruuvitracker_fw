/*
 *  MMA8652FC 3-Axis accelerometer driver for Ruuvitracker.
 *
 * @author: Tuomas Kulve
 */

#include "ch.h"
#include "hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "accelerometer.h"

/* The threshold resolution is 0.063 g/LSB */
#define ACC_MT_THS_RESOLUTION        0.063

/* Enable detection of over 1.1G values*/
#define ACC_MT_THS_G                   1.1

/* Standard slave address for MMA8652FC */
#define ACC_ADDR                      0x1D

/* Device identification ID */
#define MMA8652FC_WHO_AM_I            0x4A

/* MMA8652FC registers */
#define STATUS                        0x00
#define SYSMOD                        0x0B
#define WHO_AM_I                      0x0D
#define FF_MT_CFG                     0x15
#define FF_MT_THS                     0x17
#define CTRL_REG1                     0x2A
#define CTRL_REG2                     0x2B
#define CTRL_REG3                     0x2C
#define CTRL_REG4                     0x2D
#define CTRL_REG5                     0x2D

/* MMA8652FC register bits */
#define SYSMOD_SYSMOD_STANDBY         0b00
#define SYSMOD_SYSMOD_WAKE            0b01
#define SYSMOD_SYSMOD_SLEEP           0b10
#define FF_MT_CFG_ELE             (1 << 7)
#define FF_MT_CFG_OAE             (1 << 6)
#define FF_MT_CFG_ZEFE            (1 << 5)
#define FF_MT_CFG_YEFE            (1 << 4)
#define FF_MT_CFG_XEFE            (1 << 3)
#define FF_MT_THS_DBCNTM          (1 << 7)
#define CTRL_REG1_ACTIVE          (1 << 0)
#define CTRL_REG4_INT_EN_FF_MT    (1 << 2)
#define CTRL_REG5_INT_CFG_FF_MT   (1 << 2)

static I2CConfig i2c_cfg = {
  .op_mode = OPMODE_I2C,
  .clock_speed = 400000,
  .duty_cycle = FAST_DUTY_CYCLE_2
};

/* Enable motion detect on specified pin */
void acc_mt_enable(uint8_t pin)
{
	uint8_t txbuf[2];
	uint8_t rxbuf[1];
	uint8_t i;

	i2cStart(&I2CD1, &i2c_cfg);

	// Read WHO AM I
	txbuf[0] = WHO_AM_I;
	i2cMasterTransmit(&I2CD1, ACC_ADDR, txbuf, 1, rxbuf, 1);

    //chDbgAssert(rxbuf[0] == MMA8652FC_WHO_AM_I, "Invalid WHO AM I from MMA8652FC", "#1");

	// Enable active mode
	txbuf[0] = CTRL_REG1;
	txbuf[1] = CTRL_REG1_ACTIVE;
	i2cMasterTransmit(&I2CD1, ACC_ADDR, txbuf, 2, NULL, 0);

	// Enable motion detection on all directions
	txbuf[0] = FF_MT_CFG;
	txbuf[1] = FF_MT_CFG_OAE | FF_MT_CFG_ZEFE | FF_MT_CFG_YEFE | FF_MT_CFG_XEFE;
	i2cMasterTransmit(&I2CD1, ACC_ADDR, txbuf, 2, NULL, 0);

	// Set motion detection threshold and keep the interrupt line active
	// as long as the motion continues
	txbuf[0] = FF_MT_THS;
	txbuf[1] = FF_MT_THS_DBCNTM | (uint8_t)(ACC_MT_THS_G / ACC_MT_THS_RESOLUTION);
	i2cMasterTransmit(&I2CD1, ACC_ADDR, txbuf, 2, NULL, 0);

	if (pin == 1) {
	  // Use int1 for interrupts (int2 is the default)
	  txbuf[0] = CTRL_REG5;
	  txbuf[1] = CTRL_REG5_INT_CFG_FF_MT;
	  i2cMasterTransmit(&I2CD1, ACC_ADDR, txbuf, 2, NULL, 0);
	}

	// Enable motion interrupts
	txbuf[0] = CTRL_REG4;
	txbuf[1] = CTRL_REG4_INT_EN_FF_MT;
	i2cMasterTransmit(&I2CD1, ACC_ADDR, txbuf, 2, NULL, 0);
}

void acc_mt_disable()
{
	uint8_t txbuf[2];

	// Disable motion interrupts
	txbuf[0] = CTRL_REG4;
	txbuf[1] = 0;
	i2cMasterTransmit(&I2CD1, ACC_ADDR, txbuf, 2, NULL, 0);

	// Enter standby mode
	txbuf[0] = CTRL_REG1;
	txbuf[1] = 0;
	i2cMasterTransmit(&I2CD1, ACC_ADDR, txbuf, 2, NULL, 0);

	i2cStop(&I2CD1);
}
