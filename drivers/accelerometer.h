/*
 *  MMA8652FC 3-Axis accelerometer driver for RuuviTracker.
 *
 * @author: Tuomas Kulve
 */

#ifndef _ACCELEROMETER_H_
#define _ACCELEROMETER_H_

/******** API ***************/

/**
 * Start accelerometer sensor in motion detection mode using
 * interrupts in pin 1 or 2
 */
void acc_mt_enable(uint8_t pin);

/**
 * Stop accelerometer
 */
void acc_mt_disable(void);

#endif

