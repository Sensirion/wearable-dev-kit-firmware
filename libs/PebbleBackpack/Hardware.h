/*
 * Hardware specific definition for the pebble backpack.
 *
 * Copyright (C) 2016 Sensirion AG, Switzerland
 * Author: Raphael Nestler <raphael.nestler@sensirion.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef HARDWARE_H
#define HARDWARE_H

#include "shtc1.h"
#include "stsc1.h"

// allow to globally override the USB serial (used for debug output and reading
// the log)
#ifndef USB_SERIAL
    #define USB_SERIAL SerialUSB
#endif

/**
 * Disable any peripheral that is not used
 */
void disableUnusedPeripherals();

/**
 * Enter sleep mode
 */
void sleepNow();

/**
 * Initialize hardware like SPI and Wire
 */
void initializeHardware();

/**
 * Completly disable USB. Should be called if no host is detected.
 */
void disableUSB();

/**
 * Reenable USB
 */
void enableUSB();

#ifndef USE_MPU
// Set to '1' to enable MPU support (increases power usage)
#define USE_MPU 0
#endif

#endif // HARDWARE_H

