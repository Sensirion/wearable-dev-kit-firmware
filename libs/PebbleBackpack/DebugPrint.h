/*
 * Firmware for the pebble backpack.
 *
 * Copyright (C) 2015 Sensirion AG, Switzerland
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

#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#include "Hardware.h"

// allow to override debug output destination
#ifndef DEBUG_PRINTER
    #define DEBUG_PRINTER USB_SERIAL
#endif

// allow to enable/disable debugging
#ifdef DEBUG
    #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
    #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
    #define DEBUG_PRINT(...) {}
    #define DEBUG_PRINTLN(...) {}
#endif

#endif // DEBUG_PRINT_H
