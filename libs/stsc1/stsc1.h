/*
 * Sensirion's STSC1 temperature sensor driver
 *
 * Copyright (C) 2015 Sensirion AG, Switzerland
 * Author: Daniel Lehmann <daniel.lehmann@sensirion.com>
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

#ifndef STSC1_H
#define STSC1_H

#include <inttypes.h>
#include <Wire.h>
#include "Arduino.h"

#define BIU16(data, start) (((uint16_t)(data)[start]) << 8 | ((data)[start + 1]))

class stsc1 {
public:
    stsc1(TwoWire& wire):
        _Wire(wire)
    {}
    // wait time converted in ms + 1 ms rounding error
    static const unsigned STS_NONBLOCK_WAIT_TIME_MS = 15;
    static const uint8_t STS_ADDRESS = 0x4A;
    static const uint8_t STS_MEASUREMENT_COMMAND_0 = 0x78;
    static const uint8_t STS_MEASUREMENT_COMMAND_1 = 0x66;

    void getTemperature(int32_t *temperature);

    uint8_t startMeasurement();
    uint8_t readMeasurement(uint16_t *temperatureAdc);

    static int32_t convertAdcToMilliCelsius(uint16_t temperatureAdc)
    { return ((21875 * (int32_t)temperatureAdc) >> 13) - 45000; }

private:
    unsigned long _startTime = 0;
    TwoWire& _Wire;
};

#endif

