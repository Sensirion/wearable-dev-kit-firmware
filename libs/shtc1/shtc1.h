/*
 * Sensirion's SHTC1 humidity and temperature driver
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

#ifndef SHTC1_H
#define SHTC1_H

#include <inttypes.h>
#include <Wire.h>
#include "Arduino.h"

#define BIU16(data, start) (((uint16_t)(data)[start]) << 8 | ((data)[start + 1]))

class shtc1 {
public:
    // wait time converted in ms + 1 ms rounding error
    static const unsigned long SHT_NONBLOCK_WAIT_TIME_MS = 15;
    static const uint8_t SHTC1_ADDRESS = 0x70;
    // the SHTW2 and SHTW2_ALT are compatible with the SHTC1
    static const uint8_t SHTW2_ADDRESS = 0x70;
    static const uint8_t SHTW2_ALT_ADDRESS = 0x73;
    static const uint8_t SHT_MEASUREMENT_COMMAND_0 = 0x78;
    static const uint8_t SHT_MEASUREMENT_COMMAND_1 = 0x66;

    shtc1(TwoWire& wire, uint8_t i2cAddress = SHTC1_ADDRESS):
        _Wire(wire),
        _I2cAddress(i2cAddress)
    {}

    void getHumidityTemperature(int32_t *humidity, int32_t *temperature);

    uint8_t startMeasurement();
    uint8_t readMeasurement(uint16_t *humidityAdc, uint16_t *temperatureAdc);

    static int32_t convertAdcToHumidity(uint16_t humidityAdc)
    { return (12500 * (int32_t)humidityAdc) >> 13; }

    static int32_t convertAdcToMilliCelsius(uint16_t temperatureAdc)
    { return ((21875 * (int32_t)temperatureAdc) >> 13) - 45000; }

private:
    unsigned long _startTime = 0;
    TwoWire& _Wire;
    uint8_t _I2cAddress = SHTC1_ADDRESS;
};

#endif

