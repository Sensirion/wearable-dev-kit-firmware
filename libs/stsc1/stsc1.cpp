/*
 * Sensirion's STSC1 temperature driver
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

#include "stsc1.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void stsc1::getTemperature(int32_t *temperature)
{
    uint16_t temperatureAdc;

    startMeasurement();

    readMeasurement(&temperatureAdc);
    // convert to milli deg c
    *temperature = convertAdcToMilliCelsius(temperatureAdc);
}

uint8_t stsc1::startMeasurement()
{
    // non clock stretch, T first, high resolution
    uint8_t txData[2] = {STS_MEASUREMENT_COMMAND_0, STS_MEASUREMENT_COMMAND_1};
    _Wire.beginTransmission(STS_ADDRESS);
    _Wire.write(txData, sizeof(txData));
    uint8_t error = _Wire.endTransmission();
    _startTime = millis();
    return error;
}

uint8_t stsc1::readMeasurement(uint16_t  *temperatureAdc)
{
    uint8_t readData[6];
    uint8_t error = 0;

    while ((millis() - _startTime) < STS_NONBLOCK_WAIT_TIME_MS) {
        // wait for measurement to finish
    }

    // 2 bytes T, 1 CRC
    _Wire.requestFrom(STS_ADDRESS, (uint8_t)3);
    uint8_t rxByteCount = 0;
    while (_Wire.available()) { // wait till all arrive
      readData[rxByteCount] = _Wire.read();
      rxByteCount++;
      // TODO Stop after 3 bytes
    }

    // if not all expected bytes arrived
    if (rxByteCount != 3) {
      error |= 1;
    }

    // TODO CRC check

    // merge chars to one int
    *temperatureAdc = BIU16(readData, 0);

    return error;
}

