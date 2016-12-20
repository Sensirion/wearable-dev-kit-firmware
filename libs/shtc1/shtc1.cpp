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

#include "shtc1.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void shtc1::getHumidityTemperature(int32_t *humidity, int32_t *temperature)
{
    uint16_t humidityAdc;
    uint16_t temperatureAdc;

    startMeasurement();

    readMeasurement(&humidityAdc, &temperatureAdc);
    // convert to milli deg c and 1/1000 of % RH
    *humidity = convertAdcToHumidity(humidityAdc);
    *temperature = convertAdcToMilliCelsius(temperatureAdc);
}

uint8_t shtc1::startMeasurement()
{
    // non clock stretch, T first, high resolution
    uint8_t txData[2] = {SHT_MEASUREMENT_COMMAND_0, SHT_MEASUREMENT_COMMAND_1};

    _Wire.beginTransmission(_I2cAddress);
    _Wire.write(txData, sizeof(txData));
    uint8_t error = _Wire.endTransmission();
    _startTime = millis();
    return error;
}

uint8_t shtc1::readMeasurement(uint16_t *humidityAdc, uint16_t  *temperatureAdc)
{
    uint8_t readData[6];
    uint8_t error = 0;
    uint8_t rxByteCount;

    while ((millis() - _startTime) < SHT_NONBLOCK_WAIT_TIME_MS) {
        // wait for measurement to finish
    }

    // 2 bytes RH, 1 CRC, 2 bytes T, 1 CRC
    _Wire.requestFrom(_I2cAddress, (uint8_t)6);
    rxByteCount = 0;
    while (_Wire.available()) { // wait till all arrive
        readData[rxByteCount] = _Wire.read();
        rxByteCount++;
    }

    // if not all expected bytes arrived
    if (rxByteCount != 6) {
        error |= 1;
    }

    // TODO CRC check values

    // merge chars to one int
    *temperatureAdc = BIU16(readData, 0);
    *humidityAdc = BIU16(readData, 3);

    return error;
}

