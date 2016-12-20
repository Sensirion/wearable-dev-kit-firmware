/*
 * Describes the pebble backpack hardware
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

#ifndef PEBBLE_BACKPACK_H
#define PEBBLE_BACKPACK_H

#include <Wire.h>
#include <shtc1.h>
#include <stsc1.h>

#include "Hardware.h"
#include "PebbleProtocol.h"

#include <MPU6500.h>
#include <TemperatureCompensation.h>
#include <OnOffBody.h>
#include <FlashW25Q128FW.h>

static const int N_CS_GYRO = 8;
static const int N_CS_FLASH = 10;

extern MPU6500 mpu6500;
extern FlashW25Q128FW flash;

/**
 * Provide access to all pebble backpack peripherals.
 */
class PebbleBackpack {
public:
    PebbleBackpack(unsigned long sensorPollIntervalMs):
        _sensorPollIntervalMs(sensorPollIntervalMs)
    {}

    /**
     * Scans the I2C buses for available hardare.
     */
    void begin();

    /**
      Read all the sensors. Returns true if new sensor values are available.
     */
    bool readSensors();

    /**
     * Get the address and size of a given sensor reading attribute. This is
     * useful when one just needs to copy it into a buffer.
     * @return true on success
     */
    bool getSensorValue(SensorReadingsAttributes sensor, void **address, size_t *size);

    /**
     * Type safe overload
     */
    template<typename T>
    T getSensorValue(SensorReadingsAttributes attribute);

    /**
     * Get the address and size of a given processed value attribute. This is
     * useful when one just needs to copy it into a buffer.
     * @return true on success
     */
    bool getProcessedValue(ProcessedValuesAttributes value, void **address, size_t *size);

    /**
     * Type safe overload
     */
    template<typename T>
    T getProcessedValue(ProcessedValuesAttributes attribute);

    /**
     * Get a mask of available sensor readings, depending on the connected
     * sensors.
     */
    uint16_t getAvailableSensorReadings()
    { return _availableSensorReadings; }

    /**
     * Check if the given sensor is available
     */
    bool isSensorAvailable(SensorReadingsAttributes attribute) const
    { return _availableSensorReadings & attribute; }

    /**
     * Check if the sensor at the given index is available
     */
    bool isSensorAvailable(uint8_t index) const
    { return _availableSensorReadings & (1<<index); }

    /**
     * Get a mask of available processed values, depending on the connected
     * sensors.
     */
    uint16_t getAvailableProcessedValues()
    { return _availableProcessedValues; }

    /**
     * Check if the given processed value is available
     */
    bool isProcessedValueAvailable(ProcessedValuesAttributes attribute) const
    { return _availableProcessedValues & attribute; }

    /**
     * Check if the processed value at the given index is available
     */
    bool isProcessedValueAvailable(uint8_t index) const
    { return _availableProcessedValues & (1<<index); }


    int32_t getTemperatureUpper()
    { return _temperature_upper; }

    int32_t getHumidityUpper()
    { return _humidity_upper; }

    int32_t getTemperatureLower()
    { return _temperature_lower; }

    int32_t getHumidityLower()
    { return _humidity_lower; }

    int32_t getReserved0()
    { return _reserved0; }

    int32_t getReserved1()
    { return _reserved1; }

    float getPerspiration()
    { return _perspiration; }

    OnOffBodyState getOnOffBodyState()
    { return _onOffBodyState; }

#if USE_MPU
    int16_t getMpuTemperature()
    { return _mpuTemperature; }

    int16_t getAccX()
    { return _acc[0]; }
    int16_t getAccY()
    { return _acc[1]; }
    int16_t getAccZ()
    { return _acc[2]; }

    int16_t getGyroX()
    { return _gyro[0]; }
    int16_t getGyroY()
    { return _gyro[1]; }
    int16_t getGyroZ()
    { return _gyro[2]; }

    uint16_t getFifoCount()
    { return _fifo_count; }
#endif

    unsigned long getTimestamp()
    { return _previousMillis; }

    unsigned long getSensorReadTime()
    { return _sensorReadTime; }

    float getSkintemperature()
    { return _skinTemperature; }

    float getHeatIndex()
    { return _heatIndex; }

    float getHumidex()
    { return _humidex; }

    float getApparentTemperature()
    { return _apparentTemperature; }

    TemperatureCompensationMode getTemperatureCompensationMode()
    { return _temperatureCompensationMode; }

    void setTemperatureCompensationMode(TemperatureCompensationMode compensationMode)
    {
        _temperatureCompensationMode = compensationMode;
        initTemperatureCompensation(_temperatureCompensationMode);
    }

    AirTouchEvent getAirTouchEvent()
    { return _airTouchEvent; }

private:
    unsigned long _sensorPollIntervalMs = 0;
    unsigned long _previousMillis = 0;

    shtc1* _shtc1Ambient = nullptr;

    // only one of those is set
    stsc1* _stsc1Skin = nullptr;
    shtc1* _shtc1Skin = nullptr;

    // keep track of the available sensors
    // see SensorReadingsAttributes
    uint16_t _availableSensorReadings = 0;

    // keep track of the available processed values
    // see ProcessedValuesAttributes
    uint16_t _availableProcessedValues = 0;

#if USE_MPU
    int16_t _mpuTemperature = 0;
    int16_t _acc[3] = { 0, 0, 0 };
    int16_t _gyro[3] = { 0, 0, 0 };
    uint16_t _fifo_count = 0;
#endif

    int32_t _humidity_upper = 0;
    int32_t _temperature_upper = 0;

    int32_t _temperature_lower = 0;
    int32_t _humidity_lower = 0;

    int32_t _reserved0 = 0;
    int32_t _reserved1 = 0;

    float _perspiration = 0.0f;
    OnOffBodyState _onOffBodyState = OnOffBodyState::OffBody;

    // time needed to do the measurement
    unsigned long _sensorReadTime = 0;

    float _skinTemperature = 0.0f;
    float _heatIndex = 0.0f;
    float _humidex = 0.0f;
    float _apparentTemperature = 0.0f;
    AirTouchEvent _airTouchEvent = AirTouchEvent::None;

    TemperatureCompensationMode _temperatureCompensationMode =
        TemperatureCompensationMode::Medium;
};

template<typename T>
T PebbleBackpack::getSensorValue(SensorReadingsAttributes attribute)
{
    T value = T();
    void* address;
    size_t size;
    bool success = getSensorValue(attribute, &address, &size);
    if (!success) {
        return value;
    }
    if (size != sizeof(T)) {
        return value;
    }
    memcpy(&value, address, size);
    return value;
}

template<typename T>
T PebbleBackpack::getProcessedValue(ProcessedValuesAttributes attribute)
{
    T value = T();
    void* address;
    size_t size;
    bool success = getProcessedValue(attribute, &address, &size);
    if (!success) {
        return value;
    }
    if (size != sizeof(T)) {
        return value;
    }
    memcpy(&value, address, size);
    return value;
}

#endif // PEBBLE_BACKPACK_H
