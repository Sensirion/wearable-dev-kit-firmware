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

#include "PebbleBackpack.h"
#include "Logger.h"
#include "version.h"
#include <Perspiration.h>

MPU6500 mpu6500(N_CS_GYRO);
FlashW25Q128FW flash(N_CS_FLASH);
struct ScanResult {
    uint8_t address;
    uint8_t id;
};

enum SensorIds {
    None  = 0,
    Shtc1 = 0x07, // SHTW2, STSC1 and SHTW2_ALT have the same ID
};

/**
 * Reads a given number of words (16bit) from a sensor.
 * \return  true on error
 */
static bool readWordsFromI2c(TwoWire& wire, uint8_t address, uint8_t numberOfWords, uint16_t buffer[])
{
    const uint8_t bytesPerWord = 3; // 2 per word + crc
    const uint8_t numberOfBytes = bytesPerWord * numberOfWords;

    uint8_t bytes = wire.requestFrom(address, numberOfBytes);
    if (bytes != numberOfBytes) {
        return true;
    }
    for (uint8_t i=0; i<numberOfWords; ++i) {
        buffer[i] = wire.read()<<8;
        buffer[i] |= wire.read();
        // TODO: Check CRC
        uint8_t crc = wire.read();
    }
    return false;
}


/**
 *  Search at the given I2C bus for sensors.
 */
template<size_t N>
static void scanI2cBus(TwoWire& wire, ScanResult (&results)[N])
{
    static const uint8_t READ_ID_0 = 0xEF;
    static const uint8_t READ_ID_1 = 0xC8;

    static const uint8_t I2C_ADDR_LIST[] = {
        stsc1::STS_ADDRESS,
        shtc1::SHTC1_ADDRESS, // SHTW2 has the same address
        shtc1::SHTW2_ALT_ADDRESS,
    };

    uint8_t error = 0;
    uint16_t id = 0;    
    size_t n = 0;
    
    for (uint8_t a: I2C_ADDR_LIST) {
        wire.beginTransmission(a);
        wire.write(READ_ID_0);
        wire.write(READ_ID_1);
        uint8_t error = wire.endTransmission();

        if (error == 0) {
            DEBUG_PRINT("Found 0x");
            DEBUG_PRINT(a, HEX);
            uint16_t id = 0;
            bool error = readWordsFromI2c(wire, a, 1, &id);
            // only the lower 6 bits of the id register are valid
            id &= 0x3F;
            if (!error) {
                DEBUG_PRINT(" ID: 0x");
                DEBUG_PRINT(id);
                results[n].address = a;
                results[n].id = id;
                n++;
            }
            DEBUG_PRINTLN("");
        }
    }
}

void PebbleBackpack::begin()
{
    DEBUG_PRINT("PebbleBackpack::begin Version:");
    DEBUG_PRINTLN(VERSION);
    // General init
    initializeHardware();
    logger.begin();

    DEBUG_PRINTLN("Scan Wire");
    ScanResult wire_results[3] = {};
    scanI2cBus(Wire, wire_results);
    DEBUG_PRINTLN("Scan Wire1");
    ScanResult wire1_results[3] = {};
    scanI2cBus(Wire1, wire1_results);

    /**
     * Configure the connected sensors.
     * For the skin and ambient sensors we have 4 hardware configurations:
     * Skin        / Ambient
     * STSC1@Wire  / SHTC1@Wire1
     * SHTC1@Wire  / SHTC1@Wire1
     * SHTW2@Wire  / SHTW2_alt@Wire1
     * SHTW2_alt@Wire / SHTW2@Wire1
     */
    for (auto const &scanResult : wire_results) {
        if (scanResult.address == stsc1::STS_ADDRESS) {
            if (_stsc1Skin == nullptr) {
                DEBUG_PRINT("Wire: Create _stsc1Skin: ");
                DEBUG_PRINTLN(scanResult.address, HEX);
                _stsc1Skin = new stsc1(Wire);
            } else {
                DEBUG_PRINTLN("Error: _stsc1Skin already exists");
            }

        } else if (scanResult.address == shtc1::SHTC1_ADDRESS) {
            if (_shtc1Skin == nullptr && _stsc1Skin == nullptr) {
                DEBUG_PRINT("Wire: Create _shtc1Skin: ");
                DEBUG_PRINTLN(scanResult.address, HEX);
                _shtc1Skin = new shtc1(Wire, scanResult.address);
            } else {
                DEBUG_PRINTLN("Error: _stsc1Skin or _shtc1Skin already exists");
            }
        } else if (scanResult.address == shtc1::SHTW2_ALT_ADDRESS) {
            if (_shtc1Ambient == nullptr) {
                DEBUG_PRINT("Wire: Create _shtc1Ambient: ");
                DEBUG_PRINTLN(scanResult.address, HEX);
                _shtc1Ambient = new shtc1(Wire, scanResult.address);
            } else {
                DEBUG_PRINTLN("Error: _shtc1Ambient already exists");
            }
        }
    }

    for (auto const &scanResult : wire1_results) {
        if (scanResult.id == SensorIds::Shtc1) {
            // if no skin sensor were found on Wire both skin and ambient are on Wire1
            if (_shtc1Skin == nullptr && _stsc1Skin == nullptr) {
                DEBUG_PRINT("Wire1: Create _shtc1Skin: ");
                DEBUG_PRINTLN(scanResult.address, HEX);
                _shtc1Skin = new shtc1(Wire1, scanResult.address);
            } else if (_shtc1Ambient == nullptr) {
                DEBUG_PRINT("Wire1: Create _shtc1Ambient: ");
                DEBUG_PRINTLN(scanResult.address, HEX);
                _shtc1Ambient = new shtc1(Wire1, scanResult.address);
            } else {
                DEBUG_PRINTLN("Error: _shtc1Ambient already exists");
            }
        }
    }

    // check which sensor readings and processed values are available due to
    // connected sensors
    if (_shtc1Ambient) {
        _availableSensorReadings |= SensorReadingsAttributes::TemperatureUpper |
            SensorReadingsAttributes::HumidityUpper;
        _availableProcessedValues |= ProcessedValuesAttributes::SkinTemperature |
            ProcessedValuesAttributes::HeatIndex |
            ProcessedValuesAttributes::Humidex |
            ProcessedValuesAttributes::ApparentTemperature |
            ProcessedValuesAttributes::CompensationMode;
    }

    if (_shtc1Skin) {
        _availableSensorReadings |= SensorReadingsAttributes::TemperatureLower |
            SensorReadingsAttributes::HumidityLower;

        _availableProcessedValues |= ProcessedValuesAttributes::Perspiration |
            ProcessedValuesAttributes::OnOffBodyState;

    } else if (_stsc1Skin) {
        _availableSensorReadings |= static_cast<uint16_t>(SensorReadingsAttributes::TemperatureLower);
    }

    // give sensors time to fully perform power on reset
    delay(20);

    mpu6500.begin();
#if USE_MPU
    // start MPU sampling with given interval
    uint8_t mpu_sample_rate = (_sensorPollIntervalMs > 255) ?
        255 : (_sensorPollIntervalMs - 1);
    mpu6500.startSampling(mpu_sample_rate);

    _availableSensorReadings |= SensorReadingsAttributes::AccelerationX |
        SensorReadingsAttributes::AccelerationY |
        SensorReadingsAttributes::AccelerationZ |
        SensorReadingsAttributes::GyroX |
        SensorReadingsAttributes::GyroY |
        SensorReadingsAttributes::GyroZ |
        SensorReadingsAttributes::MPU6500Temperature;
#else
    mpu6500.setSleepMode();
#endif

    DEBUG_PRINT("Available sensor readings: ");
    DEBUG_PRINTLN(_availableSensorReadings, HEX);

    DEBUG_PRINT("Available processed values: ");
    DEBUG_PRINTLN(_availableProcessedValues, HEX);

    // initialize processing engines
    initTemperatureCompensation(_temperatureCompensationMode);
}

bool PebbleBackpack::readSensors()
{
#if USE_MPU
    const size_t PACKET_LEN = 14; // number of bytes in data packet
    uint8_t mpu6500_data[PACKET_LEN] = {};
    if (mpu6500.readFifo(mpu6500_data, PACKET_LEN)) {
        _acc[0] = (mpu6500_data[0] << 8) + mpu6500_data[1];
        _acc[1] = (mpu6500_data[2] << 8) + mpu6500_data[3];
        _acc[2] = (mpu6500_data[4] << 8) + mpu6500_data[5];

        _mpuTemperature = (mpu6500_data[6] << 8) + mpu6500_data[7];

        _gyro[0] = (mpu6500_data[8] << 8) + mpu6500_data[9];
        _gyro[1] = (mpu6500_data[10] << 8) + mpu6500_data[11];
        _gyro[2] = (mpu6500_data[12] << 8) + mpu6500_data[13];

        _fifo_count = mpu6500.getFifoCount();
    }
#endif

    unsigned long currentMillis = millis();
    if ((currentMillis - _previousMillis) >= _sensorPollIntervalMs) {
        _previousMillis = currentMillis;

        uint8_t error = 0;
        if (_shtc1Ambient) {
            error = _shtc1Ambient->startMeasurement();
            if (error) {
                DEBUG_PRINT("_shtc1Ambient error: ");
                DEBUG_PRINTLN(error);
            }
        }
        if (_shtc1Skin) {
            error = _shtc1Skin->startMeasurement();
            if (error) {
                DEBUG_PRINT("_shtc1Skin error: ");
                DEBUG_PRINTLN(error);
            }
        }
        if (_stsc1Skin) {
            error = _stsc1Skin->startMeasurement();
            if (error) {
                DEBUG_PRINT("_stsc1Skin error: ");
                DEBUG_PRINTLN(error);
            }
        }

        uint16_t adcTemperature;
        uint16_t adcHumidity;

        if (_shtc1Ambient) {
            error = _shtc1Ambient->readMeasurement(&adcHumidity, &adcTemperature);
            if (error) {
                DEBUG_PRINT("_shtc1Ambient error: ");
                DEBUG_PRINTLN(error);
            }
            _humidity_upper = shtc1::convertAdcToHumidity(adcHumidity);
            _temperature_upper = shtc1::convertAdcToMilliCelsius(adcTemperature);
        }

        if (_shtc1Skin) {
            error = _shtc1Skin->readMeasurement(&adcHumidity, &adcTemperature);
            if (error) {
                DEBUG_PRINT("_shtc1Skin error: ");
                DEBUG_PRINTLN(error);
            }
            _humidity_lower = shtc1::convertAdcToHumidity(adcHumidity);
            _temperature_lower = shtc1::convertAdcToMilliCelsius(adcTemperature);
        }
        else if (_stsc1Skin) {
            error = _stsc1Skin->readMeasurement(&adcTemperature);
            if (error) {
                DEBUG_PRINT("_stsc1Skin error: ");
                DEBUG_PRINTLN(error);
            }
            _temperature_lower = stsc1::convertAdcToMilliCelsius(adcTemperature);
        }


        _sensorReadTime = millis() - _previousMillis;

        temperatureCompensation(_previousMillis, _temperature_lower * 0.001f,
                                _temperature_upper * 0.001f, _humidity_upper * 0.001f, &_skinTemperature,
                                &_heatIndex, &_apparentTemperature, &_humidex, &_airTouchEvent);

        perspirationEngine(_previousMillis, _temperature_upper * 0.001f, _humidity_upper * 0.001f,
                           _temperature_lower * 0.001f, _humidity_lower * 0.001f,
                           &_perspiration);

        onOffBodyEngine(_previousMillis, _temperature_upper * 0.001f, _humidity_upper * 0.001f,
                        _temperature_lower * 0.001f, _humidity_lower * 0.001f,
                        &_onOffBodyState);

        return true;
    }

    return false;
}

bool PebbleBackpack::getSensorValue(SensorReadingsAttributes sensor, void **address, size_t *size)
{
    switch (sensor) {
        case SensorReadingsAttributes::TemperatureUpper:
            *address = &_temperature_upper;
            *size = sizeof(_temperature_upper);
            return true;
        case SensorReadingsAttributes::HumidityUpper:
            *address = &_humidity_upper;
            *size = sizeof(_humidity_upper);
            return true;

        case SensorReadingsAttributes::TemperatureLower:
            *address = &_temperature_lower;
            *size = sizeof(_temperature_lower);
            return true;
        case SensorReadingsAttributes::HumidityLower:
            *address = &_humidity_lower;
            *size = sizeof(_humidity_lower);
            return true;

        case SensorReadingsAttributes::ReservedData0:
            *address = &_reserved0;
            *size = sizeof(_reserved0);
            return true;
        case SensorReadingsAttributes::ReservedData1:
            *address = &_reserved1;
            *size = sizeof(_reserved1);
            return true;

#if USE_MPU
        case SensorReadingsAttributes::AccelerationX:
            *address = &_acc[0];
            *size = sizeof(_acc[0]);
            return true;
        case SensorReadingsAttributes::AccelerationY:
            *address = &_acc[1];
            *size = sizeof(_acc[1]);
            return true;
        case SensorReadingsAttributes::AccelerationZ:
            *address = &_acc[2];
            *size = sizeof(_acc[2]);
            return true;
        case SensorReadingsAttributes::GyroX:
            *address = &_gyro[0];
            *size = sizeof(_gyro[0]);
            return true;
        case SensorReadingsAttributes::GyroY:
            *address = &_gyro[1];
            *size = sizeof(_gyro[1]);
            return true;
        case SensorReadingsAttributes::GyroZ:
            *address = &_gyro[2];
            *size = sizeof(_gyro[2]);
            return true;
        case SensorReadingsAttributes::MPU6500Temperature:
            *address = &_mpuTemperature;
            *size = sizeof(_mpuTemperature);
            return true;
#endif

        default:
            return false;
    }
}

bool PebbleBackpack::getProcessedValue(ProcessedValuesAttributes value, void **address, size_t *size)
{
    switch (value) {
        case ProcessedValuesAttributes::SkinTemperature:
            *address = &_skinTemperature;
            *size = sizeof(_skinTemperature);
            return true;
        case ProcessedValuesAttributes::ApparentTemperature:
            *address = &_apparentTemperature;
            *size = sizeof(_apparentTemperature);
            return true;
        case ProcessedValuesAttributes::HeatIndex:
            *address = &_heatIndex;
            *size = sizeof(_heatIndex);
            return true;
        case ProcessedValuesAttributes::Humidex:
            *address = &_humidex;
            *size = sizeof(_humidex);
            return true;
        case ProcessedValuesAttributes::CompensationMode:
            *address = &_temperatureCompensationMode;
            *size = sizeof(_temperatureCompensationMode);
            return true;

        case ProcessedValuesAttributes::Perspiration:
            *address = &_perspiration;
            *size = sizeof(_perspiration);
            return true;
        case ProcessedValuesAttributes::OnOffBodyState:
            *address = &_onOffBodyState;
            *size = sizeof(_onOffBodyState);
            return true;

        default:
            return false;
    }
}
