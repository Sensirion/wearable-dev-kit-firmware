/*
 * Pebble protocol definitions and helper functions for the pebble backpack.
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

#ifndef PEBBLE_PROTOCOL_H
#define PEBBLE_PROTOCOL_H

#include <Arduino.h>

#include "DebugPrint.h"

enum struct DataType: uint8_t {
    Invalid = 0,
    Int8    = 1,
    UInt8   = 2,
    Int16   = 3,
    UInt16  = 4,
    Int32   = 6,
    UInt32  = 7,
    Float   = 8,
};

enum struct Services: uint16_t {
    SensorReadings  = 0x1001,
    ProcessedValues = 0x1002,
    Logger          = 0x1003,
    System          = 0x1004,
};

enum struct SensorReadingsAttributes: uint16_t {
    // first bit 0 means the following bits are a bitmask
    TemperatureUpper            = 1<<0,     // int32_t m°C
    HumidityUpper               = 1<<1,     // int32_t m%RH

    TemperatureLower            = 1<<2,     // int32_t m°C
    HumidityLower               = 1<<3,     // int32_t m%RH

    ReservedData0              = 1<<4,     // int32_t ...
    ReservedData1              = 1<<5,     // int32_t ...

    Reserved0                   = 1<<6,     // must always be zero
    Reserved1                   = 1<<7,     // must always be zero

    AccelerationX               = 1<<8,     // int16_t raw
    AccelerationY               = 1<<9,    // int16_t raw
    AccelerationZ               = 1<<10,    // int16_t raw
    GyroX                       = 1<<11,    // int16_t raw
    GyroY                       = 1<<12,    // int16_t raw
    GyroZ                       = 1<<13,    // int16_t raw
    MPU6500Temperature          = 1<<14,    // int16_t raw

    // mask that contains all reserved flags
    ReservedMask = Reserved0 | Reserved1,

    // first bit 1 means reserved values
    // 0x8000 .. 0xFFFF are reserved
};

static const int MAX_SENSOR_READINGS  = 15;

const char* SensorReadingToString(uint8_t index);

DataType SensorReadingDataType(uint8_t index);

/**
 * Implement bitmask checking
 */
constexpr uint16_t operator&(uint16_t value, SensorReadingsAttributes attribute)
{
    return value & static_cast<uint16_t>(attribute);
}

constexpr uint16_t operator|(SensorReadingsAttributes left, SensorReadingsAttributes right)
{
    return static_cast<uint16_t>(left) | static_cast<uint16_t>(right);
}

enum struct ProcessedValuesAttributes: uint16_t {
    // first bit 0 means the following bits are a bitmask
    SkinTemperature     = 1<<0,     // float °C
    ApparentTemperature = 1<<1,     // float °C
    HeatIndex           = 1<<2,     // float °C
    Humidex             = 1<<3,     // float °C
    CompensationMode    = 1<<4,     // uint8_t as set with TemperatureCompensationMode

    Perspiration        = 1<<5,     // float dimensionless
    OnOffBodyState      = 1<<6,     // uint8_t 0: off-body, 1: on-body

    Reserved0           = 1<<7,     // must always be zero
    Reserved1           = 1<<8,     // must always be zero
    Reserved2           = 1<<9,     // must always be zero
    Reserved3           = 1<<10,    // must always be zero
    Reserved4           = 1<<11,    // must always be zero
    Reserved5           = 1<<12,    // must always be zero
    Reserved6           = 1<<13,    // must always be zero
    Reserved7           = 1<<14,    // must always be zero

    // mask that contains all reserved flags
    ReservedMask = Reserved0 | Reserved1 | Reserved2 | Reserved3\
                     | Reserved4 | Reserved5 | Reserved6 | Reserved7,

    // first bit 1 means reserved values
    // 0x8000 .. 0xFFFF are reserved
    ReservedEvent      = 0x8000,    // write uint8_t

    AirTouchStartEvent     = 0x8001,   // notification
    AirTouchStopEvent      = 0x8002,   // notification

    /**
     * Read:    currentMode[uint8_t], numberOfModes[uint8_t]
     * Write:   newMode[uint8_t, 0..numberOfModes-1]
     * See enum TemperatureCompensationMode for the modes
     */
    TemperatureCompensationMode = 0x8003,

    OnBodyEvent       = 0x8004,   // notification
    OffBodyEvent      = 0x8005,   // notification
};

static const int MAX_PROCESSED_VALUES = 15;

const char* ProcessedValuesToString(uint8_t index);

DataType ProcessedValuesDataType(uint8_t index);

/**
 * Implement bitmask checking
 */
constexpr uint16_t operator&(uint16_t value, ProcessedValuesAttributes attribute)
{
    return value & static_cast<uint16_t>(attribute);
}

constexpr uint16_t operator|(ProcessedValuesAttributes left, ProcessedValuesAttributes right)
{
    return static_cast<uint16_t>(left) | static_cast<uint16_t>(right);
}

constexpr uint16_t operator|(uint16_t left, ProcessedValuesAttributes right)
{
    return left | static_cast<uint16_t>(right);
}

/**
 * Implement equality to uint16_t
 */
constexpr bool operator==(uint16_t value, ProcessedValuesAttributes attribute)
{
    return value == static_cast<uint16_t>(attribute);
}

enum struct LoggerAttributes: uint16_t {
    ClearLog        = 0x0001, // Write: no arguments
    StartLogging    = 0x0002, // Write: LogHeader (16 Bytes)
    PauseLogging    = 0x0003, // Write: no arguments
    ResumeLogging   = 0x0004, // Write: no arguments
    LogEntries      = 0x0005, // Read: uint32_t Number of written log entries
    State           = 0x0006, // Read: uint8_t Logger::State
};

enum struct SystemAttributes: uint16_t {
    Status                   = 0x0001, // Read: uint32_t with status flags
    USBPlugged               = 0x0002, // Write: no arguments
    USBUnPlugged             = 0x0003, // Write: no arguments
    Version                  = 0x0004, // Read: version string
    AvailableSensorReadings  = 0x0005, // Read: uin16_t mask with available sensors
    AvailableProcessedValues = 0x0006, // Read: uin16_t mask with available processed values
};

template<typename T, size_t N>
size_t putInBuffer(uint8_t (&buffer)[N], size_t position, T value)
{
    if(position + sizeof(value) > N) {
        DEBUG_PRINTLN("Buffer not big enough!");
        return -1;
    }
    DEBUG_PRINT("putInBuffer(");
    DEBUG_PRINT(position);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(value);
    DEBUG_PRINTLN(")")
    memcpy(buffer + position, &value, sizeof(value));

    return position + sizeof(value);
}

template<typename T>
size_t readFromBuffer(size_t position, uint8_t *buffer, size_t length, T* value)
{
    if(position + sizeof(T) > length) {
        DEBUG_PRINTLN("Buffer not big enough!");
        return -1;
    }
    DEBUG_PRINT("readFromBuffer(");
    DEBUG_PRINT(position);
    DEBUG_PRINTLN(")")
    memcpy(value, buffer + position, sizeof(T));

    return position + sizeof(T);
}

#endif // PEBBLE_PROTOCOL_H
