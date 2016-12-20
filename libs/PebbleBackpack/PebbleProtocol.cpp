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

#include "PebbleProtocol.h"

const char* SensorReadingToString(uint8_t index) {
    switch (static_cast<SensorReadingsAttributes>(1<<index)) {
        case SensorReadingsAttributes::TemperatureUpper:
            return "TemperatureUpper";
        case SensorReadingsAttributes::HumidityUpper:
            return "HumidityUpper";

        case SensorReadingsAttributes::TemperatureLower:
            return "TemperatureLower";
        case SensorReadingsAttributes::HumidityLower:
            return "HumidityLower";

        case SensorReadingsAttributes::ReservedData0:
            return "ReservedData0";
        case SensorReadingsAttributes::ReservedData1:
            return "ReservedData1";

  case SensorReadingsAttributes::Reserved0:
            return "Reserved0";
        case SensorReadingsAttributes::Reserved1:
            return "Reserved1";

        case SensorReadingsAttributes::AccelerationX:
            return "AccelerationX";
        case SensorReadingsAttributes::AccelerationY:
            return "AccelerationY";
        case SensorReadingsAttributes::AccelerationZ:
            return "AccelerationZ";
        case SensorReadingsAttributes::GyroX:
            return "GyroX";
        case SensorReadingsAttributes::GyroY:
            return "GyroY";
        case SensorReadingsAttributes::GyroZ:
            return "GyroZ";
        case SensorReadingsAttributes::MPU6500Temperature:
            return "MPU6500Temperature";
        default:
            return "Unknown";
    }
}

const char* ProcessedValuesToString(uint8_t index) {
    switch (static_cast<ProcessedValuesAttributes>(1<<index)) {
        case ProcessedValuesAttributes::SkinTemperature:
            return "SkinTemperature";
        case ProcessedValuesAttributes::ApparentTemperature:
            return "ApparentTemperature";
        case ProcessedValuesAttributes::HeatIndex:
            return "HeatIndex";
        case ProcessedValuesAttributes::Humidex:
            return "Humidex";
        case ProcessedValuesAttributes::CompensationMode:
            return "CompensationMode";

        case ProcessedValuesAttributes::Perspiration:
            return "Perspiration";
        case ProcessedValuesAttributes::OnOffBodyState:
            return "OnOffBodyState";

        case ProcessedValuesAttributes::Reserved0:
            return "Reserved0";
        case ProcessedValuesAttributes::Reserved1:
            return "Reserved1";
        case ProcessedValuesAttributes::Reserved2:
            return "Reserved2";
        case ProcessedValuesAttributes::Reserved3:
            return "Reserved3";
        case ProcessedValuesAttributes::Reserved4:
            return "Reserved4";
        case ProcessedValuesAttributes::Reserved5:
            return "Reserved5";
        case ProcessedValuesAttributes::Reserved6:
            return "Reserved6";
        case ProcessedValuesAttributes::Reserved7:
            return "Reserved7";
        default:
            return "Unknown";
    }
}

DataType SensorReadingDataType(uint8_t index) {
    switch (static_cast<SensorReadingsAttributes>(1<<index)) {
        case SensorReadingsAttributes::TemperatureUpper:
            return DataType::Int32;
        case SensorReadingsAttributes::HumidityUpper:
            return DataType::Int32;

        case SensorReadingsAttributes::TemperatureLower:
            return DataType::Int32;
        case SensorReadingsAttributes::HumidityLower:
            return DataType::Int32;

        case SensorReadingsAttributes::ReservedData0:
            return DataType::Int32;
        case SensorReadingsAttributes::ReservedData1:
            return DataType::Int32;

        case SensorReadingsAttributes::Reserved0:
            return DataType::Invalid;
        case SensorReadingsAttributes::Reserved1:
            return DataType::Invalid;

        case SensorReadingsAttributes::AccelerationX:
            return DataType::Int16;
        case SensorReadingsAttributes::AccelerationY:
            return DataType::Int16;
        case SensorReadingsAttributes::AccelerationZ:
            return DataType::Int16;
        case SensorReadingsAttributes::GyroX:
            return DataType::Int16;
        case SensorReadingsAttributes::GyroY:
            return DataType::Int16;
        case SensorReadingsAttributes::GyroZ:
            return DataType::Int16;
        case SensorReadingsAttributes::MPU6500Temperature:
            return DataType::Int16;
        default:
            return DataType::Invalid;
    }
}

DataType ProcessedValuesDataType(uint8_t index) {
    switch (static_cast<ProcessedValuesAttributes>(1<<index)) {
        case ProcessedValuesAttributes::SkinTemperature:
            return DataType::Float;
        case ProcessedValuesAttributes::ApparentTemperature:
            return DataType::Float;
        case ProcessedValuesAttributes::HeatIndex:
            return DataType::Float;
        case ProcessedValuesAttributes::Humidex:
            return DataType::Float;
        case ProcessedValuesAttributes::CompensationMode:
            return DataType::UInt8;

        case ProcessedValuesAttributes::Perspiration:
            return DataType::Float;
        case ProcessedValuesAttributes::OnOffBodyState:
            return DataType::UInt8;

        case ProcessedValuesAttributes::Reserved0:
            return DataType::Invalid;
        case ProcessedValuesAttributes::Reserved1:
            return DataType::Invalid;
        case ProcessedValuesAttributes::Reserved2:
            return DataType::Invalid;
        case ProcessedValuesAttributes::Reserved3:
            return DataType::Invalid;
        case ProcessedValuesAttributes::Reserved4:
            return DataType::Invalid;
        case ProcessedValuesAttributes::Reserved5:
            return DataType::Invalid;
        case ProcessedValuesAttributes::Reserved6:
            return DataType::Invalid;
        case ProcessedValuesAttributes::Reserved7:
            return DataType::Invalid;
        default:
            return DataType::Invalid;
    }
}

