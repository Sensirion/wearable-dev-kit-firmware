/*
 * Firmware for the Flintstone gadget.
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
#include <ArduinoPebbleSerial.h>
#include <PebbleBackpack.h>
#include <PebbleProtocol.h>
#include <Logger.h>
#include <version.h>

static const unsigned long SENSOR_POLL_INTERVAL_MS = 1000;
static const size_t MAX_PAYLOAD = 60;

static const uint16_t SUPPORTED_SERVICES[] = {
    static_cast<uint16_t>(Services::SensorReadings),
    static_cast<uint16_t>(Services::ProcessedValues),
    static_cast<uint16_t>(Services::Logger),
    static_cast<uint16_t>(Services::System),
};
static const uint8_t NUM_SERVICES = sizeof(SUPPORTED_SERVICES)/sizeof(SUPPORTED_SERVICES[0]);
static const size_t BUFFER_SIZE = GET_PAYLOAD_BUFFER_SIZE(MAX_PAYLOAD);
static uint8_t pebble_buffer[BUFFER_SIZE];
static PebbleBackpack pebble_backpack(SENSOR_POLL_INTERVAL_MS);

static bool usbEnabled = true;

void setup()
{
    disableUnusedPeripherals();

    USB_SERIAL.begin(115200);
#ifdef DEBUG
    while (!USB_SERIAL) {
        // wait for serial console
    }
    DEBUG_PRINTLN("Connected serial logger");
#endif
    pebble_backpack.begin();

    ArduinoPebbleSerial::begin_hardware(pebble_buffer, sizeof(pebble_buffer), Baud57600,
                                        SUPPORTED_SERVICES, NUM_SERVICES);
    DEBUG_PRINTLN("Pebble connection started");
}

bool handleServiceSensorReadings(RequestType type, uint16_t attribute_id, size_t *size)
{
    if (type != RequestTypeWriteRead && type != RequestTypeRead) {
        // invalid request type - just ignore the request
        DEBUG_PRINTLN("Got Invalid request on SensorReadings");
        return false;
    }

    DEBUG_PRINTLN("Got Read for SensorReadings");
    // the range 0x8000..0xFFFF is reserved
    if (attribute_id & 0x8000) {
        DEBUG_PRINTLN("Reserved attribute_id");
        return false;
    }

    // send an error if any reserved bit is set
    if (attribute_id & SensorReadingsAttributes::ReservedMask) {
        DEBUG_PRINTLN("Reserved attribute bit set!");
        return false;
    }

    for (int i = 0; i < MAX_SENSOR_READINGS; ++i) {
        if (attribute_id & (1<<i)) {
            void* value;
            size_t valueSize;
            if (pebble_backpack.getSensorValue(static_cast<SensorReadingsAttributes>(1<<i), &value, &valueSize)) {
                memcpy(pebble_buffer + *size, value, valueSize);
                *size += valueSize;
            } else {
                return false;
            }
        }
    }
    return true;
}

bool handleServiceProcessedValues(RequestType type, uint16_t attribute_id, size_t length, size_t *size)
{
    DEBUG_PRINTLN("Got Read for ProcessedValues");

    // Check if the first bit is set
    if (attribute_id & 0x8000) {
        switch (static_cast<ProcessedValuesAttributes>(attribute_id)) {
            case ProcessedValuesAttributes::TemperatureCompensationMode:
                if (type == RequestTypeWrite || type == RequestTypeWriteRead) {
                    if (length != 1) {
                        return false;
                    }
                    TemperatureCompensationMode mode = static_cast<TemperatureCompensationMode>(pebble_buffer[0]);
                    if (mode >= TemperatureCompensationMode::NumberOfModes) {
                        return false;
                    }
                    pebble_backpack.setTemperatureCompensationMode(mode);
                }
                if (type == RequestTypeRead || type == RequestTypeWriteRead) {
                    pebble_buffer[0] = static_cast<uint8_t>(pebble_backpack.getTemperatureCompensationMode());
                    pebble_buffer[1] = static_cast<uint8_t>(TemperatureCompensationMode::NumberOfModes);
                    *size = 2;
                }
                return true;
                break;

            default:
                DEBUG_PRINTLN("Reserved attribute_id");
                return false;
        }
    }

    // send an error if any reserved bit is set
    if (attribute_id & ProcessedValuesAttributes::ReservedMask) {
        DEBUG_PRINTLN("Reserved attribute bit set!");
        return false;
    }

    for (int i = 0; i < MAX_PROCESSED_VALUES; ++i) {
        if (attribute_id & (1<<i)) {
            void* value;
            size_t valueSize;
            if (pebble_backpack.getProcessedValue(static_cast<ProcessedValuesAttributes>(1<<i), &value, &valueSize)) {
                memcpy(pebble_buffer + *size, value, valueSize);
                *size += valueSize;
            } else {
                return false;
            }
        }
    }
    return true;
}

static LogHeader logHeader = {
    0,
    SENSOR_POLL_INTERVAL_MS,
    0,
};

bool handleServiceLogger(RequestType type, uint16_t attribute_id, size_t length, size_t *size)
{
    DEBUG_PRINTLN("Logger request");

    switch (static_cast<LoggerAttributes>(attribute_id)) {
        case LoggerAttributes::ClearLog:
            if (type != RequestTypeWrite) {
                DEBUG_PRINTLN("Invalid type");
                return false;
            }
            return logger.clearLog();

        case LoggerAttributes::StartLogging:
            {
                if (type != RequestTypeWrite) {
                    DEBUG_PRINTLN("Invalid type");
                    return false;
                }
                if (length != sizeof(logHeader)) {
                    DEBUG_PRINTLN("Invalid length");
                    return false;
                }
                readFromBuffer(0, &pebble_buffer[0], length, &logHeader);

                // disable not available channels
                const uint32_t available_channels = static_cast<uint32_t>(pebble_backpack.getAvailableSensorReadings()) |
                                                   (static_cast<uint32_t>(pebble_backpack.getAvailableProcessedValues()) << 16);
                const uint32_t filtered_channel_mask = logHeader.EnabledChannelMask & available_channels;
                if (filtered_channel_mask != logHeader.EnabledChannelMask) {
                    DEBUG_PRINT("Available channels: ");
                    DEBUG_PRINTLN(available_channels);
                    DEBUG_PRINT("EnabledChannelMask contains illegal channels, degrading from ");
                    DEBUG_PRINT(logHeader.EnabledChannelMask, HEX);
                    DEBUG_PRINT(" to ");
                    DEBUG_PRINTLN(filtered_channel_mask, HEX);
                    logHeader.EnabledChannelMask = filtered_channel_mask;
                }
                // overwrite Pebble transmitted logging interval
                // Pebble cannot know the used interval in the backpack
                logHeader.LogIntervalMs = SENSOR_POLL_INTERVAL_MS;
                return logger.startLogging(logHeader);
            }

        case LoggerAttributes::PauseLogging:
            if (type != RequestTypeWrite) {
                DEBUG_PRINTLN("Invalid type");
                return false;
            }
            return logger.pauseLogging();

        case LoggerAttributes::ResumeLogging:
            if (type != RequestTypeWrite) {
                DEBUG_PRINTLN("Invalid type");
                return false;
            }
            return logger.resumeLogging();

        case LoggerAttributes::State:
            if (type != RequestTypeRead) {
                DEBUG_PRINTLN("Invalid type");
                return false;
            }
            pebble_buffer[0] = static_cast<uint8_t>(logger.getState());
            *size = 1;
            return true;

        case LoggerAttributes::LogEntries:
            // TODO: Return number of log entries
        default:
            DEBUG_PRINTLN("Unknown attribute_id");
            return false;
    }
}

bool handleServiceSystem(RequestType type, uint16_t attribute_id, size_t, size_t *size)
{
    DEBUG_PRINTLN("System request");

    switch (static_cast<SystemAttributes>(attribute_id)) {
        case SystemAttributes::USBPlugged:
            if (type != RequestTypeWrite) {
                DEBUG_PRINTLN("Invalid type");
                return false;
            }
            DEBUG_PRINTLN("Enable USB?");
            if (!usbEnabled) {
                usbEnabled = true;
                enableUSB();
                DEBUG_PRINTLN("Enabled USB!");
            }
            return true;

        case SystemAttributes::USBUnPlugged:
            if (type != RequestTypeWrite) {
                DEBUG_PRINTLN("Invalid type");
                return false;
            }
            DEBUG_PRINTLN("Disable USB?");
            if (usbEnabled) {
                DEBUG_PRINTLN("Disabled USB!");
                usbEnabled = false;
                disableUSB();
            }
            return true;

        case SystemAttributes::Status:
            {
                if (type != RequestTypeRead) {
                    DEBUG_PRINTLN("Invalid type");
                    return false;
                }
                // TODO: Implement status correctly
                uint32_t system_status = 0;
                *size = putInBuffer(pebble_buffer, *size, system_status);
                return true;
            }

        case SystemAttributes::Version:
            static_assert(VERSION_LENGTH<MAX_PAYLOAD, "Version string too long");
            *size = VERSION_LENGTH;
            memcpy(pebble_buffer, VERSION, VERSION_LENGTH);
            return true;

        case SystemAttributes::AvailableSensorReadings:
            {
                uint16_t available_sensors = pebble_backpack.getAvailableSensorReadings();
                *size = putInBuffer(pebble_buffer, *size, available_sensors);
                return true;
            }
        case SystemAttributes::AvailableProcessedValues:
            {
                uint16_t available_processed_values = pebble_backpack.getAvailableProcessedValues();
                *size = putInBuffer(pebble_buffer, *size, available_processed_values);
                return true;
            }
    }
    DEBUG_PRINTLN("Unknown attribute_id");
    return false;
}

void loop()
{
    static uint8_t bytesPerLogEntry = 0;
    static bool logReadingActive = false;
    const int MAX_NUM_EVENTS = 2; // airtouch, onbody
    struct pebble_event {
        uint16_t service_id;
        uint16_t attribute_id;
    } event_stack[MAX_NUM_EVENTS];
    int event_stack_index = 0;
    bool received_pebble_request = false;

    if (USB_SERIAL.available() > 0) {
        char command = USB_SERIAL.read();
        DEBUG_PRINT("Received serial data: ");
        DEBUG_PRINTLN(command);

        if (command == 'h') {
            // read header
            bool success = logger.readHeader(&logHeader);
            if (success) {
                USB_SERIAL.write(reinterpret_cast<uint8_t*>(&logHeader), sizeof(logHeader));

                const uint16_t enabled_sensors = logHeader.EnabledChannelMask & 0xFFFF;
                const uint16_t enabled_processed_values = logHeader.EnabledChannelMask >> 16;
                // 4 for timestamp
                bytesPerLogEntry = 4;
                for (int i = 0; i < MAX_SENSOR_READINGS; ++i) {
                    if (enabled_sensors & (1<<i)) {
                        SensorReadingsAttributes attribute = static_cast<SensorReadingsAttributes>(1<<i);
                        void* value;
                        size_t size;
                        if (pebble_backpack.getSensorValue(attribute, &value, &size)) {
                            bytesPerLogEntry += size;
                        }
                    }
                }

                for (int i = 0; i < MAX_PROCESSED_VALUES; ++i) {
                    if (enabled_processed_values & (1<<i)) {
                        ProcessedValuesAttributes attribute = static_cast<ProcessedValuesAttributes>(1<<i);
                        void* value;
                        size_t size;
                        if (pebble_backpack.getProcessedValue(attribute, &value, &size)) {
                            bytesPerLogEntry += size;
                        }
                    }
                }

                DEBUG_PRINT("Header contains: ");
                DEBUG_PRINTLN((uint32_t)logHeader.StartTimeMs);
                DEBUG_PRINTLN(logHeader.LogIntervalMs);
                DEBUG_PRINTLN(logHeader.EnabledChannelMask);
                DEBUG_PRINTLN(bytesPerLogEntry);
                logReadingActive = true;
            }
        } else if (command == 'r') {
            // read log data
            uint8_t buffer[256];
            bool success = logger.readValue(buffer, bytesPerLogEntry);

            if (success) {
                USB_SERIAL.write(buffer, bytesPerLogEntry);
            }
        } else if (command == 'g') {
            // GetState
            USB_SERIAL.write(static_cast<uint8_t>(logger.getState()));
        } else if (command == 'q') {
            // QuitLogReading
            logger.readFinished();
            logReadingActive = false;
        }
    }

    // don't do anything while reading the log
    if (logReadingActive) {
        return;
    }

    static bool pebble_is_connected = false;

    if (pebble_backpack.readSensors()) {
        if (logger.getState() == Logger::State::Writing) {

            // first entry is always the timestamp
            uint32_t timestamp = pebble_backpack.getTimestamp();
            logger.logValue(&timestamp, sizeof(timestamp));

            const uint16_t enabled_sensors = logHeader.EnabledChannelMask & 0xFFFF;
            const uint16_t enabled_processed_values = logHeader.EnabledChannelMask >> 16;
            for (int i = 0; i < MAX_SENSOR_READINGS; ++i) {
                if (enabled_sensors & (1<<i)) {
                    SensorReadingsAttributes attribute = static_cast<SensorReadingsAttributes>(1<<i);
                    void* value;
                    size_t size;
                    if (pebble_backpack.getSensorValue(attribute, &value, &size)) {
                        logger.logValue(value, size);
                    }
                }
            }
            for (int i = 0; i < MAX_PROCESSED_VALUES; ++i) {
                if (enabled_processed_values & (1<<i)) {
                    ProcessedValuesAttributes attribute = static_cast<ProcessedValuesAttributes>(1<<i);
                    void* value;
                    size_t size;
                    if (pebble_backpack.getProcessedValue(attribute, &value, &size)) {
                        logger.logValue(value, size);
                    }
                }
            }
        }

        if (pebble_is_connected) {
            // handle AirTouch
            switch (pebble_backpack.getAirTouchEvent()) {
                case AirTouchEvent::Start:
                    DEBUG_PRINT(millis())
                    DEBUG_PRINTLN(" AirTouch start");
                    event_stack[event_stack_index++] = {
                        static_cast<uint16_t>(Services::ProcessedValues),
                        static_cast<uint16_t>(ProcessedValuesAttributes::AirTouchStartEvent)
                    };
                    break;
                case AirTouchEvent::Stop:
                    DEBUG_PRINT(millis())
                    DEBUG_PRINTLN(" AirTouch stop");
                    event_stack[event_stack_index++] = {
                        static_cast<uint16_t>(Services::ProcessedValues),
                        static_cast<uint16_t>(ProcessedValuesAttributes::AirTouchStopEvent)
                    };
                    break;
                default:
                    break;
            }
            // handle On-/Off-Body events
            static OnOffBodyState previous_state = OnOffBodyState::OffBody;
            OnOffBodyState current_state = pebble_backpack.getOnOffBodyState();

            if (previous_state == OnOffBodyState::OffBody &&
                current_state == OnOffBodyState::OnBody) {
                DEBUG_PRINT(millis());
                DEBUG_PRINTLN(" On Body event");
                event_stack[event_stack_index++] = {
                    static_cast<uint16_t>(Services::ProcessedValues),
                    static_cast<uint16_t>(ProcessedValuesAttributes::OnBodyEvent)
                };
            } else if (previous_state == OnOffBodyState::OnBody &&
                       current_state == OnOffBodyState::OffBody) {
                DEBUG_PRINT(millis());
                DEBUG_PRINTLN(" Off Body event");
                event_stack[event_stack_index++] = {
                    static_cast<uint16_t>(Services::ProcessedValues),
                    static_cast<uint16_t>(ProcessedValuesAttributes::OffBodyEvent)
                };
            }
            previous_state = current_state;
        }
    }

    // Let the ArduinoPebbleSerial code do its processing
    size_t length;
    uint16_t service_id;
    uint16_t attribute_id;
    RequestType type;
    // Loop through incoming polling requests while handling events
    // since requests must be handled before events
    do {
        if (ArduinoPebbleSerial::feed(&service_id, &attribute_id, &length, &type)) {
            received_pebble_request = true;
            DEBUG_PRINT("Pebble message: ");
            DEBUG_PRINT("0x");
            DEBUG_PRINT(service_id, HEX);
            DEBUG_PRINT(":0x");
            DEBUG_PRINT(attribute_id, HEX);
            DEBUG_PRINT("  type: ");
            DEBUG_PRINT(type);
            DEBUG_PRINT("  length: ");
            DEBUG_PRINTLN(length);

            bool success = false;
            size_t size = 0;
            switch (static_cast<Services>(service_id)) {
                case Services::SensorReadings:
                    success = handleServiceSensorReadings(type, attribute_id, &size);
                    break;
                case Services::ProcessedValues:
                    success = handleServiceProcessedValues(type, attribute_id, length, &size);
                    break;
                case Services::Logger:
                    success = handleServiceLogger(type, attribute_id, length, &size);
                    break;
                case Services::System:
                    success = handleServiceSystem(type, attribute_id, length, &size);
                    break;
                default:
                    DEBUG_PRINTLN("Unsupported service");
                    // unsupported attribute - fail the request
                    break;
            }
            ArduinoPebbleSerial::write(success, pebble_buffer, size);
        }
        if (event_stack_index > 0) {
            const struct pebble_event *event = &event_stack[--event_stack_index];
            DEBUG_PRINT("Pebble event: ");
            DEBUG_PRINT("0x");
            DEBUG_PRINT(event->service_id, HEX);
            DEBUG_PRINT(":0x");
            DEBUG_PRINT(event->attribute_id, HEX);
            DEBUG_PRINTLN("");
            ArduinoPebbleSerial::notify(event->service_id, event->attribute_id);
        }
    } while (event_stack_index > 0);
    if (!received_pebble_request)
        sleepNow();

    // log connection changes
    if (ArduinoPebbleSerial::is_connected()) {
        if (!pebble_is_connected) {
            DEBUG_PRINTLN("Connected to the smartstrap!");
            pebble_is_connected = true;
        }
    } else if (pebble_is_connected) {
        DEBUG_PRINTLN("Disconnected from the smartstrap!");
        pebble_is_connected = false;
    }
}
