/*
 * Logger that writes measured values to the Flash
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

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <FlashW25Q128FW.h>
#include <PebbleProtocol.h>

using LogChannelRawFlags = SensorReadingsAttributes;
using LogChannelProccesed = ProcessedValuesAttributes;

/**
 * Log header like it gets written to the flash
 */
struct LogHeader {
    /**
     * Timestamp in [ms] since epoch
     */
    uint64_t StartTimeMs;
    /**
     * Time between log entries in [ms]
     */
    uint32_t LogIntervalMs;
    /**
     * Bitmask with enabled channels corresponding to the
     * SensorReadingsAttributes and ProcessedValuesAttributes
     * - the first 16 bits correspond to SensorReadingsAttributes
     * - the second 16 bits correspond to ProcessedValuesAttributes
     */
    uint32_t EnabledChannelMask;
};


class Logger {
public:
    /**
     *          Check at start if empty or dirty
     *     +-----------+                +-----------+
     *     |                                        |
     *     |                                        |
     *  +--v----+        +---------+           +----v--+
     *  |       |        |         |  clearLog |       |
     *  | Empty <--------+ Erasing <-----------+ Dirty <-------+
     *  |       |        |         |           |       |       |
     *  +---+---+        +---------+           ++-^----+       |
     *      |                                   | |            |
     *      |startLogging             readHeader| |            |
     *      |                                   | |            |
     *  +---v-----+ pauseLogging     +--------+ | |            |
     *  |         +------------------>        | | |            |
     *  | Writing |                  | Paused | | |readFinished|
     *  |         <------------------+        | | |            |
     *  +---+-----+    resumeLogging +-----+--+ | |            |
     *      |                              |    | |            |
     *      |                              |    | |            |
     *      |                    readHeader|    | |            |
     *  +---v-----+                     +--v----v-+-+    +-----+--------+
     *  |         |          readHeader |           |    |              |
     *  | LogFull +---------------------> Reading   +----> ReadFinished |
     *  |         |                     |           |    |              |
     *  +---------+                     +-----------+    +--------------+
     */
    enum struct State: uint8_t {
        Empty,
        Dirty,
        Erasing,
        Writing,
        WritingPaused,
        LogFull, // log full
        Reading,
        ReadFinished, // everything read
    };
    static const uint16_t  PAGE_SIZE = FlashW25Q128FW::PAGE_SIZE;
    Logger():
        _pageBuffer(),
        _pageIndex(0),
        _currentPage(0),
        _state(State::Dirty)
    {}

    /**
     * Initialize logger and check state
     */
    void begin();

    /**
     * Erease the whole memory
     * @return True on success
     */
    bool clearLog();

    /**
     * Start logging by writing the header
     * @return True on success
     */
    bool startLogging(const LogHeader& logHeader);

    /**
     * Temporarily disable logging
     */
    bool pauseLogging();

    /**
     * Continue logging
     */
    bool resumeLogging();

    /**
     * Log a single value
     * @return True on success
     */
    bool logValue(const void* value, size_t size);

    /**
     * Read the header
     * @return True on success
     */
    bool readHeader(LogHeader* header);

    /**
     * Read a single value from the log
     * @return True on success
     */
    bool readValue(void* value, size_t size);

    template<typename T>
    bool readValue(T* value)
    { return readValue( (void*)value, sizeof(T)); }

    /**
     * Switch to State::Dirty from State::Reading or State::ReadFinished,
     * indicating that we are done reading.
     * @return True on success
     */
    bool readFinished();

    State getState();


private:
    uint8_t _pageBuffer[PAGE_SIZE];
    uint16_t _pageIndex;
    uint16_t _currentPage;
    // true when the log is fully read or written
    State _state;
};

extern Logger logger;

#endif // LOGGER_H
