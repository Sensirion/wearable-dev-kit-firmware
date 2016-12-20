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

#ifdef DEBUG_LOGGER
    #ifndef DEBUG
    #define DEBUG
    #endif
#else
    #ifdef DEBUG
    #undef DEBUG
    #endif
#endif

#include "Logger.h"
#include "PebbleBackpack.h"

Logger logger;

void Logger::begin()
{
    flash.begin();
    // check if log is empty
    flash.readPage(0, _pageBuffer);
    _state = State::Empty;
    for (size_t i = 0; i < PAGE_SIZE; ++i) {
        if (_pageBuffer[i] != 0xFF) {
            _state = State::Dirty;
            break;
        }
    }
}

bool Logger::clearLog()
{
    if (_state == State::Empty) {
        return true;
    }
    if (_state != State::Dirty) {
        return false;
    }
    DEBUG_PRINTLN("clearLog");
    flash._writeEnable();
    flash.eraseChip();
    _state = State::Erasing;
    return true;
}

bool Logger::startLogging(const LogHeader& logHeader)
{
    DEBUG_PRINTLN("startLogging: ");
    DEBUG_PRINT("startTime: ");
    DEBUG_PRINTLN((uint32_t)logHeader.StartTimeMs);
    DEBUG_PRINT("logInterval: ");
    DEBUG_PRINTLN(logHeader.LogIntervalMs);
    DEBUG_PRINT("enabledChannels: ");
    DEBUG_PRINTLN(logHeader.EnabledChannelMask, HEX);

    if (getState() != State::Empty) {
        return false;
    }

    _pageIndex = 0;
    _currentPage = 0;
    _state = State::Writing;
    return logValue(&logHeader, sizeof(LogHeader));
}

bool Logger::pauseLogging()
{
    if (_state != State::Writing) {
        return false;
    }
    _state = State::WritingPaused;
    return true;
}

bool Logger::resumeLogging()
{
    if (_state != State::WritingPaused) {
        return false;
    }
    _state = State::Writing;
    return true;
}

bool Logger::logValue(const void* value, size_t size)
{
    DEBUG_PRINT("logValue, pageIndex: ");
    DEBUG_PRINTLN(_pageIndex);
    if (_state != State::Writing) {
        return false;
    }
    if (_pageIndex + size > PAGE_SIZE) {
        // write remaining bytes to buffer
        size_t remainingBytes = PAGE_SIZE - _pageIndex;
        memcpy(&_pageBuffer[_pageIndex], value, remainingBytes);

        while (flash.isBusy()) {
            // wait for flash to get idle
        }
        DEBUG_PRINTLN("  writePage");
        flash._writeEnable();
        flash.writePage(_currentPage, _pageBuffer);

        // check if there is a next page
        if (_currentPage >= (FlashW25Q128FW::PAGES - 1)) {
            _state = State::LogFull;
            return false;
        }

        _currentPage++;

        _pageIndex = 0;
        memcpy(&_pageBuffer[_pageIndex], ((uint8_t*)value) + remainingBytes, size - remainingBytes);
        _pageIndex += size - remainingBytes;
    } else {
        memcpy(&_pageBuffer[_pageIndex], value, size);
        _pageIndex += size;
    }
    return true;
}

bool Logger::readHeader(LogHeader* header)
{
    DEBUG_PRINTLN("readHeader");
    if (_state != State::Dirty && _state != State::Reading &&
        _state != State::WritingPaused) {
        return false;
    }
    _pageIndex = 0;
    _currentPage = 0;
    _state = State::Reading;
    flash.readPage(_currentPage, _pageBuffer);
    _currentPage++;
    readValue(header, sizeof(LogHeader));
    return true;
}

bool Logger::readValue(void* value, size_t size)
{
    DEBUG_PRINT("readValue, pageIndex: ");
    DEBUG_PRINTLN(_pageIndex);

    if (_state != State::Reading) {
        return false;
    }

    if (_pageIndex + size > PAGE_SIZE) {
        // read remaining bytes from buffer
        size_t remainingBytes = PAGE_SIZE - _pageIndex;
        memcpy(value, &_pageBuffer[_pageIndex], remainingBytes);

        // read next page
        while(flash.isBusy()) {
            // wait for flash to get idle
        }
        DEBUG_PRINTLN("  readPage");
        flash.readPage(_currentPage, _pageBuffer);

        // check if there is a next page
        if (_currentPage >= (FlashW25Q128FW::PAGES - 1)) {
            _state = State::ReadFinished;
            return false;
        }

        _currentPage++;

        _pageIndex = 0;
        memcpy(((uint8_t*)value) + remainingBytes, &_pageBuffer[_pageIndex], size - remainingBytes);
        _pageIndex += size - remainingBytes;
    } else {
        memcpy(value, &_pageBuffer[_pageIndex], size);
        _pageIndex += size;
    }
    return true;
}

bool Logger::readFinished()
{
    if (_state != State::Reading && _state != State::ReadFinished) {
        return false;
    }
    _state = State::Dirty;
    return true;
}

Logger::State Logger::getState()
{
    // check if erasing was finished
    if (_state == State::Erasing && !flash.isBusy()) {
        _state = State::Empty;
    }
    return _state;
}
