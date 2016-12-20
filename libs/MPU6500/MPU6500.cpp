/*
 * MPU6500 Driver
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

#include "MPU6500.h"

// allow to override debug output destination
#ifndef DEBUG_PRINTER
#define DEBUG_PRINTER SerialUSB
#endif

// allow to enable/disable debugging
#ifdef MPU6500_DEBUG
    #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
    #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
    #define DEBUG_PRINT(...) {}
    #define DEBUG_PRINTLN(...) {}
#endif


void MPU6500::begin()
{
    DEBUG_PRINTLN("begin mpu6500 spi");

    digitalWrite(_nCs, HIGH);
    pinMode(_nCs, OUTPUT);
    digitalWrite(_nCs, HIGH);
    _spiSpeed = SPI_LOW_SPEED;

    /*
     *  the I2C interface should be disabled by setting the I2C_IF_DIS
     *  configuration bit.  Setting this bit should be performed immediately
     *  after waiting for the time specified by the "Start Up Time for Register
     *  Read/Write" in Section 6.3.  For further information regarding the
     *  I2C_IF_DIS bit, please refer to the MPU-6500 Register Map and Register
     *  Descriptions document.
     */

    // reset the device
    writeRegister(MPU6500::Register::PWR_MGMT_1, 0x80);
    delay(100); // page 42 - delay 100ms

    // reset gyro, accel, temp
    writeRegister(Register::SIGNAL_PATH_RESET, 0x05);
    delay(100); // page 42 - delay 100ms

    // set SPI mode by setting I2C_IF_DIS
    // reset DMP, FIFO, SIG
    writeRegister(Register::USER_CTRL, 0x10 | 0x8 | 0x4 | 0x1);

    uint8_t id = readRegister(Register::WHO_AM_I);
    DEBUG_PRINT("id should be 0x70 or 0x71: ");
    DEBUG_PRINTLN(id, HEX);
}

void MPU6500::startSampling(uint8_t sampleRateDivider)
{
    DEBUG_PRINTLN("MPU start sampling");

    // Set DLPF_CFG to 1: 1kHz Gyro sampling, 184Hz bandwidth
    writeRegister(Register::CONFIG, 0x01);

    // Default: 1kHz Accel sampling, 480Hz cutoff

    // enable temperature, gyro, and accelerometer output
    writeRegister(Register::FIFO_EN,
            Fifo::EN_ACC | Fifo::EN_TEMP | Fifo::EN_GYRO);

    writeRegister(Register::SMPLRT_DIV, sampleRateDivider);

    // enable FIFO
    // set SPI mode by setting I2C_IF_DIS
    writeRegister(Register::USER_CTRL, 0x40 | 0x10);

    // after setup we can use speeds up to 20MHz
    _spiSpeed = SPI_HIGH_SPEED;
}

void MPU6500::setSleepMode()
{
    _spiSpeed = SPI_LOW_SPEED;
    uint8_t pwr_mgmt_1 = readRegister(Register::PWR_MGMT_1);
    pwr_mgmt_1 |= 1 << 6;
    writeRegister(Register::PWR_MGMT_1, pwr_mgmt_1);
}

uint16_t MPU6500::getFifoCount()
{
    uint8_t high = readRegister(Register::FIFO_COUNT_H);
    uint8_t low = readRegister(Register::FIFO_COUNT_L);
    uint16_t fifoCount = (high << 8) + low;
    return fifoCount;
}

bool MPU6500::readFifo(uint8_t bytes[], size_t count)
{
    uint16_t fifoCount = getFifoCount();

    // not enough data available
    if( fifoCount < count ) {
        return false;
    }

    digitalWrite(_nCs, LOW);
    SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
    SPI.transfer(static_cast<uint8_t>(Register::FIFO_R_W) | 0x80);
    for( size_t i = 0; i < count; i++ ) {
        bytes[i] = SPI.transfer(0x00);
    }
    SPI.endTransaction();
    digitalWrite(_nCs, HIGH);

    // DEBUGGING check whether we are reading what we are supposed to.
#ifdef MPU6500_DEBUG
    uint16_t fifoCount2 = getFifoCount();
    if (fifoCount - fifoCount2 != count) {
        DEBUG_PRINTLN("FIFO error!");
    }
#endif
    return true;
}

uint8_t MPU6500::readRegister(Register reg_)
{
    uint8_t reg = static_cast<uint8_t>(reg_);
    digitalWrite(_nCs, LOW);
    SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
    SPI.transfer(reg | 0x80); // reg | 0x80 to denote read
    uint8_t read_value = SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(_nCs, HIGH);
    return read_value;
}

void MPU6500::writeRegister(Register reg_, uint8_t value)
{
    uint8_t reg = static_cast<uint8_t>(reg_);
    digitalWrite(_nCs, LOW);
    SPI.beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
    SPI.transfer(reg);
    SPI.transfer(value);
    SPI.endTransaction();
    digitalWrite(_nCs, HIGH);
}
