/*
 * Driver for the W25Q128FW Flash chip
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

#include "FlashW25Q128FW.h"
#include <DebugPrint.h>

void FlashW25Q128FW::begin()
{
    DEBUG_PRINTLN("Initialising Flash memory");
    pinMode(chipSelect, OUTPUT);
    digitalWrite(chipSelect, HIGH);
}

uint32_t FlashW25Q128FW::getJEDECID()
{
    uint8_t buf[4] = {Instruction::JEDEC_ID, 0x00, 0x00, 0x00};
    writeBuffer(buf, sizeof(buf));
#ifdef DEBUG
    DEBUG_PRINT("JEDEC_ID: 0x");
    for (unsigned i = 0; i < sizeof(buf); ++i) {
        DEBUG_PRINT(buf[i], HEX);
    }
    DEBUG_PRINTLN();
#endif
    uint32_t jedecId = buf[1];
    jedecId = jedecId<<8 | buf[2];
    jedecId = jedecId<<8 | buf[3];
    return jedecId;
}


bool FlashW25Q128FW::eraseSector(uint16_t sector)
{
    // convert sector to page
    uint16_t page = sector<<4;
    uint8_t buffer[4] = {Instruction::SectorErase, uint8_t(page>>8), uint8_t(page), 0x00};
    writeBuffer(buffer, 4);
    return true;
}

void FlashW25Q128FW::readPage(uint16_t page, uint8_t (&buf)[PAGE_SIZE])
{
    uint8_t readcmd[4] = {Instruction::ReadData, uint8_t(page>>8), uint8_t(page), 0x00};
    digitalWrite(chipSelect, LOW);
    SPI.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE0));
    SPI.transfer(readcmd, 4);
    SPI.transfer(buf, PAGE_SIZE);
    SPI.endTransaction();
    digitalWrite(chipSelect, HIGH);
}

void FlashW25Q128FW::writePage(uint16_t page, uint8_t (&buf)[PAGE_SIZE])
{
    uint8_t writeCmd[4] = {Instruction::PageProgram, uint8_t(page>>8), uint8_t(page), 0x00};
    digitalWrite(chipSelect, LOW);
    SPI.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE0));
    SPI.transfer(writeCmd, 4);
    SPI.transfer(buf, PAGE_SIZE);
    SPI.endTransaction();
    digitalWrite(chipSelect, HIGH);
}

uint8_t FlashW25Q128FW::getStatusRegister1()
{
    uint8_t cmd[2] = {Instruction::ReadStatusRegister1, 0x00};
    writeBuffer(cmd, 2);
    return cmd[1];
}

void FlashW25Q128FW::writeBuffer(void *buf, size_t len)
{
    digitalWrite(chipSelect, LOW);
    SPI.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE0));
    SPI.transfer(buf, len);
    SPI.endTransaction();
    digitalWrite(chipSelect, HIGH);
}

void FlashW25Q128FW::writeInstruction(Instruction instruction)
{
    uint8_t inst = static_cast<uint8_t>(instruction);
    writeBuffer(&inst, 1);
}

