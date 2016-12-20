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

#ifndef FLASHW25Q128FW_H
#define FLASHW25Q128FW_H

#include <Arduino.h>
#include <SPI.h>

class FlashW25Q128FW {
public:
    static const uint16_t PAGE_SIZE = 256;
    static const uint8_t  PAGES_PER_SECTOR = 16;
    static const uint16_t SECTORS = 4096;
    static const uint32_t PAGES = static_cast<uint32_t>(PAGES_PER_SECTOR) * SECTORS;

    enum Instruction : uint8_t {
        WriteEnable = 0x06,
        VolatileSRWriteEnable = 0x50,
        WriteDisable = 0x04,
        ReadStatusRegister1 = 0x05,
        WriteStatusRegister1 = 0x01,
        ReadStatusRegister2 = 0x35,
        WriteStatusRegister2 = 0x31,
        ReadStatusRegister3 = 0x15,
        WriteStatusRegister3 = 0x11,
        ChipErase = 0xC7,
        EraseProgramSuspend = 0x75,
        EraseProgramResume = 0x7A,
        PowerDown = 0xB9,
        ReleasePowerDownID = 0xAB,
        ManufacturerDeviceID = 0x90,
        JEDEC_ID = 0x9F,
        GlobalBlockLock = 0x7E,
        GlobalBlockUnlock = 0x98,
        EnterQPIMode = 0x38,
        EnableReset = 0x66,
        ResetDevice = 0x99,
        ReadUniqueID = 0x4B,
        PageProgram = 0x02,

        SectorErase = 0x20,
        BlockErase32KB = 0x52,
        BlockErase64KB = 0xD8,

        ReadData = 0x03,
        FastRead = 0x0B,
    };

    enum StatusFlag : uint8_t {
        BUSY = 1<<0,
        WEL  = 1<<1,
        BP0  = 1<<2,
        BP1  = 1<<3,
        BP2  = 1<<4,
        TB   = 1<<5,
        SEC  = 1<<6,
        SRP0 = 1<<7,
    };

    FlashW25Q128FW(int chipSelect):
        chipSelect(chipSelect),
        speed(1000000)
    {}

    /**
     * Disable the chip select and initialize
     */
    void begin();
    uint32_t getJEDECID();
    void writeBuffer(void* buf, size_t len);
    void writeInstruction(Instruction instruction);

    bool eraseSector(uint16_t sector);

    void eraseChip()
    { writeInstruction(Instruction::ChipErase); }

    void readPage(uint16_t page, uint8_t (&buf)[PAGE_SIZE]);

    void writePage(uint16_t page, uint8_t (&buf)[PAGE_SIZE]);

    uint8_t getStatusRegister1();

    bool isBusy()
    { return getStatusRegister1() & StatusFlag::BUSY; }

    void _writeEnable()
    { writeInstruction(Instruction::WriteEnable); }
    void _writeDisable()
    { writeInstruction(Instruction::WriteDisable); }

private:
    int chipSelect;
    uint32_t speed;
};

#endif // FLASHW25Q128FW_H
