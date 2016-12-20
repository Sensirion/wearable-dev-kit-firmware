/*
 * Hardware specific definition for the pebble backpack.
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

#include "Hardware.h"
#include "SPI.h"

#if defined(__SAMD21G18A__)

void initializeHardware()
{
    SPI.begin();
    Wire.begin();
    Wire1.begin();

    // disable deep sleep and set idle mode to 0 so that USB still works
    PM->SLEEP.bit.IDLE = 0;
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    // check chip revision (die revision)
    // Device ID register datasheet section 13.13.9
    // Bits 31:28 - PROCESSOR[3:0]: Processor
    // Bits 27:23 - FAMILY[4:0]: Product Family
    // Bits 21:16 - SERIES[5:0]: Product Series
    // Bits 15:12 - DIE[3:0]: Die Number
    // Bits 11:08 - REVISION[3:0]: Revision Number - 0x0=rev.A, 0x1=rev.B, ...
    if ((DSU->DID.reg & 0x0f00)>>8 < 0x03) {
        // Errata reference 13140
        // preventing CPU from wakeup and causes system freeze
        // affects devices with revision C and lower
        // fix increases sleep mode power consumption with about 20 uA
        NVMCTRL->CTRLB.bit.SLEEPPRM = 3;
    }
}

#warning "Sleep modes not completely implemented for SAMD21"
void disableUnusedPeripherals()
{
}

void sleepNow()
{
    __DSB();
    __WFI();
}

void disableUSB()
{
    // USB_SERIAL.end() should not be called! Because of whatever reason it
    // will use more current if we do
    //USB_SERIAL.end();

    USBDevice.detach();
    USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
    PM->AHBMASK.bit.USB_ = 0;
    // activate idle mode 2
    PM->SLEEP.bit.IDLE = 2;
}

void enableUSB()
{
    // disable deep sleep and set idle mode to 0 so that USB still works
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    PM->SLEEP.bit.IDLE = 0;

    PM->AHBMASK.bit.USB_ = 1;
    USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
    USBDevice.attach();
    USB_SERIAL.begin(115200);
}

#else

#warning "Unknown board, disabling sleep modes"
void disableUnusedPeripherals()
{
}

void sleepNow()
{
}

void disableUSB()
{
}

void enableUSB()
{
}

#endif

