/*
 * Example how to use the SPI interface to talk to MPU6500
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

#include <SPI.h>
#include "MPU6500.h"


const size_t PACKET_LEN = 14; // number of bytes in data packet

const int N_CS_GYRO = 8;
const int N_CS_FLASH = 10;

unsigned short n_samples = 0;
unsigned char bytes[PACKET_LEN];

MPU6500 mpu6500(N_CS_GYRO);

void setup()
{
    n_samples = 0;

    digitalWrite(N_CS_FLASH, HIGH);
    pinMode(N_CS_FLASH, OUTPUT);
    digitalWrite(N_CS_FLASH, HIGH);
    SPI.begin();

    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }

    mpu6500.begin();
}

void loop()
{
    if(mpu6500.readFifo(bytes, PACKET_LEN)) {
        n_samples++;
    }

    // log every 10th sample to avoid filling the FIFO
    if(n_samples >= 10) {
        short temp = 0;
        short acc[3] = { 0, 0, 0 };
        short gyro[3] = { 0, 0, 0 };
        short i16;

        i16 = (bytes[0] << 8) + bytes[1];
        acc[0] = i16;

        i16 = (bytes[2] << 8) + bytes[3];
        acc[1] = i16;

        i16 = (bytes[4] << 8) + bytes[5];
        acc[2] = i16;

        i16 = (bytes[6] << 8) + bytes[7];
        temp = i16;

        i16 = (bytes[8] << 8) + bytes[9];
        gyro[0] = i16;

        i16 = (bytes[10] << 8) + bytes[11];
        gyro[1] = i16;

        i16 = (bytes[12] << 8) + bytes[13];
        gyro[2] = i16;

        uint16_t fifo_count = mpu6500.getFifoCount();

        Serial.print( acc[0] );
        Serial.print( "\t" );
        Serial.print( acc[1] );
        Serial.print( "\t" );
        Serial.print( acc[2] );
        Serial.print( "\t" );
        Serial.print( gyro[0] );
        Serial.print( "\t" );
        Serial.print( gyro[1] );
        Serial.print( "\t" );
        Serial.print( gyro[2] );
        Serial.print( "\t" );
        Serial.print( temp );
        Serial.print( "\t" );
        Serial.print( fifo_count );
        Serial.println( "" );

        for(int i = 0; i < 3; i++) {
            acc[i] = 0;
            gyro[i] = 0;
        }

        temp = 0;
        n_samples = 0;
    }
}

