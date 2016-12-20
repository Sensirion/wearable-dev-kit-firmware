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

#ifndef _MPU6500_H_
#define _MPU6500_H_

#include <Arduino.h>
#include <SPI.h>

class MPU6500 {
public:
    /**
     * Register definition as used in the datasheet
     */
    enum struct Register : uint8_t {
        SELF_TEST_X_GYRO  = 0x00, // R/W XG_ST_DATA[7:0]
        SELF_TEST_Y_GYRO  = 0x01, // R/W YG_ST_DATA[7:0]
        SELF_TEST_Z_GYRO  = 0x02, // R/W ZG_ST_DATA[7:0]
        SELF_TEST_X_ACCEL = 0x0D, // R/W XA_ST_DATA [7:0]
        SELF_TEST_Y_ACCEL = 0x0E, // R/W YA_ST_DATA [7:0]
        SELF_TEST_Z_ACCEL = 0x0F, //R/W ZA_ST_DATA [7:0]

        XG_OFFSET_H = 0x13, //[15:0] XG_OFFS_USR
        XG_OFFSET_L = 0x14,
        YG_OFFSET_H = 0x15, //[15:0] YG_OFFS_USR
        YG_OFFSET_L = 0x16,
        ZG_OFFSET_H = 0x17, //[15:0] ZG_OFFS_USR
        ZG_OFFSET_L = 0x18,
        SMPLRT_DIV  = 0x19,

        CONFIG        = 0x1A,
        GYRO_CONFIG   = 0x1B,
        ACCEL_CONFIG  = 0x1C,
        ACCEL_CONFIG2 = 0x1D,
        LP_ACCEL_ODR  = 0x1E,
        WOM_THR       = 0x1F,

        FIFO_EN = 0x23,
        I2C_MST_CTRL = 0x24,
        I2C_SLV0_ADDR = 0x25,
        I2C_SLV0_REG = 0x26,
        I2C_SLV0_CTRL = 0x27,
        I2C_SLV1_ADDR = 0x28,
        I2C_SLV1_REG = 0x29,
        I2C_SLV1_CTRL = 0x2A,
        I2C_SLV2_ADDR = 0x2B,
        I2C_SLV2_REG = 0x2C,
        I2C_SLV2_CTRL = 0x2D,
        I2C_SLV3_ADDR = 0x2E,
        I2C_SLV3_REG = 0x2F,
        I2C_SLV3_CTRL = 0x30,
        I2C_SLV4_ADDR = 0x31,
        I2C_SLV4_REG = 0x32,
        I2C_SLV4_DO = 0x33,
        I2C_SLV4_CTRL = 0x34,
        I2C_SLV4_DI = 0x35,
        I2C_MST_STATUS = 0x36,
        INT_PIN_CFG = 0x37,
        INT_ENABLE = 0x38,

        DMP_INT_STATUS = 0x39, // not in datasheet

        INT_STATUS = 0x3A,
        ACCEL_XOUT_H = 0x3B,
        ACCEL_XOUT_L = 0x3C,
        ACCEL_YOUT_H = 0x3D,
        ACCEL_YOUT_L = 0x3E,
        ACCEL_ZOUT_H = 0x3F,
        ACCEL_ZOUT_L = 0x40,
        TEMP_OUT_H = 0x41,
        TEMP_OUT_L = 0x42,
        GYRO_XOUT_H = 0x43,
        GYRO_XOUT_L = 0x44,
        GYRO_YOUT_H = 0x45,
        GYRO_YOUT_L = 0x46,
        GYRO_ZOUT_H = 0x47,
        GYRO_ZOUT_L = 0x48,
        EXT_SENS_DATA_00 = 0x49,
        EXT_SENS_DATA_01 = 0x4A,
        EXT_SENS_DATA_02 = 0x4B,
        EXT_SENS_DATA_03 = 0x4C,
        EXT_SENS_DATA_04 = 0x4D,
        EXT_SENS_DATA_05 = 0x4E,
        EXT_SENS_DATA_06 = 0x4F,
        EXT_SENS_DATA_07 = 0x50,
        EXT_SENS_DATA_08 = 0x51,
        EXT_SENS_DATA_09 = 0x52,
        EXT_SENS_DATA_10 = 0x53,
        EXT_SENS_DATA_11 = 0x54,
        EXT_SENS_DATA_12 = 0x55,
        EXT_SENS_DATA_13 = 0x56,
        EXT_SENS_DATA_14 = 0x57,
        EXT_SENS_DATA_15 = 0x58,
        EXT_SENS_DATA_16 = 0x59,
        EXT_SENS_DATA_17 = 0x5A,
        EXT_SENS_DATA_18 = 0x5B,
        EXT_SENS_DATA_19 = 0x5C,
        EXT_SENS_DATA_20 = 0x5D,
        EXT_SENS_DATA_21 = 0x5E,
        EXT_SENS_DATA_22 = 0x5F,
        EXT_SENS_DATA_23 = 0x60,

        MOT_DETECT_STATUS = 0x61, // not in datasheet

        I2C_SLV0_DO = 0x63,
        I2C_SLV1_DO = 0x64,
        I2C_SLV2_DO = 0x65,
        I2C_SLV3_DO = 0x66,
        I2C_MST_DELAY_CTRL = 0x67,
        SIGNAL_PATH_RESET = 0x68,
        MOT_DETECT_CTRL = 0x69,
        USER_CTRL = 0x6A,
        PWR_MGMT_1 = 0x6B,
        PWR_MGMT_2 = 0x6C,

        // not in datasheet
        BANK_SEL = 0x6D,
        MEM_START_ADDR = 0x6E,
        MEM_R_W = 0x6F,
        DMP_CFG_1 = 0x70,
        DMP_CFG_2 = 0x71,

        FIFO_COUNT_H = 0x72,
        FIFO_COUNT_L = 0x73,
        FIFO_R_W = 0x74,
        WHO_AM_I = 0x75,

        XA_OFFSET_H = 0x77,
        XA_OFFSET_L = 0x78,
        YA_OFFSET_H = 0x7A,
        YA_OFFSET_L = 0x7B,
        ZA_OFFSET_H = 0x7C,
        ZA_OFFSET_L = 0x7E,
    };

    struct Fifo {
        enum Enum : uint8_t {
            EN_ACC   = 0x08,
            EN_GYROZ = 0x10,
            EN_GYROY = 0x20,
            EN_GYROX = 0x40,
            EN_GYRO  = 0x70,
            EN_TEMP  = 0x80,
        };
    };

    static const unsigned DMP_MEMORY_BANKS = 8;
    static const unsigned DMP_MEMORY_BANK_SIZE = 256;
    static const unsigned DMP_MEMORY_CHUNK_SIZE = 16;

    // 1MHz
    static const uint32_t SPI_MHZ = 1000000;
    // Low SPI speed for reading/writing configuration registers
    static const uint32_t SPI_LOW_SPEED = 1 * SPI_MHZ;
    // High SPI speed for reading measurement data
    static const uint32_t SPI_HIGH_SPEED = 20 * SPI_MHZ;

    MPU6500(int nCs):
        _nCs(nCs),
        _spiSpeed(SPI_LOW_SPEED)
    {}

    /**
     * Configure CS and read ID register
     */
    void begin();

    /**
     * @param sampleRateDivider The 1kHz sample rate will be divided by
     *                          1+sampleRateDivider
     */
    void startSampling(uint8_t sampleRateDivider = 0);

    /**
     * Set the device to sleep mode.
     *
     * The MPU will be set to sleep mode where Gyro, Accell and DMP are off.
     * See https://store.invensense.com/datasheets/invensense/MPU_6500_Rev1.0.pdf
     * section 4.19 "Standard Power Modes" for more information.
     *
     * Use begin() to get out of sleep mode.
     */
    void setSleepMode();

    /**
     * Return the number of bytes in the FIFO
     */
    uint16_t getFifoCount();
    /**
     * Read count bytes from the FIFO if available
     */
    bool readFifo(uint8_t bytes[], size_t count);

    uint8_t readRegister(Register reg);
    void writeRegister(Register reg, uint8_t value);

private:
    int _nCs; ///< Chip select pin
    uint32_t _spiSpeed; /// SPI speed in Hz
};

#endif /* _MPU6500_H_ */
