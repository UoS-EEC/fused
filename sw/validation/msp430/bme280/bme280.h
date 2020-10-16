/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// Commands
static const unsigned BME280_R_REG = 0b10000000;
static const unsigned BME280_W_REG = 0b01111111;
static const unsigned BME280_NOP = 0b11111111;
static const unsigned BME280_RESET_WORD = 0xB6;

// Register addresses
// Data registers
static const unsigned BME280_HUM_LSB = 0xfe;
static const unsigned BME280_HUM_MSB = 0xfd;
static const unsigned BME280_TEMP_XLSB = 0xfc;
static const unsigned BME280_TEMP_LSB = 0xfb;
static const unsigned BME280_TEMP_MSB = 0xfa;
static const unsigned BME280_PRESS_XLSB = 0xf9;
static const unsigned BME280_PRESS_LSB = 0xf8;
static const unsigned BME280_PRESS_MSB = 0xf7;

// Control & status registers
static const unsigned BME280_CONFIG = 0xf5;
static const unsigned BME280_CTRL_MEAS = 0xf4;
static const unsigned BME280_STATUS = 0xf3;
static const unsigned BME280_CTRL_HUM = 0xf2;
static const unsigned BME280_RESET = 0xe0;
static const unsigned BME280_ID = 0xd0;

// Calibration registers
static const unsigned BME280_CALIB_00 = 0x88;
static const unsigned BME280_CALIB_01 = 0x89;
static const unsigned BME280_CALIB_02 = 0x8a;
static const unsigned BME280_CALIB_03 = 0x8b;
static const unsigned BME280_CALIB_04 = 0x8c;
static const unsigned BME280_CALIB_05 = 0x8d;
static const unsigned BME280_CALIB_06 = 0x8e;
static const unsigned BME280_CALIB_07 = 0x8f;
static const unsigned BME280_CALIB_08 = 0x90;
static const unsigned BME280_CALIB_09 = 0x91;
static const unsigned BME280_CALIB_10 = 0x92;
static const unsigned BME280_CALIB_11 = 0x93;
static const unsigned BME280_CALIB_12 = 0x94;
static const unsigned BME280_CALIB_13 = 0x95;
static const unsigned BME280_CALIB_14 = 0x96;
static const unsigned BME280_CALIB_15 = 0x97;
static const unsigned BME280_CALIB_16 = 0x98;
static const unsigned BME280_CALIB_17 = 0x99;
static const unsigned BME280_CALIB_18 = 0x9a;
static const unsigned BME280_CALIB_19 = 0x9b;
static const unsigned BME280_CALIB_20 = 0x9c;
static const unsigned BME280_CALIB_21 = 0x9d;
static const unsigned BME280_CALIB_22 = 0x9e;
static const unsigned BME280_CALIB_23 = 0x9f;
static const unsigned BME280_CALIB_24 = 0xa0;
static const unsigned BME280_CALIB_25 = 0xa1;
static const unsigned BME280_CALIB_26 = 0xe1;
static const unsigned BME280_CALIB_27 = 0xe2;
static const unsigned BME280_CALIB_28 = 0xe3;
static const unsigned BME280_CALIB_29 = 0xe4;
static const unsigned BME280_CALIB_30 = 0xe5;
static const unsigned BME280_CALIB_31 = 0xe6;
static const unsigned BME280_CALIB_32 = 0xe7;
static const unsigned BME280_CALIB_33 = 0xe8;
static const unsigned BME280_CALIB_34 = 0xe9;
static const unsigned BME280_CALIB_35 = 0xea;
static const unsigned BME280_CALIB_36 = 0xeb;
static const unsigned BME280_CALIB_37 = 0xec;
static const unsigned BME280_CALIB_38 = 0xed;
static const unsigned BME280_CALIB_39 = 0xee;
static const unsigned BME280_CALIB_40 = 0xef;
static const unsigned BME280_CALIB_41 = 0xf0;
