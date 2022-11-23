/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

/***************************************************
 * BMP180 Register Addresses
 ***************************************************/
#define KXTJ_SENSOR_I2C_ADDR 0x0E
// 运动传感器的主要寄存器地址。
#define AXIS_XL 0x06
#define AXIS_XH 0x07
#define AXIS_YL 0x08
#define AXIS_YH 0x09
#define AXIS_ZL 0x0A
#define AXIS_ZH 0x0B
#define INT_SOURCE2 0x17
#define STATUS_REG 0x18
#define INT_REL 0x1A
#define CTRL_REG1 0x1B
#define INT_CTRL_REG1 0x1E

//新增对于触发运动的阈值的设置
#define ADDR_WAKEUP_THRESHOLD_L 0x6B
#define ADDR_WAKEUP_THRESHOLD_H 0x6A

#define ADDR_DATA_CTRL_REG 0x21  //内容需要设置为0x02，加速度的采样频率为50Hz
#define ADDR_CTRL_REG2 0x1D      //内容需要设置为0x06,唤醒函数也调整为50Hz
#define ADDR_WAKEUP_COUNTER 0x29 // 设置为0x05，这样0.1秒的加速度变化也能够被监测出来

/***************************************************
 * BMP180 Control Commands
 ***************************************************/
#define BMP180_SENSOR_CMD_READ_TEMPERATURE 0x2E
#define BMP180_SENSOR_CMD_READ_PRESSURE_OSS_0 0x34
#define BMP180_SENSOR_CMD_READ_PRESSURE_OSS_1 0x74
#define BMP180_SENSOR_CMD_READ_PRESSURE_OSS_2 0xB4
#define BMP180_SENSOR_CMD_READ_PRESSURE_OSS_3 0xF4
#define BMP180_SENSOR_CMD_SOFT_RESET 0xB6

/***************************************************
 * BMP180 Chip ID
 ***************************************************/
#define BMP180_SENSOR_CHIP_ID 0x55

/***************************************************
 * BMP180 Calibration Data
 ***************************************************/

#define SENSOR_WAKEUP_COUNTER 0x0F     // 0x0A对应的窗口时间为 0.2s, 0x0F 对应0.3S
#define SENSOR_WAKEUP_THRESHOLD 0x01A0 //  0x01A0 对应的是26，对应换算公式的0.1个g左右;0x0340对应52,0.2g
#define SENSOR_WAKEUP_PARAM_DEFAULT (((SENSOR_WAKEUP_COUNTER & 0xFF) << 16) | (SENSOR_WAKEUP_THRESHOLD & 0xFFFF))
