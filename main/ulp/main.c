/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* ULP RISC-V RTC I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This code runs on ULP RISC-V coprocessor
*/

#include <stdint.h>
#include <stdbool.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_i2c_ulp_core.h"
#include "../bmp180_defs.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

/************************************************
 * Shared data between main CPU and ULP
 ************************************************/
int32_t status_reg_data = 0;
int32_t conut = 0;
uint32_t rtc_time_h = 0, rtc_time_l = 0;

static uint64_t my_rtc_time_get(void);
static void my_ulp_resv_i2c_write_byte(uint8_t reg_addr, uint8_t reg_data);
static void my_ulp_resv_i2c_read_byte(uint8_t reg_addr, uint8_t *reg_data);

int main(void)
{
    // uint8_t status_reg = 0, int_rel;
    // my_ulp_resv_i2c_read_byte(STATUS_REG, &status_reg);
    // status_reg_data = status_reg;
    // // bool isMove = (status_reg >> 4) & 1;
    // ulp_riscv_delay_cycles(10 * ULP_RISCV_CYCLES_PER_US * 1000);
    // my_ulp_resv_i2c_read_byte(INT_REL, &int_rel); // 读取以清除状态
    uint64_t rtc_time = my_rtc_time_get();
    // uint64_t rtc_time = 0;
    rtc_time_h = (rtc_time >> 32) & 0xFFFFFFFF;
    rtc_time_l = rtc_time & 0xFFFFFFFF;
    // if (isMove  || ++conut > 10)
    if (++conut > 5)
    {
        ulp_riscv_wakeup_main_processor();
    }

    return 0;
}

static uint64_t my_rtc_time_get(void)
{
    SET_PERI_REG_MASK(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_UPDATE);
    ulp_riscv_delay_cycles(ULP_RISCV_CYCLES_PER_US * 1000);
    uint64_t t;
    t = READ_PERI_REG(RTC_CNTL_TIME0_REG);
    t |= ((uint64_t)READ_PERI_REG(RTC_CNTL_TIME1_REG)) << 32;
    return t;
}

static void my_ulp_resv_i2c_write_byte(uint8_t reg_addr, uint8_t reg_data)
{
    ulp_riscv_i2c_master_set_slave_reg_addr(reg_addr);
    uint8_t data_wr = reg_data;
    ulp_riscv_i2c_master_write_to_device(&data_wr, 1);
}

static void my_ulp_resv_i2c_read_byte(uint8_t reg_addr, uint8_t *reg_data)
{
    ulp_riscv_i2c_master_set_slave_reg_addr(reg_addr);
    ulp_riscv_i2c_master_read_from_device(reg_data, 1);
}
