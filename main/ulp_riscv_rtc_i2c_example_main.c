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
*/

#include <stdio.h>
#include <math.h>
#include "esp_sleep.h"
#include "ulp_riscv.h"
#include "ulp_riscv_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ulp_main.h"
#include "bmp180_defs.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

/************************************************
 * ULP utility APIs
 ************************************************/
static void init_ulp_program(void);

/************************************************
 * RTC I2C utility APIs
 ************************************************/
static void init_i2c(void);
static void my_ulp_resv_i2c_write_byte(uint8_t reg_addr, uint8_t reg_data);
static void my_ulp_resv_i2c_read_byte(uint8_t reg_addr, uint8_t *reg_data);

void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    /* Not a wakeup from ULP
     * Initialize RTC I2C
     * Setup BMP180 sensor
     * Store current temperature and pressure values
     * Load the ULP firmware
     * Go to deep sleep
     */
    if (cause != ESP_SLEEP_WAKEUP_ULP)
    {
        printf("Not a ULP-RISC V wakeup (cause = %d)\n", cause);

        /* Initialize RTC I2C */
        // if (cause == 0)
        //     init_i2c();

        // /* Configure I2C slave address */
        // ulp_riscv_i2c_master_set_slave_addr(KXTJ_SENSOR_I2C_ADDR);
        // /* 判断传感器是否启动 */
        // uint8_t ctrlReg1 = 0;
        // my_ulp_resv_i2c_read_byte(CTRL_REG1, &ctrlReg1);
        // bool isStart = (ctrlReg1 >> 7) & 0x01;
        // if (!isStart)
        // {
        //     uint8_t sensorWakeupCounter = (SENSOR_WAKEUP_PARAM_DEFAULT >> 16) & 0xFF;
        //     uint8_t sensorWakeupThresholdH = (SENSOR_WAKEUP_PARAM_DEFAULT >> 8) & 0xFF;
        //     uint8_t sensorWakeupThresholdL = SENSOR_WAKEUP_PARAM_DEFAULT & 0xFF;
        //     /* 设置加速度阈值 */
        //     my_ulp_resv_i2c_write_byte(ADDR_WAKEUP_THRESHOLD_H, sensorWakeupThresholdH);
        //     my_ulp_resv_i2c_write_byte(ADDR_WAKEUP_THRESHOLD_L, sensorWakeupThresholdL);

        //     my_ulp_resv_i2c_write_byte(ADDR_DATA_CTRL_REG, 0x02);
        //     my_ulp_resv_i2c_write_byte(ADDR_CTRL_REG2, 0x06);
        //     /* 设置时间窗口 */
        //     my_ulp_resv_i2c_write_byte(ADDR_WAKEUP_COUNTER, sensorWakeupCounter);
        //     my_ulp_resv_i2c_write_byte(INT_CTRL_REG1, 0x32); // 物理中断
        //     /* 启动以运动传感器 */
        //     my_ulp_resv_i2c_write_byte(CTRL_REG1, 0x82); // 开启传感器，并打开运动唤醒;8A是正负4G，82是正负2G
        // }
        init_ulp_program();
    }

    /* ULP RISC-V read and detected a temperature or pressure above the limit */
    if (cause == ESP_SLEEP_WAKEUP_ULP)
    {
        printf("ULP RISC-V woke up the main CPU\n");

        /* Pause ULP while we are using the RTC I2C from the main CPU */
        ulp_timer_stop();
        ulp_riscv_halt();

        printf("Uncompensated rtc_time:%08x%08x\n", ulp_rtc_time_h, ulp_rtc_time_l);
        // printf("Uncompensated status_reg_data = %d\n", ulp_status_reg_data);
        printf("Uncompensated conut = %d\n", ulp_conut);
        ulp_conut = 0;

        // /* Read the calibration data again */
        // printf("Reading calibration data from BMP180 ...\n");
        // // bmp180_read_cal_data();
        // printf("\n");

        /* Calculate real temperature and pressure again */
        // temperature = 0;
        // temperature = bmp180_calculate_real_temp((int32_t)ulp_ut_data);
        // printf("New Real Temperature = %f deg celcius\n", (float)(temperature / 10.0));

        // /* Calculate real pressure value */
        // pressure = 0;
        // pressure = bmp180_calculate_real_pressure(ulp_up_data, (int32_t)ulp_ut_data, oss_mode);
        // printf("New Real Pressure = %f hPa\n", pressure / 100.0);

        /* Resume ULP and go to deep sleep again */
        ulp_timer_resume();
    }
    // uint8_t status_reg = 0, int_rel;
    // my_ulp_resv_i2c_read_byte(STATUS_REG, &status_reg);
    // // status_reg_data = status_reg;
    // bool isMove = (status_reg >> 4) & 1;
    // // ulp_riscv_delay_cycles(5 * ULP_RISCV_CYCLES_PER_US * 1000);
    // my_ulp_resv_i2c_read_byte(INT_REL, &int_rel); // 读取以清除状态
    // printf("status_reg:0x%02x\r\n", status_reg);

    /* Add a delay for everything to the printed before heading in to light sleep */
    vTaskDelay(100);

    /* Go back to sleep, only the ULP RISC-V will run */
    printf("Entering deep sleep\n\n");

    /* RTC peripheral power domain needs to be kept on to keep RTC I2C related configs during sleep */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

    esp_deep_sleep_start();
}

static void init_i2c(void)
{
    /* Configure RTC I2C */
    printf("Initializing RTC I2C ...\n");
    ulp_riscv_i2c_cfg_t i2c_cfg = ULP_RISCV_I2C_DEFAULT_CONFIG();
    i2c_cfg.i2c_pin_cfg.sda_io_num = GPIO_NUM_1;
    i2c_cfg.i2c_pin_cfg.scl_io_num = GPIO_NUM_2;
    ulp_riscv_i2c_master_init(&i2c_cfg);
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


static void init_ulp_program(void)
{
    esp_err_t err = ulp_riscv_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
    ESP_ERROR_CHECK(err);

    /* The first argument is the period index, which is not used by the ULP-RISC-V timer
     * The second argument is the period in microseconds, which gives a wakeup time period of: 20ms
     */
    ulp_set_wakeup_period(0, 1000000);

    /* Start the program */
    err = ulp_riscv_run();
    ESP_ERROR_CHECK(err);
}
