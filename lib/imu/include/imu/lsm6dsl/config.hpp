/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * LSM6DSL Configuration.
 */

#ifndef IMU_LSM6DSL_CONFIG_HPP
#define IMU_LSM6DSL_CONFIG_HPP

#include <cstdint>

#include <lsm6dsl_reg.h>

namespace imu::lsm6dsl::config {

enum class IMUI2CAddress : std::uint8_t {
/*
 * STMicroelectronics defines I2C addresses in 8-bit format, which includes
 * the Read/Write bit at the Least Significant Bit (LSB) position.
 * Frameworks like Arduino (Wire) and the Raspberry Pi Pico SDK strictly
 * require standard 7-bit I2C addresses and handle the R/W bit automatically.
 * We right-shift (>> 1) the ST-provided macros to extract the 7-bit base
 * address for these platforms, while preserving the raw 8-bit address
 * for native STM32 HAL usage.
 */
#if defined(TARGET_PLATFORM_ARDUINO) || defined(TARGET_PLATFORM_PICO)
    /* Shift right by 1 to convert from 8-bit to 7-bit address format. */
    LSM6DSL_SA0_LOW  = (LSM6DSL_I2C_ADD_L >> 1), /* SDO/SA0 pin low. */
    LSM6DSL_SA0_HIGH = (LSM6DSL_I2C_ADD_H >> 1), /* SDO/SA0 pin high. */
#else
    /* Leave as is for native STM32 HAL usage. */
    LSM6DSL_SA0_LOW  = LSM6DSL_I2C_ADD_L, /* SDO/SA0 pin low. */
    LSM6DSL_SA0_HIGH = LSM6DSL_I2C_ADD_H, /* SDO/SA0 pin high. */
#endif
};

/* User-facing configuration struct utilizing ST's PID enums directly. */
struct IMUConfig {
    uint8_t whoami_value = LSM6DSL_ID;

    lsm6dsl_fs_xl_t accel_range = LSM6DSL_4g;
    lsm6dsl_odr_xl_t accel_odr  = LSM6DSL_XL_ODR_208Hz;

    lsm6dsl_fs_g_t gyro_range = LSM6DSL_2000dps;
    lsm6dsl_odr_g_t gyro_odr  = LSM6DSL_GY_ODR_208Hz;

    /* Analog filter for accel (from earlier) */
    lsm6dsl_bw0_xl_t accel_bw = LSM6DSL_XL_ANA_BW_400Hz;

    /* Digital filter for gyro */
    lsm6dsl_lpf1_sel_g_t gyro_bw = LSM6DSL_HP_DISABLE_LP1_LIGHT;

    /* Digital filter for accel (LPF1 + LPF2 path) */
    lsm6dsl_input_composite_t accel_lp2_bw = LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9;

    /* Enable Block Data Update (BDU). */
    bool enable_bdu = true;

    [[nodiscard]] constexpr bool is_valid() const noexcept {
        return this->whoami_value == LSM6DSL_ID;
    }
};

} /* namespace imu::lsm6dsl::config */

#endif /* IMU_LSM6DSL_CONFIG_HPP */
