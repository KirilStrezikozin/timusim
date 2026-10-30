/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * LSM9DS1 Configuration.
 */

#ifndef IMU_LSM9DS1_CONFIG_HPP
#define IMU_LSM9DS1_CONFIG_HPP

#include <cstdint>

#include <lsm9ds1_reg.h>

namespace imu::lsm9ds1::config {

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
    LSM9DS1_SA0_LOW  = (LSM9DS1_IMU_I2C_ADD_L >> 1), /* SDO/SA0 pin low. */
    LSM9DS1_SA0_HIGH = (LSM9DS1_IMU_I2C_ADD_H >> 1), /* SDO/SA0 pin high. */
#else
    /* Leave as is for native STM32 HAL usage. */
    LSM9DS1_SA0_LOW  = LSM9DS1_IMU_I2C_ADD_L, /* SDO/SA0 pin low. */
    LSM9DS1_SA0_HIGH = LSM9DS1_IMU_I2C_ADD_H, /* SDO/SA0 pin high. */
#endif
};

/* User-facing configuration struct utilizing ST's PID enums directly. */
struct IMUConfig {
    uint8_t whoami_value = LSM9DS1_IMU_ID;

    lsm9ds1_xl_fs_t accel_range = LSM9DS1_4g;
    lsm9ds1_gy_fs_t gyro_range  = LSM9DS1_2000dps;

    /* Configures ODR for both Accel and Gyro when activated together */
    lsm9ds1_imu_odr_t imu_odr = LSM9DS1_IMU_238Hz;

    /* Low pass filter bandwidth for accel */
    lsm9ds1_xl_lp_bw_t accel_bw = LSM9DS1_LP_ODR_DIV_9;

    /* Low pass filter bandwidth for gyro (LPF2); no effect unless gyro_out_path includes LPF2 */
    lsm9ds1_gy_lp_bw_t gyro_bw = LSM9DS1_LP_ULTRA_LIGHT;

    lsm9ds1_gy_out_path_t gyro_out_path = LSM9DS1_LPF1_OUT;

    /* Enable Block Data Update (BDU). */
    bool enable_bdu = true;

    [[nodiscard]] constexpr bool is_valid() const noexcept {
        return (this->whoami_value == LSM9DS1_IMU_ID)
            || (this->whoami_value == (LSM9DS1_IMU_ID | 0b10));
    }
};

} /* namespace imu::lsm9ds1::config */

#endif /* IMU_LSM9DS1_CONFIG_HPP */
