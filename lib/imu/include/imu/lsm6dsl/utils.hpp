/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * LSM6DSL register values as physical quantities and back. 0 or _ND for values with none.
 */

#ifndef IMU_LSM6DSL_UTILS_HPP
#define IMU_LSM6DSL_UTILS_HPP

#include <lsm6dsl_reg.h>

namespace imu::lsm6dsl::utils {

/* High-performance mode rates; 1.6 Hz is low-power only. */
constexpr double odr_hz(lsm6dsl_odr_xl_t odr) {
    switch (odr) {
    case LSM6DSL_XL_ODR_12Hz5:
        return 12.5;
    case LSM6DSL_XL_ODR_26Hz:
        return 26.0;
    case LSM6DSL_XL_ODR_52Hz:
        return 52.0;
    case LSM6DSL_XL_ODR_104Hz:
        return 104.0;
    case LSM6DSL_XL_ODR_208Hz:
        return 208.0;
    case LSM6DSL_XL_ODR_416Hz:
        return 416.0;
    case LSM6DSL_XL_ODR_833Hz:
        return 833.0;
    case LSM6DSL_XL_ODR_1k66Hz:
        return 1666.0;
    case LSM6DSL_XL_ODR_3k33Hz:
        return 3332.0;
    case LSM6DSL_XL_ODR_6k66Hz:
        return 6664.0;
    default:
        return 0.0;
    }
}

constexpr lsm6dsl_odr_xl_t accel_odr(int hz) {
    switch (hz) {
    case 104:
        return LSM6DSL_XL_ODR_104Hz;
    case 208:
        return LSM6DSL_XL_ODR_208Hz;
    case 416:
        return LSM6DSL_XL_ODR_416Hz;
    case 833:
        return LSM6DSL_XL_ODR_833Hz;
    default:
        return LSM6DSL_XL_ODR_ND;
    }
}

constexpr lsm6dsl_odr_g_t gyro_odr(int hz) {
    switch (hz) {
    case 104:
        return LSM6DSL_GY_ODR_104Hz;
    case 208:
        return LSM6DSL_GY_ODR_208Hz;
    case 416:
        return LSM6DSL_GY_ODR_416Hz;
    case 833:
        return LSM6DSL_GY_ODR_833Hz;
    default:
        return LSM6DSL_GY_ODR_ND;
    }
}

constexpr double accel_fs_g(lsm6dsl_fs_xl_t fs) {
    switch (fs) {
    case LSM6DSL_2g:
        return 2.0;
    case LSM6DSL_4g:
        return 4.0;
    case LSM6DSL_8g:
        return 8.0;
    case LSM6DSL_16g:
        return 16.0;
    default:
        return 0.0;
    }
}

constexpr double gyro_fs_dps(lsm6dsl_fs_g_t fs) {
    switch (fs) {
    case LSM6DSL_125dps:
        return 125.0;
    case LSM6DSL_250dps:
        return 250.0;
    case LSM6DSL_500dps:
        return 500.0;
    case LSM6DSL_1000dps:
        return 1000.0;
    case LSM6DSL_2000dps:
        return 2000.0;
    default:
        return 0.0;
    }
}

inline double accel_lsb_g(lsm6dsl_fs_xl_t fs) {
    switch (fs) {
    case LSM6DSL_2g:
        return lsm6dsl_from_fs2g_to_mg(1) / 1000.0;
    case LSM6DSL_4g:
        return lsm6dsl_from_fs4g_to_mg(1) / 1000.0;
    case LSM6DSL_8g:
        return lsm6dsl_from_fs8g_to_mg(1) / 1000.0;
    case LSM6DSL_16g:
        return lsm6dsl_from_fs16g_to_mg(1) / 1000.0;
    default:
        return 0.0;
    }
}

inline double gyro_lsb_dps(lsm6dsl_fs_g_t fs) {
    switch (fs) {
    case LSM6DSL_125dps:
        return lsm6dsl_from_fs125dps_to_mdps(1) / 1000.0;
    case LSM6DSL_250dps:
        return lsm6dsl_from_fs250dps_to_mdps(1) / 1000.0;
    case LSM6DSL_500dps:
        return lsm6dsl_from_fs500dps_to_mdps(1) / 1000.0;
    case LSM6DSL_1000dps:
        return lsm6dsl_from_fs1000dps_to_mdps(1) / 1000.0;
    case LSM6DSL_2000dps:
        return lsm6dsl_from_fs2000dps_to_mdps(1) / 1000.0;
    default:
        return 0.0;
    }
}

} /* namespace imu::lsm6dsl::utils */

#endif /* IMU_LSM6DSL_UTILS_HPP */
