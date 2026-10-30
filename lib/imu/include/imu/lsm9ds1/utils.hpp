/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * LSM9DS1 register values as physical quantities. 0 for values with none.
 */

#ifndef IMU_LSM9DS1_UTILS_HPP
#define IMU_LSM9DS1_UTILS_HPP

#include <lsm9ds1_reg.h>

namespace imu::lsm9ds1::utils {

/* Accel and gyro both on. */
constexpr double odr_hz(lsm9ds1_imu_odr_t odr) {
    switch (odr) {
    case LSM9DS1_IMU_14Hz9:
        return 14.9;
    case LSM9DS1_IMU_59Hz5:
        return 59.5;
    case LSM9DS1_IMU_119Hz:
        return 119.0;
    case LSM9DS1_IMU_238Hz:
        return 238.0;
    case LSM9DS1_IMU_476Hz:
        return 476.0;
    case LSM9DS1_IMU_952Hz:
        return 952.0;
    default:
        return 0.0;
    }
}

constexpr double accel_fs_g(lsm9ds1_xl_fs_t fs) {
    switch (fs) {
    case LSM9DS1_2g:
        return 2.0;
    case LSM9DS1_4g:
        return 4.0;
    case LSM9DS1_8g:
        return 8.0;
    case LSM9DS1_16g:
        return 16.0;
    default:
        return 0.0;
    }
}

constexpr double gyro_fs_dps(lsm9ds1_gy_fs_t fs) {
    switch (fs) {
    case LSM9DS1_245dps:
        return 245.0;
    case LSM9DS1_500dps:
        return 500.0;
    case LSM9DS1_2000dps:
        return 2000.0;
    default:
        return 0.0;
    }
}

inline double accel_lsb_g(lsm9ds1_xl_fs_t fs) {
    switch (fs) {
    case LSM9DS1_2g:
        return lsm9ds1_from_fs2g_to_mg(1) / 1000.0;
    case LSM9DS1_4g:
        return lsm9ds1_from_fs4g_to_mg(1) / 1000.0;
    case LSM9DS1_8g:
        return lsm9ds1_from_fs8g_to_mg(1) / 1000.0;
    case LSM9DS1_16g:
        return lsm9ds1_from_fs16g_to_mg(1) / 1000.0;
    default:
        return 0.0;
    }
}

inline double gyro_lsb_dps(lsm9ds1_gy_fs_t fs) {
    switch (fs) {
    case LSM9DS1_245dps:
        return lsm9ds1_from_fs245dps_to_mdps(1) / 1000.0;
    case LSM9DS1_500dps:
        return lsm9ds1_from_fs500dps_to_mdps(1) / 1000.0;
    case LSM9DS1_2000dps:
        return lsm9ds1_from_fs2000dps_to_mdps(1) / 1000.0;
    default:
        return 0.0;
    }
}

} /* namespace imu::lsm9ds1::utils */

#endif /* IMU_LSM9DS1_UTILS_HPP */
