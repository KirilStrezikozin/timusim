/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * LSM9DS1 accelerometer and gyroscope config resolved onto the chip::Model path:
 *   accel: anti-aliasing filter (internal), digital LPF in HR mode (ODR)
 *   gyro:  LPF1 (internal), LPF2 when in the output path (ODR)
 * The chip frame is left-handed and the driver negates z.
 *
 * Cutoffs come from the datasheet (DocID025715 Rev 3) Tables 46, 47 and 71 and
 * CTRL_REG6_XL. Filter shapes, noise and bias terms are Params, fitted to the V1
 * bench captures.
 */

#ifndef CHIP_LSM9DS1_HPP
#define CHIP_LSM9DS1_HPP

#include <cstdint>

#include <tl/expected.hpp>

#include "chip/model.hpp"
#include "imu/iimu.hpp"
#include "imu/lsm9ds1/config.hpp"
#include "imu/lsm9ds1/utils.hpp"

namespace chip::lsm9ds1 {

using chip::Cutoffs;
using chip::Markov;
using chip::Mat3;
using chip::Noise;
using chip::Output;
using chip::Params;
using chip::Vec3;
using Config = imu::lsm9ds1::config::IMUConfig;

/* 14.9 and 59.5 Hz have no LPF2 or anti-aliasing entry in the datasheet. */
inline tl::expected<double, imu::IMUError> nominal_odr_hz(lsm9ds1_imu_odr_t odr) {
    const double hz = imu::lsm9ds1::utils::odr_hz(odr);
    if (hz < 100.0) { return tl::make_unexpected(imu::IMUError::INVALID_CONFIG); }
    return hz;
}

/* CTRL_REG6_XL, BW_SCAL_ODR = 0 (the driver sets auto). Analog, so not scaled by odr_scale. */
inline double accel_aa_hz(double odr) {
    return odr > 900.0 ? 408.0 : odr > 400.0 ? 211.0 : odr > 200.0 ? 105.0 : 50.0;
}

/* Table 46. */
inline double gyro_lpf1_hz(double odr) {
    return odr > 900.0 ? 100.0 : odr > 400.0 ? 100.0 : odr > 200.0 ? 76.0 : 38.0;
}

/* Table 47. */
inline double gyro_lpf2_hz(double odr, lsm9ds1_gy_lp_bw_t bw) {
    static constexpr double TABLE[4][4] = {
        {14.0, 31.0, 31.0, 31.0},  /* 119 */
        {14.0, 29.0, 63.0, 78.0},  /* 238 */
        {21.0, 28.0, 57.0, 100.0}, /* 476 */
        {33.0, 40.0, 58.0, 100.0}, /* 952 */
    };
    const int row = odr > 900.0 ? 3 : odr > 400.0 ? 2 : odr > 200.0 ? 1 : 0;
    return TABLE[row][static_cast<int>(bw) & 3];
}

/* Table 71, HR = 1; 0 when the filter is off. */
inline double accel_lp_divider(lsm9ds1_xl_lp_bw_t bw) {
    switch (bw) {
    case LSM9DS1_LP_ODR_DIV_50:
        return 50.0;
    case LSM9DS1_LP_ODR_DIV_100:
        return 100.0;
    case LSM9DS1_LP_ODR_DIV_9:
        return 9.0;
    case LSM9DS1_LP_ODR_DIV_400:
        return 400.0;
    default:
        return 0.0;
    }
}

inline tl::expected<Path, imu::IMUError> path(const Config &config, const Params &params) {
    auto odr = nominal_odr_hz(config.imu_odr);
    if (!odr) { return tl::make_unexpected(odr.error()); }
    namespace u = imu::lsm9ds1::utils;
    if ((config.gyro_out_path != LSM9DS1_LPF1_OUT && config.gyro_out_path != LSM9DS1_LPF1_LPF2_OUT)
        || u::accel_fs_g(config.accel_range) == 0.0 || u::gyro_fs_dps(config.gyro_range) == 0.0) {
        return tl::make_unexpected(imu::IMUError::INVALID_CONFIG);
    }
    const double div = accel_lp_divider(config.accel_bw);
    const bool lpf2  = config.gyro_out_path == LSM9DS1_LPF1_LPF2_OUT;
    return Path{
        .chip           = "LSM9DS1",
        .nominal_odr_hz = *odr,
        .oversample     = params.oversample,
        .cutoffs =
            Cutoffs{
                .accel_aa  = accel_aa_hz(*odr),
                .accel_lp  = div > 0.0 ? *odr / div : 0.0,
                .gyro_lpf1 = gyro_lpf1_hz(*odr),
                .gyro_lpf2 = lpf2 ? gyro_lpf2_hz(*odr, config.gyro_bw) : 0.0,
            },
        .accel_fs_g    = u::accel_fs_g(config.accel_range),
        .gyro_fs_dps   = u::gyro_fs_dps(config.gyro_range),
        .accel_lsb_g   = u::accel_lsb_g(config.accel_range),
        .gyro_lsb_dps  = u::gyro_lsb_dps(config.gyro_range),
        .negate_z      = true,
        .accel_bw      = static_cast<int>(config.accel_bw),
        .gyro_bw       = static_cast<int>(config.gyro_bw),
        .gyro_out_path = static_cast<int>(config.gyro_out_path),
    };
}

struct Model {
    using Config = lsm9ds1::Config;

    static tl::expected<chip::Model, imu::IMUError> make(
        const Config &config, const Params &params, std::uint64_t seed
    ) {
        auto p = path(config, params);
        if (!p) { return tl::make_unexpected(p.error()); }
        return chip::Model::make(*p, params, seed);
    }
};

} /* namespace chip::lsm9ds1 */

namespace chip {

inline tl::expected<Path, imu::IMUError> path(const lsm9ds1::Config &config, const Params &params) {
    return lsm9ds1::path(config, params);
}

} /* namespace chip */

#endif /* CHIP_LSM9DS1_HPP */
