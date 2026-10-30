/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * LSM6DSL accelerometer and gyroscope config, high-performance mode, resolved
 * onto the chip::Model path:
 *   accel: analog anti-aliasing filter and digital LPF1 as one internal stage
 *          (accel_aa), LPF2 at ODR (accel_lp); the driver always enables LPF2
 *   gyro:  LPF1 and the fixed LPF2 as one internal stage (gyro_lpf1) with the
 *          overall cutoff; the gyro_lpf2 slot stays empty
 * The chip frame is right-handed and the driver passes it through.
 *
 * Cutoffs come from AN5040 Rev 3 Tables 8, 9 and 11. The internal rate is taken
 * as the highest ODR, 6.66 kHz (DS Table 3), where LPF2 is bypassed.
 */

#ifndef CHIP_LSM6DSL_HPP
#define CHIP_LSM6DSL_HPP

#include <cmath>
#include <cstdint>

#include <tl/expected.hpp>

#include "chip/model.hpp"
#include "imu/iimu.hpp"
#include "imu/lsm6dsl/config.hpp"
#include "imu/lsm6dsl/utils.hpp"

namespace chip::lsm6dsl {

using Config = imu::lsm6dsl::config::IMUConfig;

constexpr double INTERNAL_HZ = 6664.0;

/* Other rates have no FIFO capture to fit against yet. */
inline tl::expected<double, imu::IMUError> nominal_odr_hz(lsm6dsl_odr_xl_t odr) {
    const double hz = imu::lsm6dsl::utils::odr_hz(odr);
    if (hz < 100.0 || hz > 1000.0) { return tl::make_unexpected(imu::IMUError::INVALID_CONFIG); }
    return hz;
}

/* AN5040 Table 8: below 1666 Hz ODR the analog filter is 400 Hz whatever BW0_XL is. */
constexpr double ACCEL_ANALOG_HZ = 400.0;

/* AN5040 Table 11, overall gyro cutoff for LPF1 off, then FTYPE 00, 01, 10, 11. */
inline double gyro_cutoff_hz(double odr, lsm6dsl_lpf1_sel_g_t bw) {
    static constexpr double TABLE[4][5] = {
        {33.0, 33.0, 33.0, 33.0, 33.0},      /* 104 */
        {67.0, 67.0, 67.0, 67.0, 67.0},      /* 208 */
        {137.0, 138.0, 131.0, 121.0, 138.0}, /* 416 */
        {312.0, 245.0, 195.0, 155.0, 293.0}, /* 833 */
    };
    const int row      = odr > 800.0 ? 3 : odr > 400.0 ? 2 : odr > 200.0 ? 1 : 0;
    const auto v       = static_cast<unsigned>(bw);
    const bool lpf1_on = (v & 0x08U) != 0U;
    return TABLE[row][lpf1_on ? 1 + (v & 0x03U) : 0];
}

/* AN5040 Table 9, low-pass path with LPF2_XL_EN = 1: HPCF_XL picks the divider. */
inline double accel_lp2_divider(lsm6dsl_input_composite_t bw) {
    switch (static_cast<unsigned>(bw) & 0x03U) {
    case 0x00U:
        return 50.0;
    case 0x01U:
        return 100.0;
    case 0x02U:
        return 9.0;
    default:
        return 400.0;
    }
}

/* INPUT_COMPOSITE: 0 feeds LPF2 from the ODR/2 output of LPF1, 1 from ODR/4. */
inline double accel_lpf1_divider(lsm6dsl_input_composite_t bw) {
    return (static_cast<unsigned>(bw) & 0x10U) != 0U ? 4.0 : 2.0;
}

inline tl::expected<Path, imu::IMUError> path(
    const Config &config, [[maybe_unused]] const Params &params
) {
    auto odr = nominal_odr_hz(config.accel_odr);
    if (!odr) { return tl::make_unexpected(odr.error()); }
    /* One rate for both, as the driver's FIFO needs; the gyro HP filter is not modelled. */
    namespace u = imu::lsm6dsl::utils;
    if (static_cast<int>(config.gyro_odr) != static_cast<int>(config.accel_odr)
        || (static_cast<unsigned>(config.gyro_bw) & 0x80U) != 0U
        || config.gyro_bw == LSM6DSL_HP_GY_BAND_NA || u::accel_fs_g(config.accel_range) == 0.0
        || u::gyro_fs_dps(config.gyro_range) == 0.0) {
        return tl::make_unexpected(imu::IMUError::INVALID_CONFIG);
    }
    const double lpf1 = *odr / accel_lpf1_divider(config.accel_lp2_bw);
    return Path{
        .chip           = "LSM6DSL",
        .nominal_odr_hz = *odr,
        .oversample     = static_cast<int>(std::lround(INTERNAL_HZ / *odr)),
        .cutoffs =
            Cutoffs{
                .accel_aa  = std::min(ACCEL_ANALOG_HZ, lpf1),
                .accel_lp  = *odr / accel_lp2_divider(config.accel_lp2_bw),
                .gyro_lpf1 = gyro_cutoff_hz(*odr, config.gyro_bw),
                .gyro_lpf2 = 0.0,
            },
        .accel_fs_g    = u::accel_fs_g(config.accel_range),
        .gyro_fs_dps   = u::gyro_fs_dps(config.gyro_range),
        .accel_lsb_g   = u::accel_lsb_g(config.accel_range),
        .gyro_lsb_dps  = u::gyro_lsb_dps(config.gyro_range),
        .negate_z      = false,
        .accel_bw      = static_cast<int>(config.accel_lp2_bw),
        .gyro_bw       = static_cast<int>(config.gyro_bw),
        .gyro_out_path = -1,
    };
}

struct Model {
    using Config = lsm6dsl::Config;

    static tl::expected<chip::Model, imu::IMUError> make(
        const Config &config, const Params &params, std::uint64_t seed
    ) {
        auto p = path(config, params);
        if (!p) { return tl::make_unexpected(p.error()); }
        return chip::Model::make(*p, params, seed);
    }
};

} /* namespace chip::lsm6dsl */

namespace chip {

inline tl::expected<Path, imu::IMUError> path(const lsm6dsl::Config &config, const Params &params) {
    return lsm6dsl::path(config, params);
}

} /* namespace chip */

#endif /* CHIP_LSM6DSL_HPP */
