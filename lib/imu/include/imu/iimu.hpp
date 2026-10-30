/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-independent IMU interface.
 *
 * Every implementation, real or simulated, reports in one convention:
 * - accelerometer in g, as specific force: at rest the axis pointing up reads +1 g;
 * - gyroscope in dps, positive counter-clockwise about each axis (right-hand rule);
 * - both in the same right-handed sensor frame (x cross y = z).
 * A chip whose native frame differs is converted in its driver. imu/conformance.hpp
 * checks a sample stream against this convention.
 */

#ifndef IMU_IIMU_HPP
#define IMU_IIMU_HPP

#include <cstddef>
#include <cstdint>

#include <tl/expected.hpp>

#include "log/ilog.hpp"
#include "math-utils/math-utils.hpp"
#include "span.hpp"

namespace imu {

enum class IMUError : std::uint8_t {
    UNKNOWN,
    BUS_ERROR,
    INVALID_DEVICE,
    INVALID_CONFIG,
    NOT_INITIALIZED,
    UNSUPPORTED_OPERATION,
};

class IIMU {
  public:
    /* Common measurement type for all IMU sensor data. */
    using value_type = float;

    using gyro_measurement_type  = Vector3<value_type>;
    using accel_measurement_type = Vector3<value_type>;
    using mag_measurement_type   = Vector3<value_type>;

    IIMU(const IIMU &)            = delete;
    IIMU(IIMU &&)                 = delete;
    IIMU &operator=(const IIMU &) = delete;
    IIMU &operator=(IIMU &&)      = delete;

    IIMU()          = default;
    virtual ~IIMU() = default;

    virtual tl::expected<void, IMUError> init() noexcept                                      = 0;
    virtual tl::expected<void, IMUError> get_gyro_data(gyro_measurement_type &dst) noexcept   = 0;
    virtual tl::expected<void, IMUError> get_accel_data(accel_measurement_type &dst) noexcept = 0;
    virtual tl::expected<void, IMUError> get_mag_data(mag_measurement_type &dst) noexcept     = 0;
    virtual tl::expected<bool, IMUError> get_is_data_ready() noexcept                         = 0;

    /* A gyro sample newer than the last read. Chips without a separate flag report any new sample. */
    virtual tl::expected<bool, IMUError> get_is_gyro_data_ready() noexcept { return this->get_is_data_ready(); }

    /* Die temperature, for chips that have a sensor. */
    virtual tl::expected<value_type, IMUError> get_temperature_c() noexcept {
        return tl::make_unexpected(IMUError::UNSUPPORTED_OPERATION);
    }

    /* One sample as output codes, already in the IIMU frame. */
    struct RawSample {
        std::int16_t accel[3];
        std::int16_t gyro[3];
    };

    /* Multiply a RawSample code by these to get g and dps. */
    struct RawScale {
        value_type accel_g_per_lsb;
        value_type gyro_dps_per_lsb;
    };

    virtual tl::expected<RawScale, IMUError> get_raw_scale() const noexcept {
        return tl::make_unexpected(IMUError::UNSUPPORTED_OPERATION);
    }

    /* Buffers every sample at ODR in the chip's FIFO, so none is missed between reads. */
    virtual tl::expected<void, IMUError> enable_fifo() noexcept {
        return tl::make_unexpected(IMUError::UNSUPPORTED_OPERATION);
    }

    /* Pops up to dst.size() samples, oldest first. overrun is set when the chip dropped
     * samples since the previous read. */
    virtual tl::expected<std::size_t, IMUError> read_fifo(
        [[maybe_unused]] span<RawSample> dst, [[maybe_unused]] bool &overrun
    ) noexcept {
        return tl::make_unexpected(IMUError::UNSUPPORTED_OPERATION);
    }
};

} /* namespace imu */

template <> struct fmt::formatter<imu::IMUError> : fmt::formatter<const char *> {
    template <typename FmtContext> auto format(const imu::IMUError &error, FmtContext &ctx) const {
        const char *name = nullptr;
        switch (error) {
        case imu::IMUError::UNKNOWN:
            name = "UNKNOWN";
            break;
        case imu::IMUError::BUS_ERROR:
            name = "BUS_ERROR";
            break;
        case imu::IMUError::INVALID_DEVICE:
            name = "INVALID_DEVICE";
            break;
        case imu::IMUError::INVALID_CONFIG:
            name = "INVALID_CONFIG";
            break;
        case imu::IMUError::NOT_INITIALIZED:
            name = "NOT_INITIALIZED";
            break;
        case imu::IMUError::UNSUPPORTED_OPERATION:
            name = "UNSUPPORTED_OPERATION";
            break;
        }

        return fmt::formatter<const char *>::format(name, ctx);
    }
};

#endif /* IMU_IIMU_HPP */
