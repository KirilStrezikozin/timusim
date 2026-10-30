/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * LSM9DS1 Base Implementation utilizing ST's PID.
 */

#ifndef IMU_LSM9DS1_BASE_HPP
#define IMU_LSM9DS1_BASE_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

#include <lsm9ds1_reg.h>
#include <tl/expected.hpp>

#include "bus/ibus.hpp"
#include "imu/iimu.hpp"
#include "imu/lsm9ds1/config.hpp"
#include "log/ilog.hpp"
#include "span.hpp"
#include "time/sleep.h"

namespace imu::lsm9ds1::base {

class BaseIMU_ : public IIMU {
#define ensure_initialized_hook()                                                                  \
    auto initialized = this->check_initialized_();                                                 \
    if (!initialized) { return tl::make_unexpected(initialized.error()); }

  public:
    using Base           = IIMU;
    using raw_value_type = int16_t;
    using bus_value_type = bus::IBus::element_type;

    BaseIMU_(config::IMUConfig config, bus::IBus *bus, logger::ILogger *logger) noexcept
        : bus_(bus), config_(config), logger_(logger) {
        /* Bind ST driver callbacks to this instance. */
        this->st_dev_ctx_.write_reg = BaseIMU_::st_write_callback_;
        this->st_dev_ctx_.read_reg  = BaseIMU_::st_read_callback_;
        this->st_dev_ctx_.mdelay    = BaseIMU_::st_delay_callback_;
        this->st_dev_ctx_.handle    = this;
    }

    BaseIMU_(const BaseIMU_ &)            = delete;
    BaseIMU_ &operator=(const BaseIMU_ &) = delete;

    BaseIMU_(BaseIMU_ &&other) noexcept
        : bus_(std::exchange(other.bus_, nullptr)), config_(std::move(other.config_)),
          logger_(std::exchange(other.logger_, nullptr)),
          st_dev_ctx_(std::exchange(other.st_dev_ctx_, {})),
          is_initialized_(std::exchange(other.is_initialized_, false)) {
        this->st_dev_ctx_.handle = this; /* Re-bind handle after move. */
    }

    BaseIMU_ &operator=(BaseIMU_ &&other) noexcept {
        if (this != &other) {
            this->bus_               = std::exchange(other.bus_, nullptr);
            this->config_            = std::exchange(other.config_, {});
            this->logger_            = std::exchange(other.logger_, nullptr);
            this->st_dev_ctx_        = std::exchange(other.st_dev_ctx_, {});
            this->st_dev_ctx_.handle = this; /* Re-bind handle. */
            this->is_initialized_    = std::exchange(other.is_initialized_, false);
        }
        return *this;
    }

    ~BaseIMU_() noexcept override = default;

    virtual tl::expected<void, IMUError> init() noexcept override {
        /* Check config sanity. */
        if (!this->config_.is_valid()) {
            this->logger_->error("invalid configuration");
            return tl::make_unexpected(IMUError::INVALID_CONFIG);
        }

        /* Check sensor identity. */
        auto whoami = this->get_is_whoami_valid();
        if (!whoami) {
            this->logger_->error("failed to read WHOAMI register: {}", whoami.error());
            return tl::make_unexpected(whoami.error());
        }
        if (!whoami.value()) {
            this->logger_->error(
                "invalid WHOAMI value: expected {:#x}", this->config_.whoami_value
            );
            return tl::make_unexpected(IMUError::INVALID_DEVICE);
        }

        /* Reset hardware. */
        auto res = this->reset_hardware();
        if (!res) {
            this->logger_->error("failed to reset hardware: {}", res.error());
            return res;
        }

        /* Enable Block Data Update (Write explicitly to IMU context) */
        lsm9ds1_ctrl_reg8_t ctrl_reg8;
        if (lsm9ds1_read_reg(&this->st_dev_ctx_, LSM9DS1_CTRL_REG8, (uint8_t *)&ctrl_reg8, 1)
            != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        ctrl_reg8.bdu = this->config_.enable_bdu ? PROPERTY_ENABLE : PROPERTY_DISABLE;
        if (lsm9ds1_write_reg(&this->st_dev_ctx_, LSM9DS1_CTRL_REG8, (uint8_t *)&ctrl_reg8, 1)
            != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        /* Set Output Data Rate (For both XL and GY) */
        if (lsm9ds1_imu_data_rate_set(&this->st_dev_ctx_, this->config_.imu_odr) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        /* Set full scale */
        if (lsm9ds1_xl_full_scale_set(&this->st_dev_ctx_, this->config_.accel_range) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        if (lsm9ds1_gy_full_scale_set(&this->st_dev_ctx_, this->config_.gyro_range) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        /* Configure filtering chain */
        if (lsm9ds1_xl_filter_aalias_bandwidth_set(&this->st_dev_ctx_, LSM9DS1_AUTO) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        if (lsm9ds1_xl_filter_lp_bandwidth_set(&this->st_dev_ctx_, this->config_.accel_bw) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        if (lsm9ds1_xl_filter_out_path_set(&this->st_dev_ctx_, LSM9DS1_LP_OUT) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        if (lsm9ds1_gy_filter_lp_bandwidth_set(&this->st_dev_ctx_, this->config_.gyro_bw) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        if (lsm9ds1_gy_filter_hp_bandwidth_set(&this->st_dev_ctx_, LSM9DS1_HP_ULTRA_LIGHT) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        if (lsm9ds1_gy_filter_out_path_set(&this->st_dev_ctx_, this->config_.gyro_out_path) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        this->is_initialized_ = true;
        return {};
    }

    tl::expected<void, IMUError> get_accel_data(accel_measurement_type &accel
    ) noexcept override final {
        ensure_initialized_hook();

        if (!this->accel_cached_) {
            auto res = this->read_gyro_accel_burst_();
            if (!res) { return tl::make_unexpected(res.error()); }
            this->gyro_cached_ = true;
        }
        this->accel_cached_            = false;
        const raw_value_type *data_raw = this->accel_raw_;

        /* Convert to mg using ST PID, then convert to g */
        switch (this->config_.accel_range) {
        case LSM9DS1_2g:
            accel[0] = lsm9ds1_from_fs2g_to_mg(data_raw[0]) / 1000.0f;
            accel[1] = lsm9ds1_from_fs2g_to_mg(data_raw[1]) / 1000.0f;
            accel[2] = lsm9ds1_from_fs2g_to_mg(data_raw[2]) / 1000.0f;
            break;
        case LSM9DS1_4g:
            accel[0] = lsm9ds1_from_fs4g_to_mg(data_raw[0]) / 1000.0f;
            accel[1] = lsm9ds1_from_fs4g_to_mg(data_raw[1]) / 1000.0f;
            accel[2] = lsm9ds1_from_fs4g_to_mg(data_raw[2]) / 1000.0f;
            break;
        case LSM9DS1_8g:
            accel[0] = lsm9ds1_from_fs8g_to_mg(data_raw[0]) / 1000.0f;
            accel[1] = lsm9ds1_from_fs8g_to_mg(data_raw[1]) / 1000.0f;
            accel[2] = lsm9ds1_from_fs8g_to_mg(data_raw[2]) / 1000.0f;
            break;
        case LSM9DS1_16g:
            accel[0] = lsm9ds1_from_fs16g_to_mg(data_raw[0]) / 1000.0f;
            accel[1] = lsm9ds1_from_fs16g_to_mg(data_raw[1]) / 1000.0f;
            accel[2] = lsm9ds1_from_fs16g_to_mg(data_raw[2]) / 1000.0f;
            break;
        default:
            return tl::make_unexpected(IMUError::INVALID_CONFIG);
        }

        /* The chip's accel/gyro frame is left-handed; negating Z on both gives the
         * right-handed frame IIMU promises. */
        accel[2] = -accel[2];

        return {};
    }

    tl::expected<void, IMUError> get_gyro_data(gyro_measurement_type &gyro
    ) noexcept override final {
        ensure_initialized_hook();

        if (!this->gyro_cached_) {
            auto res = this->read_gyro_accel_burst_();
            if (!res) { return tl::make_unexpected(res.error()); }
            this->accel_cached_ = true;
        }
        this->gyro_cached_             = false;
        const raw_value_type *data_raw = this->gyro_raw_;

        /* Convert to mdps using ST PID, then to deg/s */
        switch (this->config_.gyro_range) {
        case LSM9DS1_245dps:
            gyro[0] = lsm9ds1_from_fs245dps_to_mdps(data_raw[0]) / 1000.0f;
            gyro[1] = lsm9ds1_from_fs245dps_to_mdps(data_raw[1]) / 1000.0f;
            gyro[2] = lsm9ds1_from_fs245dps_to_mdps(data_raw[2]) / 1000.0f;
            break;
        case LSM9DS1_500dps:
            gyro[0] = lsm9ds1_from_fs500dps_to_mdps(data_raw[0]) / 1000.0f;
            gyro[1] = lsm9ds1_from_fs500dps_to_mdps(data_raw[1]) / 1000.0f;
            gyro[2] = lsm9ds1_from_fs500dps_to_mdps(data_raw[2]) / 1000.0f;
            break;
        case LSM9DS1_2000dps:
            gyro[0] = lsm9ds1_from_fs2000dps_to_mdps(data_raw[0]) / 1000.0f;
            gyro[1] = lsm9ds1_from_fs2000dps_to_mdps(data_raw[1]) / 1000.0f;
            gyro[2] = lsm9ds1_from_fs2000dps_to_mdps(data_raw[2]) / 1000.0f;
            break;
        default:
            return tl::make_unexpected(IMUError::INVALID_CONFIG);
        }

        gyro[2] = -gyro[2];

        return {};
    }

    tl::expected<void, IMUError> get_mag_data([[maybe_unused]] mag_measurement_type &mag
    ) noexcept override final {
        ensure_initialized_hook();
        /* Magnetometer wrapper logic is excluded on purpose per request constraints. */
        return tl::make_unexpected(IMUError::UNSUPPORTED_OPERATION);
    }

    [[nodiscard]] tl::expected<value_type, IMUError> get_temperature_c() noexcept override {
        ensure_initialized_hook();
        int16_t raw = 0;
        if (lsm9ds1_temperature_raw_get(&this->st_dev_ctx_, &raw) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        return lsm9ds1_from_lsb_to_celsius(raw);
    }

    [[nodiscard]] tl::expected<bool, IMUError> get_is_data_ready() noexcept override {
        ensure_initialized_hook();
        uint8_t xlda = 0, gda = 0;

        /* Bypass composite logic requiring the unused Mag context by reading IMU flags natively */
        if (lsm9ds1_xl_flag_data_ready_get(&this->st_dev_ctx_, &xlda) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        if (lsm9ds1_gy_flag_data_ready_get(&this->st_dev_ctx_, &gda) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        return (xlda || gda);
    }

    [[nodiscard]] tl::expected<bool, IMUError> get_is_gyro_data_ready() noexcept override {
        ensure_initialized_hook();
        uint8_t gda = 0;
        if (lsm9ds1_gy_flag_data_ready_get(&this->st_dev_ctx_, &gda) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        return gda != 0;
    }

    [[nodiscard]] tl::expected<RawScale, IMUError> get_raw_scale() const noexcept override {
        RawScale scale{};
        switch (this->config_.accel_range) {
        case LSM9DS1_2g:
            scale.accel_g_per_lsb = lsm9ds1_from_fs2g_to_mg(1) / 1000.0f;
            break;
        case LSM9DS1_4g:
            scale.accel_g_per_lsb = lsm9ds1_from_fs4g_to_mg(1) / 1000.0f;
            break;
        case LSM9DS1_8g:
            scale.accel_g_per_lsb = lsm9ds1_from_fs8g_to_mg(1) / 1000.0f;
            break;
        case LSM9DS1_16g:
            scale.accel_g_per_lsb = lsm9ds1_from_fs16g_to_mg(1) / 1000.0f;
            break;
        default:
            return tl::make_unexpected(IMUError::INVALID_CONFIG);
        }
        switch (this->config_.gyro_range) {
        case LSM9DS1_245dps:
            scale.gyro_dps_per_lsb = lsm9ds1_from_fs245dps_to_mdps(1) / 1000.0f;
            break;
        case LSM9DS1_500dps:
            scale.gyro_dps_per_lsb = lsm9ds1_from_fs500dps_to_mdps(1) / 1000.0f;
            break;
        case LSM9DS1_2000dps:
            scale.gyro_dps_per_lsb = lsm9ds1_from_fs2000dps_to_mdps(1) / 1000.0f;
            break;
        default:
            return tl::make_unexpected(IMUError::INVALID_CONFIG);
        }
        return scale;
    }

    tl::expected<void, IMUError> enable_fifo() noexcept override {
        ensure_initialized_hook();
        /* Bypass empties the FIFO before streaming starts. */
        if (lsm9ds1_fifo_mode_set(&this->st_dev_ctx_, LSM9DS1_BYPASS_MODE) != 0
            || lsm9ds1_fifo_mode_set(&this->st_dev_ctx_, LSM9DS1_STREAM_MODE) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        /* The datasheet says to drop the first sample after switching FIFO mode. */
        this->fifo_discard_ = true;
        return {};
    }

    tl::expected<std::size_t, IMUError> read_fifo(span<RawSample> dst, bool &overrun) noexcept override {
        ensure_initialized_hook();
        lsm9ds1_fifo_src_t src;
        if (lsm9ds1_read_reg(&this->st_dev_ctx_, LSM9DS1_FIFO_SRC, (uint8_t *)&src, 1) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        overrun = src.ovrn != 0;

        const std::size_t available = std::min<std::size_t>(src.fss, dst.size());
        std::size_t count           = 0;
        for (std::size_t i = 0; i < available; ++i) {
            /* One slot holds a gyro and an accel sample; the burst pops both. */
            auto res = this->read_gyro_accel_burst_();
            if (!res) { return tl::make_unexpected(res.error()); }
            if (this->fifo_discard_) {
                this->fifo_discard_ = false;
                continue;
            }
            RawSample &sample = dst.data()[count++];
            for (int k = 0; k < 3; ++k) {
                sample.accel[k] = this->accel_raw_[k];
                sample.gyro[k]  = this->gyro_raw_[k];
            }
            sample.accel[2] = negate_(sample.accel[2]);
            sample.gyro[2]  = negate_(sample.gyro[2]);
        }
        return count;
    }

    [[nodiscard]] tl::expected<bool, IMUError> get_is_whoami_valid() noexcept {
        uint8_t whoami = 0;
        if (lsm9ds1_read_reg(&this->st_dev_ctx_, LSM9DS1_WHO_AM_I, &whoami, 1) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        return whoami == this->config_.whoami_value;
    }

    virtual tl::expected<void, IMUError> reset_hardware() noexcept {
        lsm9ds1_ctrl_reg8_t rst_reg;

        /* Read register to maintain structural bits untouched except for reset flag */
        if (lsm9ds1_read_reg(&this->st_dev_ctx_, LSM9DS1_CTRL_REG8, (uint8_t *)&rst_reg, 1) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        rst_reg.sw_reset = PROPERTY_ENABLE;

        if (lsm9ds1_write_reg(&this->st_dev_ctx_, LSM9DS1_CTRL_REG8, (uint8_t *)&rst_reg, 1) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        do {
            if (lsm9ds1_read_reg(&this->st_dev_ctx_, LSM9DS1_CTRL_REG8, (uint8_t *)&rst_reg, 1)
                != 0) {
                return tl::make_unexpected(IMUError::BUS_ERROR);
            }
            platform_sleep_ms(5);
        } while (rst_reg.sw_reset);

        return {};
    }

  protected:
    bus::IBus *bus_;
    config::IMUConfig config_;
    logger::ILogger *logger_;
    stmdev_ctx_t st_dev_ctx_;
    bool is_initialized_ = false;

    virtual tl::expected<void, IMUError> write_registers_(
        bus::IBus::reg_type reg, span<const bus_value_type> data
    ) noexcept = 0;
    virtual tl::expected<void, IMUError> read_registers_(
        bus::IBus::reg_type reg, span<bus_value_type> data
    ) noexcept = 0;

    tl::expected<void, IMUError> check_initialized_() const noexcept {
        if (!this->is_initialized_) { return tl::make_unexpected(IMUError::NOT_INITIALIZED); }
        return {};
    }

  private:
    raw_value_type gyro_raw_[3]  = {0, 0, 0};
    raw_value_type accel_raw_[3] = {0, 0, 0};
    bool gyro_cached_            = false;
    bool accel_cached_           = false;
    bool fifo_discard_           = false;

    static constexpr raw_value_type negate_(raw_value_type v) noexcept {
        return v == std::numeric_limits<raw_value_type>::min() ? std::numeric_limits<raw_value_type>::max()
                                                               : static_cast<raw_value_type>(-v);
    }

    /**
     * With address auto-increment (CTRL_REG8.IF_ADD_INC, enabled by default and
     * preserved by init), a read starting at OUT_X_L_G (0x18) runs through the
     * gyro output block and then continues at OUT_X_L_XL (0x28), skipping the
     * control registers in between, and wraps back to the gyro after the accel
     * block. So one 12-byte read returns both blocks. Callers ask for accel and
     * gyro back to back each sample: the first call reads the bus, the second is
     * served from this read.
     */
    tl::expected<void, IMUError> read_gyro_accel_burst_() noexcept {
        /* Gyro XYZ at [0..5], accel XYZ at [6..11]. */
        uint8_t buf[12];
        if (lsm9ds1_read_reg(&this->st_dev_ctx_, LSM9DS1_OUT_X_L_G, buf, sizeof(buf)) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        for (int i = 0; i < 3; i++) {
            this->gyro_raw_[i]  = static_cast<raw_value_type>((buf[2 * i + 1] << 8) | buf[2 * i]);
            this->accel_raw_[i] = static_cast<raw_value_type>((buf[2 * i + 7] << 8) | buf[2 * i + 6]);
        }
        return {};
    }

    static int32_t st_write_callback_(
        void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len
    ) {
        auto *imu = static_cast<BaseIMU_ *>(handle);
        auto res  = imu->write_registers_(reg, {bufp, len});
        return res ? 0 : -1;
    }

    static int32_t st_read_callback_(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
        auto *imu = static_cast<BaseIMU_ *>(handle);
        auto res  = imu->read_registers_(reg, {bufp, len});
        return res ? 0 : -1;
    }

    static void st_delay_callback_(uint32_t ms) { platform_sleep_ms(ms); }

#undef ensure_initialized_hook
};

} /* namespace imu::lsm9ds1::base */

#endif /* IMU_LSM9DS1_BASE_HPP */
