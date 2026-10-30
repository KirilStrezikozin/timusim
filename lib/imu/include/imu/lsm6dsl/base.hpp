/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * LSM6DSL Base Implementation utilizing ST's PID.
 *
 * -----------------------------------------------------------------------------
 * This file incorporates and derives logic from STMicroelectronics'
 * lsm6dsl_STdC/examples/read_data_polling.c example available at:
 * <https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsl_STdC/examples/lsm6dsl_read_data_polling.c>,
 * which is subject to the following license:
 *
 * Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 * opensource.org/licenses/BSD-3-Clause
 * -----------------------------------------------------------------------------
 */

#ifndef IMU_LSM6DSL_BASE_HPP
#define IMU_LSM6DSL_BASE_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>

#include <lsm6dsl_reg.h>
#include <tl/expected.hpp>

#include "bus/ibus.hpp"
#include "imu/iimu.hpp"
#include "imu/lsm6dsl/config.hpp"
#include "log/ilog.hpp"
#include "span.hpp"
#include "time/sleep.h"

namespace imu::lsm6dsl::base {

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

        /* Enable Block Data Update */
        if (lsm6dsl_block_data_update_set(
                &this->st_dev_ctx_, this->config_.enable_bdu ? PROPERTY_ENABLE : PROPERTY_DISABLE
            )
            != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        /* Set Output Data Rate */
        if (lsm6dsl_xl_data_rate_set(&this->st_dev_ctx_, this->config_.accel_odr) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        if (lsm6dsl_gy_data_rate_set(&this->st_dev_ctx_, this->config_.gyro_odr) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        /* Set full scale */
        if (lsm6dsl_xl_full_scale_set(&this->st_dev_ctx_, this->config_.accel_range) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        if (lsm6dsl_gy_full_scale_set(&this->st_dev_ctx_, this->config_.gyro_range) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        /* Configure filtering chain */
        /* Accelerometer - analog filter */
        if (lsm6dsl_xl_filter_analog_set(&this->st_dev_ctx_, this->config_.accel_bw) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        /* Accelerometer - LPF1 + LPF2 path */
        if (lsm6dsl_xl_lp2_bandwidth_set(&this->st_dev_ctx_, this->config_.accel_lp2_bw) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        /* */
        /* Alternative. Accelerometer - LPF1 path (LPF2 not used) */
        // lsm6dsl_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSL_XL_LP1_ODR_DIV_4);
        /* */
        /* Alternative. Accelerometer - High Pass / Slope path */
        // lsm6dsl_xl_reference_mode_set(&this->st_dev_ctx_, PROPERTY_DISABLE);
        // lsm6dsl_xl_hp_bandwidth_set(&this->st_dev_ctx_, LSM6DSL_XL_HP_ODR_DIV_100);
        /* */

        /* Gyroscope - filtering chain */
        if (lsm6dsl_gy_band_pass_set(&this->st_dev_ctx_, this->config_.gyro_bw) != 0) {
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
        this->accel_cached_ = false;
        const raw_value_type *data_raw = this->accel_raw_;

        /* Convert to mg using ST PID, then convert to g */
        switch (this->config_.accel_range) {
        case LSM6DSL_2g:
            accel[0] = lsm6dsl_from_fs2g_to_mg(data_raw[0]) / 1000.0f;
            accel[1] = lsm6dsl_from_fs2g_to_mg(data_raw[1]) / 1000.0f;
            accel[2] = lsm6dsl_from_fs2g_to_mg(data_raw[2]) / 1000.0f;
            break;
        case LSM6DSL_4g:
            accel[0] = lsm6dsl_from_fs4g_to_mg(data_raw[0]) / 1000.0f;
            accel[1] = lsm6dsl_from_fs4g_to_mg(data_raw[1]) / 1000.0f;
            accel[2] = lsm6dsl_from_fs4g_to_mg(data_raw[2]) / 1000.0f;
            break;
        case LSM6DSL_8g:
            accel[0] = lsm6dsl_from_fs8g_to_mg(data_raw[0]) / 1000.0f;
            accel[1] = lsm6dsl_from_fs8g_to_mg(data_raw[1]) / 1000.0f;
            accel[2] = lsm6dsl_from_fs8g_to_mg(data_raw[2]) / 1000.0f;
            break;
        case LSM6DSL_16g:
            accel[0] = lsm6dsl_from_fs16g_to_mg(data_raw[0]) / 1000.0f;
            accel[1] = lsm6dsl_from_fs16g_to_mg(data_raw[1]) / 1000.0f;
            accel[2] = lsm6dsl_from_fs16g_to_mg(data_raw[2]) / 1000.0f;
            break;
        default:
            return tl::make_unexpected(IMUError::INVALID_CONFIG);
        }

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
        this->gyro_cached_ = false;
        const raw_value_type *data_raw = this->gyro_raw_;

        /* Convert to mdps using ST PID, then to deg/s */
        switch (this->config_.gyro_range) {
        case LSM6DSL_125dps:
            gyro[0] = lsm6dsl_from_fs125dps_to_mdps(data_raw[0]) / 1000.0f;
            gyro[1] = lsm6dsl_from_fs125dps_to_mdps(data_raw[1]) / 1000.0f;
            gyro[2] = lsm6dsl_from_fs125dps_to_mdps(data_raw[2]) / 1000.0f;
            break;
        case LSM6DSL_250dps:
            gyro[0] = lsm6dsl_from_fs250dps_to_mdps(data_raw[0]) / 1000.0f;
            gyro[1] = lsm6dsl_from_fs250dps_to_mdps(data_raw[1]) / 1000.0f;
            gyro[2] = lsm6dsl_from_fs250dps_to_mdps(data_raw[2]) / 1000.0f;
            break;
        case LSM6DSL_500dps:
            gyro[0] = lsm6dsl_from_fs500dps_to_mdps(data_raw[0]) / 1000.0f;
            gyro[1] = lsm6dsl_from_fs500dps_to_mdps(data_raw[1]) / 1000.0f;
            gyro[2] = lsm6dsl_from_fs500dps_to_mdps(data_raw[2]) / 1000.0f;
            break;
        case LSM6DSL_1000dps:
            gyro[0] = lsm6dsl_from_fs1000dps_to_mdps(data_raw[0]) / 1000.0f;
            gyro[1] = lsm6dsl_from_fs1000dps_to_mdps(data_raw[1]) / 1000.0f;
            gyro[2] = lsm6dsl_from_fs1000dps_to_mdps(data_raw[2]) / 1000.0f;
            break;
        case LSM6DSL_2000dps:
            gyro[0] = lsm6dsl_from_fs2000dps_to_mdps(data_raw[0]) / 1000.0f;
            gyro[1] = lsm6dsl_from_fs2000dps_to_mdps(data_raw[1]) / 1000.0f;
            gyro[2] = lsm6dsl_from_fs2000dps_to_mdps(data_raw[2]) / 1000.0f;
            break;
        default:
            return tl::make_unexpected(IMUError::INVALID_CONFIG);
        }

        return {};
    }

    tl::expected<void, IMUError> get_mag_data([[maybe_unused]] mag_measurement_type &mag
    ) noexcept override final {
        ensure_initialized_hook();
        /* LSM6DSL does not have a built-in magnetometer. */
        return tl::make_unexpected(IMUError::UNSUPPORTED_OPERATION);
    }

    [[nodiscard]] tl::expected<value_type, IMUError> get_temperature_c() noexcept override {
        ensure_initialized_hook();
        int16_t raw = 0;
        if (lsm6dsl_temperature_raw_get(&this->st_dev_ctx_, &raw) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        return lsm6dsl_from_lsb_to_celsius(raw);
    }

    [[nodiscard]] tl::expected<bool, IMUError> get_is_data_ready() noexcept override {
        ensure_initialized_hook();
        lsm6dsl_status_reg_t reg;
        if (lsm6dsl_status_reg_get(&this->st_dev_ctx_, &reg) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        return reg.xlda || reg.gda;
    }

    [[nodiscard]] tl::expected<bool, IMUError> get_is_gyro_data_ready() noexcept override {
        ensure_initialized_hook();
        lsm6dsl_status_reg_t reg;
        if (lsm6dsl_status_reg_get(&this->st_dev_ctx_, &reg) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        return reg.gda != 0;
    }

    [[nodiscard]] tl::expected<RawScale, IMUError> get_raw_scale() const noexcept override {
        RawScale scale{};
        switch (this->config_.accel_range) {
        case LSM6DSL_2g:
            scale.accel_g_per_lsb = lsm6dsl_from_fs2g_to_mg(1) / 1000.0f;
            break;
        case LSM6DSL_4g:
            scale.accel_g_per_lsb = lsm6dsl_from_fs4g_to_mg(1) / 1000.0f;
            break;
        case LSM6DSL_8g:
            scale.accel_g_per_lsb = lsm6dsl_from_fs8g_to_mg(1) / 1000.0f;
            break;
        case LSM6DSL_16g:
            scale.accel_g_per_lsb = lsm6dsl_from_fs16g_to_mg(1) / 1000.0f;
            break;
        default:
            return tl::make_unexpected(IMUError::INVALID_CONFIG);
        }
        switch (this->config_.gyro_range) {
        case LSM6DSL_125dps:
            scale.gyro_dps_per_lsb = lsm6dsl_from_fs125dps_to_mdps(1) / 1000.0f;
            break;
        case LSM6DSL_250dps:
            scale.gyro_dps_per_lsb = lsm6dsl_from_fs250dps_to_mdps(1) / 1000.0f;
            break;
        case LSM6DSL_500dps:
            scale.gyro_dps_per_lsb = lsm6dsl_from_fs500dps_to_mdps(1) / 1000.0f;
            break;
        case LSM6DSL_1000dps:
            scale.gyro_dps_per_lsb = lsm6dsl_from_fs1000dps_to_mdps(1) / 1000.0f;
            break;
        case LSM6DSL_2000dps:
            scale.gyro_dps_per_lsb = lsm6dsl_from_fs2000dps_to_mdps(1) / 1000.0f;
            break;
        default:
            return tl::make_unexpected(IMUError::INVALID_CONFIG);
        }
        return scale;
    }

    tl::expected<void, IMUError> enable_fifo() noexcept override {
        ensure_initialized_hook();
        /* read_fifo expects the pattern Gx Gy Gz XLx XLy XLz, which needs equal rates. */
        if (static_cast<int>(this->config_.accel_odr) != static_cast<int>(this->config_.gyro_odr)) {
            return tl::make_unexpected(IMUError::INVALID_CONFIG);
        }
        const auto fifo_odr = static_cast<lsm6dsl_odr_fifo_t>(this->config_.gyro_odr);
        if (lsm6dsl_fifo_mode_set(&this->st_dev_ctx_, LSM6DSL_BYPASS_MODE) != 0
            || lsm6dsl_fifo_xl_batch_set(&this->st_dev_ctx_, LSM6DSL_FIFO_XL_NO_DEC) != 0
            || lsm6dsl_fifo_gy_batch_set(&this->st_dev_ctx_, LSM6DSL_FIFO_GY_NO_DEC) != 0
            || lsm6dsl_fifo_data_rate_set(&this->st_dev_ctx_, fifo_odr) != 0
            || lsm6dsl_fifo_mode_set(&this->st_dev_ctx_, LSM6DSL_STREAM_MODE) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        return {};
    }

    tl::expected<std::size_t, IMUError> read_fifo(span<RawSample> dst, bool &overrun) noexcept override {
        ensure_initialized_hook();
        constexpr std::size_t WORDS_PER_SAMPLE = 6;

        /* FIFO_STATUS1..4: unread words, overrun flag, index of the next word in the pattern. */
        uint8_t status[4];
        if (lsm6dsl_read_reg(&this->st_dev_ctx_, LSM6DSL_FIFO_STATUS1, status, sizeof(status)) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        std::size_t words   = status[0] | ((status[1] & 0x07U) << 8);
        overrun             = (status[1] & 0x40U) != 0;
        std::size_t pattern = status[2] | ((status[3] & 0x03U) << 8);

        /* After an overrun the next word can sit mid-sample; skip to the next Gx. */
        while (pattern != 0 && words > 0) {
            uint8_t skip[2];
            if (lsm6dsl_fifo_raw_data_get(&this->st_dev_ctx_, skip, sizeof(skip)) != 0) {
                return tl::make_unexpected(IMUError::BUS_ERROR);
            }
            --words;
            pattern = (pattern + 1) % WORDS_PER_SAMPLE;
        }

        const std::size_t count = std::min(words / WORDS_PER_SAMPLE, dst.size());
        for (std::size_t i = 0; i < count; ++i) {
            uint8_t buf[2 * WORDS_PER_SAMPLE];
            if (lsm6dsl_fifo_raw_data_get(&this->st_dev_ctx_, buf, sizeof(buf)) != 0) {
                return tl::make_unexpected(IMUError::BUS_ERROR);
            }
            RawSample &sample = dst.data()[i];
            for (int k = 0; k < 3; ++k) {
                sample.gyro[k]  = static_cast<raw_value_type>((buf[2 * k + 1] << 8) | buf[2 * k]);
                sample.accel[k] = static_cast<raw_value_type>((buf[2 * k + 7] << 8) | buf[2 * k + 6]);
            }
        }
        return count;
    }

    [[nodiscard]] tl::expected<bool, IMUError> get_is_whoami_valid() noexcept {
        uint8_t whoami = 0;
        if (lsm6dsl_device_id_get(&this->st_dev_ctx_, &whoami) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        return whoami == this->config_.whoami_value;
    }

    virtual tl::expected<void, IMUError> reset_hardware() noexcept {
        if (lsm6dsl_reset_set(&this->st_dev_ctx_, PROPERTY_ENABLE) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        uint8_t rst;
        do {
            if (lsm6dsl_reset_get(&this->st_dev_ctx_, &rst) != 0) {
                return tl::make_unexpected(IMUError::BUS_ERROR);
            }
            platform_sleep_ms(5);
        } while (rst);

        return {};
    }

  protected:
    bus::IBus *bus_;
    config::IMUConfig config_;
    logger::ILogger *logger_;
    stmdev_ctx_t st_dev_ctx_;
    bool is_initialized_ = false;

    /* Latest gyro and accel samples from one burst read; see read_gyro_accel_burst_(). */
    raw_value_type gyro_raw_[3]  = {0, 0, 0};
    raw_value_type accel_raw_[3] = {0, 0, 0};
    bool gyro_cached_            = false;
    bool accel_cached_           = false;

    /**
     * Gyro (OUTX_L_G, 0x22) and accel (OUTX_L_XL, 0x28) output registers are
     * contiguous, so one 12-byte auto-incrementing read returns both. Callers
     * ask for accel and gyro back to back each sample: the first call reads the
     * bus, the second is served from this read, saving a whole SPI transaction.
     */
    tl::expected<void, IMUError> read_gyro_accel_burst_() noexcept {
        uint8_t buf[12];
        if (lsm6dsl_read_reg(&this->st_dev_ctx_, LSM6DSL_OUTX_L_G, buf, sizeof(buf)) != 0) {
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }
        for (int i = 0; i < 3; i++) {
            this->gyro_raw_[i]  = static_cast<raw_value_type>((buf[2 * i + 1] << 8) | buf[2 * i]);
            this->accel_raw_[i] = static_cast<raw_value_type>((buf[2 * i + 7] << 8) | buf[2 * i + 6]);
        }
        return {};
    }

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

} /* namespace imu::lsm6dsl::base */

#endif /* IMU_LSM6DSL_BASE_HPP */
