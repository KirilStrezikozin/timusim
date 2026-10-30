/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * A simulated LSM9DS1 or LSM6DSL behind the IIMU interface. It runs the chip model up to
 * the virtual clock on every call, so a caller polling slower than ODR sees the
 * same decimation as on the real chip, and the FIFO path holds 32 samples and
 * flags overruns like the driver's stream mode.
 */

#ifndef CHIP_SIM_IMU_HPP
#define CHIP_SIM_IMU_HPP

#include <cstddef>
#include <deque>
#include <functional>
#include <optional>
#include <utility>

#include "chip/lsm6dsl.hpp"
#include "chip/lsm9ds1.hpp"
#include "chip/model.hpp"
#include "imu/iimu.hpp"
#include "time/virtual_clock.hpp"

namespace chip {

class SimIMU final : public imu::IIMU {
  public:
    using Config = lsm9ds1::Config;
    /* Ideal {specific force g, rate dps} in the IIMU frame at virtual time t_s. */
    using Truth = std::function<std::pair<Vec3, Vec3>(double t_s)>;

    static constexpr std::size_t FIFO_SLOTS = 32;

    /* Config is lsm9ds1::Config or lsm6dsl::Config. */
    template <typename ChipConfig>
    SimIMU(const ChipConfig &config, const Params &params, std::uint64_t seed, Truth truth)
        : path_(chip::path(config, params)), params_(params), seed_(seed), truth_(std::move(truth)) {}

    /* A fresh power-up: new turn-on bias draw from the same seed sequence. */
    tl::expected<void, imu::IMUError> init() noexcept override {
        if (!this->path_) { return tl::make_unexpected(this->path_.error()); }
        auto model = Model::make(*this->path_, this->params_, this->seed_++);
        if (!model) { return tl::make_unexpected(model.error()); }
        this->model_.emplace(std::move(model.value()));
        this->t0_s_ = now_s_();
        this->latest_.reset();
        this->fifo_.clear();
        this->fifo_mode_ = this->overrun_ = this->new_accel_ = this->new_gyro_ = false;
        return {};
    }

    tl::expected<void, imu::IMUError> get_gyro_data(gyro_measurement_type &dst) noexcept override {
        if (auto r = this->sync_(); !r) { return r; }
        if (!this->latest_) { return tl::make_unexpected(imu::IMUError::NOT_INITIALIZED); }
        const float lsb = this->model_->raw_scale().gyro_dps_per_lsb;
        const auto &g   = this->latest_->gyro;
        dst             = gyro_measurement_type(g[0] * lsb, g[1] * lsb, g[2] * lsb);
        this->new_gyro_ = false;
        return {};
    }

    tl::expected<void, imu::IMUError> get_accel_data(accel_measurement_type &dst) noexcept override {
        if (auto r = this->sync_(); !r) { return r; }
        if (!this->latest_) { return tl::make_unexpected(imu::IMUError::NOT_INITIALIZED); }
        const float lsb  = this->model_->raw_scale().accel_g_per_lsb;
        const auto &a    = this->latest_->accel;
        dst              = accel_measurement_type(a[0] * lsb, a[1] * lsb, a[2] * lsb);
        this->new_accel_ = false;
        return {};
    }

    tl::expected<void, imu::IMUError> get_mag_data([[maybe_unused]] mag_measurement_type &dst) noexcept override {
        return tl::make_unexpected(imu::IMUError::UNSUPPORTED_OPERATION);
    }

    tl::expected<bool, imu::IMUError> get_is_data_ready() noexcept override {
        if (auto r = this->sync_(); !r) { return tl::make_unexpected(r.error()); }
        return this->new_accel_ || this->new_gyro_;
    }

    tl::expected<bool, imu::IMUError> get_is_gyro_data_ready() noexcept override {
        if (auto r = this->sync_(); !r) { return tl::make_unexpected(r.error()); }
        return this->new_gyro_;
    }

    tl::expected<value_type, imu::IMUError> get_temperature_c() noexcept override {
        if (!this->model_) { return tl::make_unexpected(imu::IMUError::NOT_INITIALIZED); }
        return static_cast<value_type>(this->model_->temperature_c());
    }

    tl::expected<RawScale, imu::IMUError> get_raw_scale() const noexcept override {
        if (!this->model_) { return tl::make_unexpected(imu::IMUError::NOT_INITIALIZED); }
        return this->model_->raw_scale();
    }

    tl::expected<void, imu::IMUError> enable_fifo() noexcept override {
        if (auto r = this->sync_(); !r) { return r; }
        this->fifo_.clear();
        this->fifo_mode_ = true;
        this->overrun_   = false;
        return {};
    }

    tl::expected<std::size_t, imu::IMUError> read_fifo(span<RawSample> dst, bool &overrun) noexcept override {
        if (auto r = this->sync_(); !r) { return tl::make_unexpected(r.error()); }
        if (!this->fifo_mode_) { return tl::make_unexpected(imu::IMUError::NOT_INITIALIZED); }
        std::size_t n = 0;
        while (n < dst.size() && !this->fifo_.empty()) {
            dst[n++] = this->fifo_.front();
            this->fifo_.pop_front();
        }
        overrun        = this->overrun_;
        this->overrun_ = false;
        return n;
    }

    [[nodiscard]] const tl::expected<Path, imu::IMUError> &path() const { return this->path_; }

    [[nodiscard]] double odr_hz() const { return this->model_ ? this->model_->odr_hz() : 0.0; }

    /* Virtual time of output sample k since the last init; FIFO sample k when the FIFO was enabled right after init. */
    [[nodiscard]] double sample_time_s(std::uint64_t k) const { return this->t0_s_ + this->model_->output_time_s(k); }

  private:
    static double now_s_() { return static_cast<double>(vclock::now_us()) * 1e-6; }

    tl::expected<void, imu::IMUError> sync_() {
        if (!this->model_) { return tl::make_unexpected(imu::IMUError::NOT_INITIALIZED); }
        const double t0 = this->t0_s_;
        this->model_->advance(
            now_s_() - t0, [&](double t) { return this->truth_(t0 + t); },
            [&](const Output &out) {
                this->latest_    = out.raw;
                this->new_accel_ = this->new_gyro_ = true;
                if (!this->fifo_mode_) { return; }
                if (this->fifo_.size() == FIFO_SLOTS) {
                    this->fifo_.pop_front();
                    this->overrun_ = true;
                }
                this->fifo_.push_back(out.raw);
            }
        );
        return {};
    }

    tl::expected<Path, imu::IMUError> path_;
    Params params_;
    std::uint64_t seed_;
    Truth truth_;
    std::optional<Model> model_;
    double t0_s_ = 0.0;
    std::optional<RawSample> latest_;
    std::deque<RawSample> fifo_;
    bool fifo_mode_ = false;
    bool overrun_   = false;
    bool new_accel_ = false;
    bool new_gyro_  = false;
};

} /* namespace chip */

#endif /* CHIP_SIM_IMU_HPP */
