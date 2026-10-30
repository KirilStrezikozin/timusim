/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Checks that an IIMU sample stream follows the convention in imu/iimu.hpp.
 *
 * - RestCheck: while still, the mean accel is 1 g and points along the axis
 *   expected to be up.
 * - HandednessCheck: while rotating, the unit accel vector u turns as
 *   du/dt = -omega x u. A left-handed frame reverses the sign; a gyro in the wrong
 *   unit changes the gain. A negated accel passes this check but fails RestCheck.
 */

#ifndef IMU_CONFORMANCE_HPP
#define IMU_CONFORMANCE_HPP

#include <cmath>
#include <cstdint>

#include "math-utils/math-utils.hpp"

namespace imu::conformance {

using vector_type = Vector3<float>;

struct RestSettings {
    /* Still means a fast running mean stays close to a slow one, so neither a gyro
     * zero-rate offset nor sample noise counts as motion. */
    float still_dps           = 2.0F;
    float still_g             = 0.03F;
    float fast_alpha          = 0.2F;
    float slow_alpha          = 0.01F;
    float norm_tolerance_g    = 0.05F;
    float up_tolerance_deg    = 10.0F;
    std::uint32_t min_samples = 200;
};

struct RestResult {
    std::uint32_t samples = 0;
    vector_type mean_accel_g{};
    vector_type mean_gyro_dps{};
    float norm_error_g = 0.0F;
    float up_error_deg = 0.0F;
    bool pass          = false;
};

/* Uses the first still stretch of at least min_samples, in any orientation. */
class RestCheck {
  public:
    /* A zero expected_up skips the up-axis test. */
    explicit RestCheck(vector_type expected_up, RestSettings settings = {}) noexcept
        : expected_up_(expected_up.normalized()), settings_(settings) {}

    void update(const vector_type &accel_g, const vector_type &gyro_dps) noexcept {
        if (this->done_) { return; }

        if (!this->has_running_) {
            this->fast_accel_  = this->slow_accel_ = accel_g;
            this->fast_gyro_   = this->slow_gyro_  = gyro_dps;
            this->has_running_ = true;
        }
        const float f     = this->settings_.fast_alpha;
        const float s     = this->settings_.slow_alpha;
        this->fast_accel_ = this->fast_accel_ * (1.0F - f) + accel_g * f;
        this->fast_gyro_  = this->fast_gyro_ * (1.0F - f) + gyro_dps * f;
        this->slow_accel_ = this->slow_accel_ * (1.0F - s) + accel_g * s;
        this->slow_gyro_  = this->slow_gyro_ * (1.0F - s) + gyro_dps * s;

        const bool still = (this->fast_gyro_ - this->slow_gyro_).magnitude() <= this->settings_.still_dps
                        && (this->fast_accel_ - this->slow_accel_).magnitude() <= this->settings_.still_g;
        if (!still) {
            if (this->samples_ >= this->settings_.min_samples) {
                this->done_ = true;
            } else {
                this->sum_accel_ = vector_type{};
                this->sum_gyro_  = vector_type{};
                this->samples_   = 0;
            }
            return;
        }

        this->sum_accel_ += accel_g;
        this->sum_gyro_ += gyro_dps;
        ++this->samples_;
    }

    [[nodiscard]] RestResult result() const noexcept {
        RestResult r;
        r.samples = this->samples_;
        if (this->samples_ == 0) { return r; }

        const float n   = static_cast<float>(this->samples_);
        r.mean_accel_g  = this->sum_accel_ / n;
        r.mean_gyro_dps = this->sum_gyro_ / n;
        r.norm_error_g  = r.mean_accel_g.magnitude() - 1.0F;

        const bool check_up = this->expected_up_.magnitude() > 0.0F;
        if (check_up) {
            const float cos_up = std::fmax(
                -1.0F, std::fmin(1.0F, r.mean_accel_g.normalized().dot(this->expected_up_))
            );
            r.up_error_deg = std::acos(cos_up) * rad_to_deg_;
        }
        r.pass = r.samples >= this->settings_.min_samples
              && std::fabs(r.norm_error_g) <= this->settings_.norm_tolerance_g
              && (!check_up || r.up_error_deg <= this->settings_.up_tolerance_deg);
        return r;
    }

    /* Gyro zero-rate offset seen while still; zero until a still stretch is found. */
    [[nodiscard]] vector_type gyro_bias_dps() const noexcept {
        return this->samples_ >= this->settings_.min_samples
                 ? this->sum_gyro_ / static_cast<float>(this->samples_)
                 : vector_type{};
    }

  private:
    static constexpr float rad_to_deg_ = 57.2957795F;

    vector_type expected_up_;
    RestSettings settings_;

    vector_type fast_accel_{};
    vector_type fast_gyro_{};
    vector_type slow_accel_{};
    vector_type slow_gyro_{};
    bool has_running_ = false;

    vector_type sum_accel_{};
    vector_type sum_gyro_{};
    std::uint32_t samples_ = 0;
    bool done_             = false;
};

struct HandednessSettings {
    /* Samples averaged per block; differencing blocks instead of samples keeps
     * accel noise from swamping du/dt at high ODR. */
    std::uint32_t block_samples = 24;
    float min_rate_dps          = 20.0F;
    float max_linear_g          = 0.1F;
    /* Hand-held rotation adds lever-arm acceleration: real boards turned at the wrist
     * score about 0.75, a left-handed frame -1. */
    float min_correlation       = 0.5F;
    float gain_tolerance        = 0.15F;
    std::uint32_t min_blocks    = 50;
    float accel_only_factor     = 3.0F;
    /* More accel-only blocks than this fraction of scored ones fails the check. */
    float max_accel_only_ratio  = 0.5F;
};

struct HandednessResult {
    std::uint32_t blocks = 0;
    /* Correlation of measured du/dt with -omega x u: +1 right-handed, -1 left-handed. */
    float correlation = 0.0F;
    /* Measured over predicted du/dt: 1 when the gyro reports dps. */
    float gain = 0.0F;
    std::uint32_t accel_only_blocks = 0;
    bool pass  = false;
};

class HandednessCheck {
  public:
    explicit HandednessCheck(HandednessSettings settings = {}) noexcept : settings_(settings) {}

    void update(const vector_type &accel_g, const vector_type &gyro_dps, float dt_s) noexcept {
        this->block_accel_ += accel_g;
        this->block_gyro_ += gyro_dps;
        this->block_dt_ += dt_s;
        if (++this->block_count_ < this->settings_.block_samples) { return; }

        const float n           = static_cast<float>(this->block_count_);
        const Block block       = {this->block_accel_ / n, this->block_gyro_ / n, this->block_dt_};
        this->block_accel_      = vector_type{};
        this->block_gyro_       = vector_type{};
        this->block_dt_         = 0.0F;
        this->block_count_      = 0;

        if (this->has_previous_) { this->accumulate_(this->previous_, block); }
        this->previous_     = block;
        this->has_previous_ = true;
    }

    [[nodiscard]] HandednessResult result() const noexcept {
        HandednessResult r;
        r.blocks            = this->blocks_;
        r.accel_only_blocks = this->accel_only_blocks_;
        if (this->blocks_ == 0 || this->sum_pp_ <= 0.0F || this->sum_mm_ <= 0.0F) { return r; }

        r.correlation = this->sum_mp_ / std::sqrt(this->sum_mm_ * this->sum_pp_);
        r.gain        = this->sum_mp_ / this->sum_pp_;
        r.pass        = r.blocks >= this->settings_.min_blocks
                && r.correlation >= this->settings_.min_correlation
                && std::fabs(r.gain - 1.0F) <= this->settings_.gain_tolerance
                && static_cast<float>(r.accel_only_blocks)
                       <= this->settings_.max_accel_only_ratio * static_cast<float>(r.blocks);
        return r;
    }

  private:
    static constexpr float deg_to_rad_ = 0.0174532925F;

    struct Block {
        vector_type accel_g;
        vector_type gyro_dps;
        float dt_s;
    };

    void accumulate_(const Block &a, const Block &b) noexcept {
        const float max_linear = this->settings_.max_linear_g;
        if (std::fabs(a.accel_g.magnitude() - 1.0F) > max_linear
            || std::fabs(b.accel_g.magnitude() - 1.0F) > max_linear) {
            return;
        }

        const vector_type omega = (a.gyro_dps + b.gyro_dps) * (0.5F * deg_to_rad_);
        const vector_type ua    = a.accel_g.normalized();
        const vector_type ub    = b.accel_g.normalized();
        const float dt          = 0.5F * (a.dt_s + b.dt_s);

        const vector_type measured  = (ub - ua) / dt;
        const vector_type predicted = omega.cross((ua + ub).normalized()) * -1.0F;

        /* Rotation the accel sees but the gyro doesn't points at a gyro scale or
         * unit error; it is counted, not scored, so hand jitter can't dilute the score. */
        const float min_rate = this->settings_.min_rate_dps * deg_to_rad_;
        if (predicted.magnitude() < min_rate) {
            if (measured.magnitude() >= this->settings_.accel_only_factor * min_rate) {
                ++this->accel_only_blocks_;
            }
            return;
        }

        this->sum_mp_ += measured.dot(predicted);
        this->sum_mm_ += measured.dot(measured);
        this->sum_pp_ += predicted.dot(predicted);
        ++this->blocks_;
    }

    HandednessSettings settings_;

    vector_type block_accel_{};
    vector_type block_gyro_{};
    float block_dt_            = 0.0F;
    std::uint32_t block_count_ = 0;

    Block previous_{};
    bool has_previous_ = false;

    float sum_mp_          = 0.0F;
    float sum_mm_          = 0.0F;
    float sum_pp_                     = 0.0F;
    std::uint32_t blocks_             = 0;
    std::uint32_t accel_only_blocks_  = 0;
};

} /* namespace imu::conformance */

#endif /* IMU_CONFORMANCE_HPP */
