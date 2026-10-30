/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Accelerometer and gyroscope signal path shared by the chip models, from ideal
 * specific force and rate (IIMU frame, g and dps) to the raw codes the driver
 * reads from the FIFO.
 *
 * Per internal step, at oversample x ODR:
 *   gain and g-sensitivity -> + zero level, bias drift, white noise
 *   -> accel internal stage, gyro internal stage
 * Per output tick, at ODR:
 *   -> accel ODR stage, gyro ODR stage
 *   -> quantize and clip in the chip frame -> IIMU frame
 *
 * Each chip header turns its register config into a Path: the nominal cutoffs,
 * resolutions and frame. Params hold what the V1 fits give; the default Params
 * are an ideal chip with unfiltered paths.
 */

#ifndef CHIP_MODEL_HPP
#define CHIP_MODEL_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <tl/expected.hpp>

#include "chip/filter.hpp"
#include "imu/iimu.hpp"

namespace chip {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/* First-order Gauss-Markov drift: steady-state std and correlation time. */
struct Markov {
    Vec3 sigma = Vec3::Zero();
    Vec3 tau_s = Vec3::Ones();
};

/* Per axis, in the sensor's unit (g or dps). */
struct Noise {
    Vec3 white_density = Vec3::Zero(); /* unit/sqrt(Hz), one-sided, before the filters */
    Vec3 turn_on_sigma = Vec3::Zero(); /* zero level drawn per power-up */
    /* Bias instability as a sum of Gauss-Markov terms: log-spaced correlation times give the
     * flat Allan floor of flicker noise that one term cannot. */
    std::vector<Markov> markov{};
    Vec3 rate_random_walk = Vec3::Zero(); /* unit/sqrt(s) */
};

/* The filter slots are named after the LSM9DS1 stages; chip/lsm6dsl.hpp maps its own onto them. */
struct Params {
    /* Actual over nominal ODR; the digital filters' cutoffs scale with it. */
    double odr_scale = 1.0;
    /* Internal steps per output sample, for chips whose internal rate is not fixed. */
    int oversample = 8;

    Noise accel{};
    Noise gyro{};
    /* White noise density by full scale (g, dps) where it differs from Noise::white_density:
     * the LSM9DS1 gyro is quieter at +-245 dps than at +-2000. */
    std::map<double, Vec3> accel_density_by_fs{};
    std::map<double, Vec3> gyro_density_by_fs{};
    Vec3 accel_offset = Vec3::Zero(); /* fixed zero level, g */
    Vec3 gyro_offset  = Vec3::Zero(); /* dps */
    /* Drift level by full scale (g, dps): multiplies every Markov sigma and the random walk. */
    std::map<double, Vec3> accel_drift_scale_by_fs{};
    std::map<double, Vec3> gyro_drift_scale_by_fs{};
    /* Zero-rate level by full scale (dps) where it differs from gyro_offset. */
    std::map<double, Vec3> gyro_offset_by_fs{};
    Mat3 accel_gain         = Mat3::Identity(); /* sensitivity and cross-axis */
    Mat3 gyro_gain          = Mat3::Identity();
    /* Per axis, on negative rates, before gyro_gain: the LSM9DS1 reads one turning direction
     * about 3% lower than the other. */
    Vec3 gyro_negative_gain = Vec3::Ones();
    Mat3 gyro_g_sensitivity = Mat3::Zero();     /* dps per g of specific force */

    FilterShape accel_aa{};  /* accel internal stage */
    FilterShape accel_lp{};  /* accel ODR stage */
    FilterShape gyro_lpf1{}; /* gyro internal stage */
    FilterShape gyro_lpf2{}; /* gyro ODR stage */
    /* ODR-stage gyro shape by bandwidth register value where it differs from gyro_lpf2: the
     * strongest LSM9DS1 LPF2 setting measures as a moving average, the lighter ones do not. */
    std::map<int, FilterShape> gyro_lpf2_by_bw{};
    /* Internal-stage gyro shape per axis by nominal ODR where it differs from gyro_lpf1: the
     * measured knee sits off the datasheet by an ODR- and axis-dependent factor, applied on top
     * of odr_scale. */
    struct ScaledShape {
        FilterShape shape{};
        double cutoff_scale = 1.0;
    };
    std::map<int, std::array<ScaledShape, 3>> gyro_lpf1_by_odr{};

    double temperature_c = 25.0;
};

/* Cutoffs in use, Hz; 0 for a stage the config leaves out of the path. gyro_lpf1 is before
 * the per-axis factor of gyro_lpf1_by_odr; Model::gyro_lpf1_hz() is per axis. */
struct Cutoffs {
    double accel_aa;
    double accel_lp;
    double gyro_lpf1;
    double gyro_lpf2;
};

/* One chip config resolved against its datasheet. The register values go to the capture header. */
struct Path {
    const char *chip;
    double nominal_odr_hz;
    int oversample;
    Cutoffs cutoffs; /* before odr_scale and the fitted LPF1 factor */
    double accel_fs_g;
    double gyro_fs_dps;
    double accel_lsb_g;
    double gyro_lsb_dps;
    bool negate_z; /* the driver negates z to make the chip frame right-handed */
    int accel_bw;
    int gyro_bw;
    int gyro_out_path;
};

struct Output {
    double t_s;
    imu::IIMU::RawSample raw;
};

class Model {
  public:
    static tl::expected<Model, imu::IMUError> make(
        const Path &path, const Params &params, std::uint64_t seed
    ) {
        if (path.oversample < 1 || params.odr_scale <= 0.0) {
            return tl::make_unexpected(imu::IMUError::INVALID_CONFIG);
        }
        return Model(path, params, seed);
    }

    [[nodiscard]] double odr_hz() const { return this->odr_hz_; }
    [[nodiscard]] double internal_hz() const { return this->odr_hz_ * this->oversample_; }
    [[nodiscard]] const Cutoffs &cutoffs() const { return this->cutoffs_; }
    [[nodiscard]] const Vec3 &gyro_lpf1_hz() const { return this->gyro_lpf1_hz_; }
    [[nodiscard]] double temperature_c() const { return this->params_.temperature_c; }
    /* Time of output sample k, counted from 0 at power-up. */
    [[nodiscard]] double output_time_s(std::uint64_t k) const {
        return static_cast<double>(((k + 1) * this->oversample_) - 1) / this->internal_hz();
    }
    [[nodiscard]] double time_s() const {
        return static_cast<double>(this->step_) / this->internal_hz();
    }
    [[nodiscard]] imu::IIMU::RawScale raw_scale() const {
        return {static_cast<float>(this->accel_lsb_), static_cast<float>(this->gyro_lsb_)};
    }

    /* Runs internal steps up to t_end_s. truth(t) returns {specific force g, rate dps}
     * in the IIMU frame; sink(Output) gets every output sample. */
    template <typename Truth, typename Sink>
    void advance(double t_end_s, Truth &&truth, Sink &&sink) {
        const double dt = 1.0 / this->internal_hz();
        while (this->time_s() + (0.5 * dt) < t_end_s) {
            const double t    = this->time_s();
            const auto [f, w] = truth(t);
            if (!this->started_) { this->settle_(f, w); }

            Vec3 a = this->params_.accel_gain * f + this->params_.accel_offset
                   + this->accel_bias_.value(this->accel_noise_, dt, this->rng_)
                   + this->white_(this->accel_density_);
            Vec3 g = this->sensed_rate_(w) + this->params_.gyro_g_sensitivity * f
                   + this->gyro_offset_ + this->gyro_bias_.value(this->gyro_noise_, dt, this->rng_)
                   + this->white_(this->gyro_density_);
            for (int i = 0; i < 3; ++i) {
                a[i] = this->accel_aa_[i].step(a[i]);
                g[i] = this->gyro_lpf1_[i].step(g[i]);
            }

            ++this->step_;
            if (this->step_ % this->oversample_ == 0) {
                Output out{.t_s = t, .raw = {}};
                for (int i = 0; i < 3; ++i) {
                    const bool flip  = this->negate_z_ && i == 2;
                    out.raw.accel[i] = code_(this->accel_lp_[i].step(a[i]), this->accel_lsb_, flip);
                    out.raw.gyro[i]  = code_(this->gyro_lpf2_[i].step(g[i]), this->gyro_lsb_, flip);
                }
                sink(out);
            }
        }
    }

  private:
    struct Bias {
        Vec3 turn_on = Vec3::Zero();
        std::vector<Vec3> markov{};
        Vec3 walk = Vec3::Zero();

        /* Starts each Markov term in its steady state. */
        void start(const Noise &noise, std::mt19937_64 &rng) {
            std::normal_distribution<double> n(0.0, 1.0);
            for (int i = 0; i < 3; ++i) {
                this->turn_on[i] = noise.turn_on_sigma[i] * n(rng);
            }
            this->markov.assign(noise.markov.size(), Vec3::Zero());
            for (std::size_t k = 0; k < noise.markov.size(); ++k) {
                for (int i = 0; i < 3; ++i) {
                    this->markov[k][i] = noise.markov[k].sigma[i] * n(rng);
                }
            }
        }

        Vec3 value(const Noise &noise, double dt, std::mt19937_64 &rng) {
            std::normal_distribution<double> n(0.0, 1.0);
            Vec3 sum = this->turn_on;
            for (std::size_t k = 0; k < noise.markov.size(); ++k) {
                const Markov &term = noise.markov[k];
                for (int i = 0; i < 3; ++i) {
                    const double phi   = std::exp(-dt / term.tau_s[i]);
                    this->markov[k][i] = (phi * this->markov[k][i])
                                       + (term.sigma[i] * std::sqrt(1.0 - (phi * phi)) * n(rng));
                }
                sum += this->markov[k];
            }
            for (int i = 0; i < 3; ++i) {
                this->walk[i] += noise.rate_random_walk[i] * std::sqrt(dt) * n(rng);
            }
            return sum + this->walk;
        }
    };

    static Noise scaled(Noise noise, const std::map<double, Vec3> &by_fs, double fs) {
        const auto it = by_fs.find(fs);
        if (it == by_fs.end()) { return noise; }
        for (Markov &term : noise.markov) {
            term.sigma = term.sigma.cwiseProduct(it->second);
        }
        noise.rate_random_walk = noise.rate_random_walk.cwiseProduct(it->second);
        return noise;
    }

    Model(const Path &path, const Params &params, std::uint64_t seed)
        : params_(params), odr_hz_(path.nominal_odr_hz * params.odr_scale),
          oversample_(path.oversample), rng_(seed), accel_lsb_(path.accel_lsb_g),
          gyro_lsb_(path.gyro_lsb_dps), negate_z_(path.negate_z) {
        auto by_fs = [](const std::map<double, Vec3> &table, double fs, const Vec3 &fallback)
        {
            const auto it = table.find(fs);
            return it == table.end() ? fallback : it->second;
        };
        this->accel_density_ = by_fs(params.accel_density_by_fs, path.accel_fs_g, params.accel.white_density);
        this->gyro_density_  = by_fs(params.gyro_density_by_fs, path.gyro_fs_dps, params.gyro.white_density);
        this->gyro_offset_   = by_fs(params.gyro_offset_by_fs, path.gyro_fs_dps, params.gyro_offset);
        const double fs      = this->internal_hz();
        const double odr     = this->odr_hz_;
        const auto lpf1      = params.gyro_lpf1_by_odr.find(static_cast<int>(std::lround(path.nominal_odr_hz)));
        const auto lpf2  = params.gyro_lpf2_by_bw.find(path.gyro_bw);
        this->cutoffs_   = Cutoffs{
              .accel_aa  = path.cutoffs.accel_aa,
              .accel_lp  = path.cutoffs.accel_lp * params.odr_scale,
              .gyro_lpf1 = path.cutoffs.gyro_lpf1 * params.odr_scale,
              .gyro_lpf2 = path.cutoffs.gyro_lpf2 * params.odr_scale,
        };
        const Cutoffs &c = this->cutoffs_;
        for (int i = 0; i < 3; ++i) {
            this->accel_aa_[i]  = LowPass(params.accel_aa, c.accel_aa, fs);
            const Params::ScaledShape lpf1_fit = lpf1 == params.gyro_lpf1_by_odr.end()
                                                   ? Params::ScaledShape{params.gyro_lpf1, 1.0}
                                                   : lpf1->second[static_cast<std::size_t>(i)];
            this->gyro_lpf1_hz_[i] = c.gyro_lpf1 * lpf1_fit.cutoff_scale;
            this->gyro_lpf1_[i]    = LowPass(lpf1_fit.shape, this->gyro_lpf1_hz_[i], fs);
            this->accel_lp_[i]  = LowPass(params.accel_lp, c.accel_lp, odr);
            this->gyro_lpf2_[i] = LowPass(
                lpf2 == params.gyro_lpf2_by_bw.end() ? params.gyro_lpf2 : lpf2->second, c.gyro_lpf2, odr
            );
        }

        this->accel_noise_ = scaled(params.accel, params.accel_drift_scale_by_fs, path.accel_fs_g);
        this->gyro_noise_  = scaled(params.gyro, params.gyro_drift_scale_by_fs, path.gyro_fs_dps);
        this->accel_bias_.start(this->accel_noise_, this->rng_);
        this->gyro_bias_.start(this->gyro_noise_, this->rng_);
    }

    void settle_(const Vec3 &f, const Vec3 &w) {
        const Vec3 a = this->params_.accel_gain * f + this->params_.accel_offset
                     + this->accel_bias_.turn_on;
        const Vec3 g = this->sensed_rate_(w) + this->params_.gyro_g_sensitivity * f
                     + this->gyro_offset_ + this->gyro_bias_.turn_on;
        for (int i = 0; i < 3; ++i) {
            this->accel_aa_[i].reset(a[i]);
            this->accel_lp_[i].reset(a[i]);
            this->gyro_lpf1_[i].reset(g[i]);
            this->gyro_lpf2_[i].reset(g[i]);
        }
        this->started_ = true;
    }

    [[nodiscard]] Vec3 sensed_rate_(const Vec3 &w) const {
        const Vec3 &neg = this->params_.gyro_negative_gain;
        return this->params_.gyro_gain
             * Vec3(w[0] < 0 ? w[0] * neg[0] : w[0], w[1] < 0 ? w[1] * neg[1] : w[1], w[2] < 0 ? w[2] * neg[2] : w[2]);
    }

    Vec3 white_(const Vec3 &density) {
        /* One-sided density over the internal Nyquist band. */
        const double scale = std::sqrt(this->internal_hz() / 2.0);
        std::normal_distribution<double> n(0.0, 1.0);
        return {
            density[0] * scale * n(this->rng_),
            density[1] * scale * n(this->rng_),
            density[2] * scale * n(this->rng_)
        };
    }

    /* Clip in the chip frame, then negate z as the driver does when the chip frame is left-handed. */
    static std::int16_t code_(double value, double lsb, bool negate) {
        const double chip = negate ? -value : value;
        const double code = std::clamp(std::round(chip / lsb), -32768.0, 32767.0);
        const auto c      = static_cast<std::int16_t>(code);
        if (!negate) { return c; }
        return c == INT16_MIN ? INT16_MAX : static_cast<std::int16_t>(-c);
    }

    Params params_;
    double odr_hz_;
    int oversample_;
    std::mt19937_64 rng_;
    double accel_lsb_;
    double gyro_lsb_;
    bool negate_z_;
    Vec3 accel_density_ = Vec3::Zero();
    Vec3 gyro_density_  = Vec3::Zero();
    Vec3 gyro_offset_   = Vec3::Zero();
    Noise accel_noise_{};
    Noise gyro_noise_{};
    Cutoffs cutoffs_{};
    Vec3 gyro_lpf1_hz_ = Vec3::Zero();
    LowPass accel_aa_[3], gyro_lpf1_[3], accel_lp_[3], gyro_lpf2_[3];
    Bias accel_bias_{}, gyro_bias_{};
    std::uint64_t step_ = 0;
    bool started_       = false;
};

} /* namespace chip */

#endif /* CHIP_MODEL_HPP */
