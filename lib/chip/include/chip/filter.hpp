/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Low-pass filters:
 * - BUTTERWORTH: the analog prototype through the bilinear transform with the
 *   cutoff prewarped, so the response has a zero at Nyquist;
 * - ONE_POLE: n identical sections y += a (x - y), whose response stays
 *   finite at Nyquist, as measured on the LSM9DS1 accelerometer path;
 * both put the cascade's -3 dB point at the cutoff;
 * - MOVING_AVERAGE: the mean of the last n samples, zeros at multiples of fs / n
 *   and -13 dB sidelobes, as measured on the LSM9DS1 gyro LPF2; the cutoff is
 *   ignored, n sets it.
 */

#ifndef CHIP_FILTER_HPP
#define CHIP_FILTER_HPP

#include <cmath>
#include <complex>
#include <cstdint>
#include <deque>
#include <vector>

namespace chip {

enum class FilterKind : std::uint8_t {
    BUTTERWORTH,
    ONE_POLE,
    MOVING_AVERAGE,
};

struct FilterShape {
    FilterKind kind = FilterKind::BUTTERWORTH;
    int order       = 0; /* 0 bypasses the stage */
};

class LowPass {
  public:
    LowPass() = default;

    LowPass(FilterShape shape, double cutoff_hz, double fs_hz) {
        if (shape.order <= 0) { return; }
        if (shape.kind == FilterKind::MOVING_AVERAGE) {
            this->taps_ = static_cast<std::size_t>(shape.order);
            return;
        }
        if (cutoff_hz <= 0.0) { return; }
        if (shape.kind == FilterKind::ONE_POLE) {
            this->one_pole_(shape.order, cutoff_hz, fs_hz);
        } else {
            this->butterworth_(shape.order, cutoff_hz, fs_hz);
        }
    }

    /* Settles every section on a constant input, so a filter started on 1 g has no step transient. */
    void reset(double x) {
        this->window_.assign(this->taps_, x);
        this->sum_ = x * static_cast<double>(this->taps_);
        for (Section &s : this->sections_) {
            s.z2 = (s.b[2] - s.a2) * x;
            s.z1 = ((s.b[1] - s.a1) * x) + s.z2;
        }
    }

    double step(double x) {
        if (this->taps_ > 0) {
            if (this->window_.size() == this->taps_) {
                this->sum_ -= this->window_.front();
                this->window_.pop_front();
            }
            this->window_.push_back(x);
            this->sum_ += x;
            return this->sum_ / static_cast<double>(this->window_.size());
        }
        for (Section &s : this->sections_) {
            const double y = (s.b[0] * x) + s.z1;
            s.z1           = (s.b[1] * x) - (s.a1 * y) + s.z2;
            s.z2           = (s.b[2] * x) - (s.a2 * y);
            x              = y;
        }
        return x;
    }

    [[nodiscard]] std::complex<double> response(double f_hz, double fs_hz) const {
        const std::complex<double> z1 = std::polar(1.0, -2.0 * M_PI * f_hz / fs_hz);
        std::complex<double> h        = 1.0;
        if (this->taps_ > 0) {
            std::complex<double> sum = 0.0;
            for (std::size_t k = 0; k < this->taps_; ++k) { sum += std::pow(z1, static_cast<double>(k)); }
            return sum / static_cast<double>(this->taps_);
        }
        for (const Section &s : this->sections_) {
            h *= (s.b[0] + (s.b[1] * z1) + (s.b[2] * z1 * z1)) / (1.0 + (s.a1 * z1) + (s.a2 * z1 * z1));
        }
        return h;
    }

  private:
    struct Section {
        double b[3] = {};
        double a1   = 0.0;
        double a2   = 0.0;
        double z1   = 0.0;
        double z2   = 0.0;
    };

    void butterworth_(int order, double cutoff_hz, double fs_hz) {
        const double k  = 2.0 * fs_hz;
        const double wc = k * std::tan(M_PI * cutoff_hz / fs_hz);
        for (int i = 0; i < order / 2; ++i) {
            /* Real part of a pole of the unit-cutoff prototype. */
            const double re = -std::sin(M_PI * (2 * i + 1) / (2.0 * order));
            const double a0 = (k * k) - (2.0 * re * wc * k) + (wc * wc);
            this->sections_.push_back(Section{
                .b  = {wc * wc / a0, 2.0 * wc * wc / a0, wc * wc / a0},
                .a1 = ((2.0 * wc * wc) - (2.0 * k * k)) / a0,
                .a2 = ((k * k) + (2.0 * re * wc * k) + (wc * wc)) / a0,
            });
        }
        if (order % 2 == 1) {
            const double a0 = k + wc;
            this->sections_.push_back(Section{.b = {wc / a0, wc / a0, 0.0}, .a1 = (wc - k) / a0, .a2 = 0.0});
        }
    }

    /* |H|^2 of one section at the cutoff is g = 0.5^(1/n); with m = 1 - cos(2 pi fc / fs),
     * a^2 (1 - g) + 2 g m a - 2 g m = 0. */
    void one_pole_(int order, double cutoff_hz, double fs_hz) {
        const double g = std::pow(0.5, 1.0 / order);
        const double m = 1.0 - std::cos(2.0 * M_PI * cutoff_hz / fs_hz);
        const double a = ((-2.0 * g * m) + std::sqrt((4.0 * g * g * m * m) + (8.0 * (1.0 - g) * g * m))) / (2.0 * (1.0 - g));
        for (int i = 0; i < order; ++i) {
            this->sections_.push_back(Section{.b = {a, 0.0, 0.0}, .a1 = a - 1.0, .a2 = 0.0});
        }
    }

    std::vector<Section> sections_;
    std::size_t taps_ = 0;
    std::deque<double> window_;
    double sum_ = 0.0;
};

} /* namespace chip */

#endif /* CHIP_FILTER_HPP */
