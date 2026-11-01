/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Mount jolts: the chip turning on its strap or skin, not through the joint chain, while its
 * segment moves. Three independent axes of white noise through the second-order band-pass of
 * SharedDrive give the rate, peak density level^2 per (rad/s)^2 of the gate (the segment's
 * scripted speed); its integral is the small angle that also turns what the chip senses.
 */

#ifndef HAND_JOLT_HPP
#define HAND_JOLT_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <random>

#include "hand/chain.hpp"

namespace hand {

struct JoltParams {
    double frequency_hz = 0.0;
    double bandwidth_hz = 0.0;
    double level_dps    = 0.0; /* dps / sqrt(Hz) at the peak, per rad/s of segment speed; 0 is off */

    [[nodiscard]] bool active() const { return this->level_dps > 0.0 && this->frequency_hz > 0.0 && this->bandwidth_hz > 0.0; }
};

class Jolt {
  public:
    static constexpr double STEP_S   = 0.0005;
    static constexpr double WINDOW_S = 1.0;

    using Gate = std::function<double(double)>;

    Jolt() = default;
    Jolt(const JoltParams &p, std::uint64_t seed, Gate gate) : p_(p), rng_(seed), gate_(std::move(gate)) {
        this->history_.push_back(Node{});
    }

    [[nodiscard]] bool active() const { return this->p_.active(); }

    struct Sample {
        Vec3 rate_dps  = Vec3::Zero(); /* sensor frame */
        Vec3 angle_rad = Vec3::Zero(); /* small rotation of the chip, sensor frame */
    };

    /* t must not go back more than WINDOW_S. */
    Sample at(double t) {
        if (!this->active() || t <= 0.0) { return {}; }
        while (this->history_.back().t < t) { this->step_(); }
        while (this->history_.size() > 2 && this->history_[1].t < t - WINDOW_S) { this->history_.pop_front(); }
        const auto k = static_cast<std::size_t>(std::floor((t - this->history_.front().t) / STEP_S));
        const Node &a = this->history_[std::min(k, this->history_.size() - 2)];
        const Node &b = this->history_[std::min(k, this->history_.size() - 2) + 1];
        const double s = std::clamp((t - a.t) / (b.t - a.t), 0.0, 1.0);
        Sample out;
        for (int i = 0; i < 3; ++i) {
            out.rate_dps[i]  = a.out[i] + ((b.out[i] - a.out[i]) * s);
            out.angle_rad[i] = (a.angle[i] + ((b.angle[i] - a.angle[i]) * s)) * (M_PI / 180.0);
        }
        return out;
    }

  private:
    struct Node {
        double t = 0.0;
        std::array<double, 3> y{}, v{}, out{}, angle{};
    };

    void step_() {
        const Node &prev = this->history_.back();
        Node next        = prev;
        next.t           = prev.t + STEP_S;
        const double w    = 2.0 * M_PI * this->p_.frequency_hz;
        const double zeta = this->p_.bandwidth_hz / (2.0 * this->p_.frequency_hz);
        const double gate = this->gate_ ? std::abs(this->gate_(next.t)) : 1.0;
        std::normal_distribution<double> n(0.0, 1.0);
        for (int i = 0; i < 3; ++i) {
            const double in = 2.0 * zeta * w * this->p_.level_dps * n(this->rng_) / std::sqrt(2.0 * STEP_S);
            next.v[i] += (in - (2.0 * zeta * w * next.v[i]) - (w * w * next.y[i])) * STEP_S;
            next.y[i] += next.v[i] * STEP_S;
            next.out[i] = gate * next.v[i];
            next.angle[i] += 0.5 * (prev.out[i] + next.out[i]) * STEP_S;
            next.angle[i] *= 1.0 - (STEP_S * 2.0 * M_PI * 0.5); /* forget below 0.5 Hz: the strap returns */
        }
        this->history_.push_back(next);
    }

    JoltParams p_{};
    std::mt19937_64 rng_;
    Gate gate_;
    std::deque<Node> history_;
};

} /* namespace hand */

#endif /* HAND_JOLT_HPP */
