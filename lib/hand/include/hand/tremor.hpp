/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Joint-level tremor: each degree of freedom is a torsional spring-mass-damper
 * I q'' + B q' + K q = tau(t) driven by white noise through a first-order
 * muscle force filter (Lakie et al. 2012: 120 ms),
 * so tremor comes out as rotation about real joints with its resonance at
 * sqrt(K / I) (Elble et al.), and added mass on a segment raises I and lowers
 * the peak. Displacement noise on poses, which Leite et al. found gives
 * unrealistic motion, never enters.
 *
 * The oscillators run at a fixed step with the noise held per step; any time in
 * the last WINDOW_S is served by cubic Hermite interpolation of q (with q') and
 * of q' (with q''), so callers may interleave slightly out of order.
 */

#ifndef HAND_TREMOR_HPP
#define HAND_TREMOR_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <random>
#include <vector>

namespace hand {

struct Oscillator {
    double inertia        = 0.0; /* kg m^2, segment plus added mass */
    double stiffness      = 0.0; /* N m / rad */
    double damping        = 0.0; /* N m s / rad */
    double torque_density = 0.0; /* N m / sqrt(Hz), one-sided, of the drive before the muscle filter; 0 is off */
    double muscle_tau_s   = 0.0; /* muscle force low-pass time constant; 0 passes white torque */
    int muscle_order      = 1;   /* identical first-order stages: 1 as Lakie et al., 2 a critically damped twitch */

    [[nodiscard]] bool active() const { return this->torque_density > 0.0 && this->inertia > 0.0; }
};

/* Joint tremor in Joints order, then postural sway: forearm rotation about world x, y, z
 * (rad) and wrist translation along world x, y, z (m; the "oscillator" then has units of
 * mass and N/m, and its drive is a force). */
enum Dof : std::uint8_t {
    WRIST_FLEX,
    WRIST_DEV,
    MCP_ABD,
    MCP_FLEX,
    PIP_FLEX,
    SWAY_RX,
    SWAY_RY,
    SWAY_RZ,
    SWAY_X,
    SWAY_Y,
    SWAY_Z,
    DOF_COUNT,
};

/* A narrow-band torque shared by several DOF (a central oscillation, the heartbeat, breathing,
 * movement roughness): white noise through a second-order band-pass per harmonic, centred at
 * k * frequency_hz with a half-power width of k * bandwidth_hz and a peak of harmonics[k-1]^2
 * times gain^2 (gain in N m / sqrt(Hz), one-sided, per DOF). It joins each DOF's drive before
 * the muscle filter, times the gate: Pose::support like the joint noise, 1, or the DOF's own
 * scripted speed in rad/s (gain then per rad/s). */
struct SharedDrive {
    enum class Gate : std::uint8_t { Support, None, Speed };
    double frequency_hz = 0.0;
    double bandwidth_hz = 0.0;
    std::vector<double> harmonics{1.0};
    std::array<double, DOF_COUNT> gain{};
    Gate gate = Gate::Support;
};

struct JointMotion {
    double q   = 0.0;
    double qd  = 0.0;
    double qdd = 0.0;
};

class Tremor {
  public:
    static constexpr double STEP_S   = 0.0005;
    static constexpr double WINDOW_S = 2.0;

    using Oscillators = std::array<Oscillator, DOF_COUNT>;
    /* Scales the torque noise at time t; Pose::support along the trajectory. */
    using Support = std::function<double(double)>;
    /* Each DOF's scripted speed at time t, rad/s, for speed-gated drives. */
    using Speed = std::function<std::array<double, DOF_COUNT>(double)>;

    Tremor() = default;

    Tremor(
        const Oscillators &oscillators, std::uint64_t seed, Support support, std::vector<SharedDrive> drives = {},
        Speed speed = {}
    )
        : osc_(oscillators), drives_(std::move(drives)), rng_(seed), support_(std::move(support)), speed_(std::move(speed)) {
        for (const SharedDrive &d : this->drives_) { this->band_.resize(this->band_.size() + d.harmonics.size()); }
        this->history_.push_back(Node{});
    }

    [[nodiscard]] bool active() const {
        for (int j = 0; j < DOF_COUNT; ++j) {
            if (this->dof_active_(j)) { return true; }
        }
        return false;
    }

    std::array<JointMotion, DOF_COUNT> at(double t) {
        std::array<JointMotion, DOF_COUNT> out{};
        if (this->history_.empty() || t <= 0.0) { return out; }
        while (this->history_.back().t < t) { this->step_(); }
        while (this->history_.size() > 2 && this->history_[1].t < this->history_.back().t - WINDOW_S
               && this->history_[1].t < t) {
            this->history_.pop_front();
        }
        const auto k = static_cast<std::size_t>(std::floor((t - this->history_.front().t) / STEP_S));
        const Node &a = this->history_[std::min(k, this->history_.size() - 2)];
        const Node &b = this->history_[std::min(k, this->history_.size() - 2) + 1];
        const double h = b.t - a.t;
        const double s = std::clamp((t - a.t) / h, 0.0, 1.0);
        for (int j = 0; j < DOF_COUNT; ++j) {
            out[j].q   = hermite_(a.x[j].q, a.x[j].qd, b.x[j].q, b.x[j].qd, s, h);
            out[j].qd  = hermite_(a.x[j].qd, a.x[j].qdd, b.x[j].qd, b.x[j].qdd, s, h);
            out[j].qdd = a.x[j].qdd + (b.x[j].qdd - a.x[j].qdd) * s;
        }
        return out;
    }

  private:
    static constexpr int MAX_MUSCLE_ORDER = 4;

    struct Node {
        double t = 0.0;
        std::array<JointMotion, DOF_COUNT> x{};
        std::array<std::array<double, MAX_MUSCLE_ORDER>, DOF_COUNT> torque{};
    };

    struct Band {
        double y = 0.0;
        double v = 0.0;
    };

    [[nodiscard]] bool dof_active_(int j) const {
        const Oscillator &o = this->osc_[j];
        if (o.inertia <= 0.0) { return false; }
        if (o.torque_density > 0.0) { return true; }
        return std::any_of(this->drives_.begin(), this->drives_.end(), [j](const SharedDrive &d) {
            return d.gain[j] != 0.0 && d.frequency_hz > 0.0;
        });
    }

    /* Advances every band one step; returns each drive's band-passed signal, unit peak density
     * per harmonic amplitude. Semi-implicit Euler on y'' + 2 z w y' + w^2 y = 2 z w a n, whose
     * rate y' has a peak gain a at w and a half-power width 2 z w. */
    std::vector<double> step_drives_(std::normal_distribution<double> &n) {
        std::vector<double> out(this->drives_.size(), 0.0);
        std::size_t b = 0;
        for (std::size_t i = 0; i < this->drives_.size(); ++i) {
            const SharedDrive &d = this->drives_[i];
            for (std::size_t k = 0; k < d.harmonics.size(); ++k, ++b) {
                const double h    = static_cast<double>(k + 1);
                const double w    = 2.0 * M_PI * h * d.frequency_hz;
                const double zeta = d.bandwidth_hz / (2.0 * d.frequency_hz);
                Band &s           = this->band_[b];
                const double in   = 2.0 * zeta * w * d.harmonics[k] * n(this->rng_) / std::sqrt(2.0 * STEP_S);
                s.v += (in - (2.0 * zeta * w * s.v) - (w * w * s.y)) * STEP_S;
                s.y += s.v * STEP_S;
                out[i] += s.v;
            }
        }
        return out;
    }

    static double hermite_(double p0, double m0, double p1, double m1, double s, double h) {
        const double s2 = s * s;
        const double s3 = s2 * s;
        return ((2 * s3 - 3 * s2 + 1) * p0) + ((s3 - 2 * s2 + s) * h * m0) + ((-2 * s3 + 3 * s2) * p1)
             + ((s3 - s2) * h * m1);
    }

    /* Stable for the stiffest joints here (f0 < 100 Hz at STEP_S). The held noise torque has
     * density sigma when its per-step std is sigma / sqrt(2 STEP_S). */
    void step_() {
        const Node &prev = this->history_.back();
        Node next;
        next.t                 = prev.t + STEP_S;
        const double support   = this->support_ ? this->support_(next.t) : 1.0;
        std::normal_distribution<double> n(0.0, 1.0);
        const std::vector<double> shared = this->step_drives_(n);
        const bool gated = this->speed_ && std::any_of(this->drives_.begin(), this->drives_.end(), [](const SharedDrive &d) {
            return d.gate == SharedDrive::Gate::Speed;
        });
        const std::array<double, DOF_COUNT> speed = gated ? this->speed_(next.t) : std::array<double, DOF_COUNT>{};
        for (int j = 0; j < DOF_COUNT; ++j) {
            const Oscillator &o = this->osc_[j];
            if (!this->dof_active_(j)) { continue; }
            double drive = support * o.torque_density / std::sqrt(2.0 * STEP_S) * n(this->rng_);
            for (std::size_t i = 0; i < this->drives_.size(); ++i) {
                const SharedDrive &d = this->drives_[i];
                const double gate    = d.gate == SharedDrive::Gate::Support ? support
                                     : d.gate == SharedDrive::Gate::Speed   ? std::abs(speed[j])
                                                                            : 1.0;
                drive += gate * d.gain[j] * shared[i];
            }
            const double keep  = o.muscle_tau_s > 0.0 ? std::exp(-STEP_S / o.muscle_tau_s) : 0.0;
            const int stages   = std::clamp(o.muscle_order, 1, MAX_MUSCLE_ORDER);
            double in          = drive;
            for (int k = 0; k < stages; ++k) {
                next.torque[j][k] = (keep * prev.torque[j][k]) + ((1.0 - keep) * in);
                in                = next.torque[j][k];
            }
            /* Velocity Verlet, damping on the predicted rate. Each node keeps the acceleration
             * at its own state, the slope the Hermite interpolation needs there. */
            const JointMotion &x  = prev.x[j];
            const double q        = x.q + (x.qd * STEP_S) + (0.5 * x.qdd * STEP_S * STEP_S);
            const double qd_guess = x.qd + (x.qdd * STEP_S);
            const double qdd      = (in - (o.damping * qd_guess) - (o.stiffness * q)) / o.inertia;
            next.x[j]             = {q, x.qd + (0.5 * (x.qdd + qdd) * STEP_S), qdd};
        }
        this->history_.push_back(next);
    }

    Oscillators osc_{};
    std::vector<SharedDrive> drives_;
    std::vector<Band> band_;
    std::mt19937_64 rng_;
    Support support_;
    Speed speed_;
    std::deque<Node> history_;
};

} /* namespace hand */

#endif /* HAND_TREMOR_HPP */
