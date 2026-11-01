/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * The scripted trajectory with joint tremor and postural sway on top. Sway
 * rotates the whole hand about the wrist (x, y, z rotations in turn) and moves
 * the wrist.
 */

#ifndef HAND_MOTION_HPP
#define HAND_MOTION_HPP

#include <cmath>
#include <memory>

#include "hand/chain.hpp"
#include "hand/trajectory.hpp"
#include "hand/tremor.hpp"

namespace hand {

/* The scripted joint rates at time t, rad/s, by Dof. Forearm rotation (SWAY_RY) is never
 * scripted; it takes the wrist's speed, so that its roughness follows wrist moves. The other
 * sway DOF are 0. */
inline std::array<double, DOF_COUNT> scripted_speed(const Trajectory &tr, double t) {
    const Joints v = tr.at(t).joints_vel;
    std::array<double, DOF_COUNT> out{};
    out[WRIST_FLEX] = v.wrist_flex;
    out[WRIST_DEV]  = v.wrist_dev;
    out[MCP_ABD]    = v.mcp_abd;
    out[MCP_FLEX]   = v.mcp_flex;
    out[PIP_FLEX]   = v.pip_flex;
    out[SWAY_RY]    = std::hypot(v.wrist_flex, v.wrist_dev);
    return out;
}

class Motion {
  public:
    explicit Motion(
        Trajectory trajectory, const Tremor::Oscillators &oscillators = {}, std::uint64_t seed = 1,
        std::vector<SharedDrive> drives = {}
    )
        : trajectory_(std::make_shared<Trajectory>(std::move(trajectory))) {
        std::shared_ptr<const Trajectory> tr = this->trajectory_;
        this->tremor_ = Tremor(
            oscillators, seed, [tr](double t) { return tr->at(t).pose.support; }, std::move(drives),
            [tr](double t) { return scripted_speed(*tr, t); }
        );
    }

    [[nodiscard]] double duration() const { return this->trajectory_->duration(); }
    [[nodiscard]] const Trajectory &trajectory() const { return *this->trajectory_; }

    State at(double t) {
        State s = this->trajectory_->at(t);
        /* On the table from the start support up to 1: full contact at the start, none once lifted. */
        const double s0 = this->trajectory_->at(0.0).pose.support;
        s.contact       = s0 < 1.0 ? std::clamp((1.0 - s.pose.support) / (1.0 - s0), 0.0, 1.0) : 0.0;
        if (!this->tremor_.active()) { return s; }
        const auto d = this->tremor_.at(t);
        auto add     = [&](double Joints::*field, Dof dof) {
            s.pose.joints.*field += d[dof].q;
            s.joints_vel.*field += d[dof].qd;
            s.joints_acc.*field += d[dof].qdd;
        };
        add(&Joints::wrist_flex, WRIST_FLEX);
        add(&Joints::wrist_dev, WRIST_DEV);
        add(&Joints::mcp_abd, MCP_ABD);
        add(&Joints::mcp_flex, MCP_FLEX);
        add(&Joints::pip_flex, PIP_FLEX);

        detail::Link sway{
            .r = Mat3::Identity(), .rate = Vec3::Zero(), .rate_dot = Vec3::Zero(),
            .origin = Vec3::Zero(), .vel = Vec3::Zero(), .acc = Vec3::Zero(),
        };
        for (const auto &[axis, dof] : {std::pair{Vec3::UnitX(), SWAY_RX}, std::pair{Vec3::UnitY(), SWAY_RY},
                                        std::pair{Vec3::UnitZ(), SWAY_RZ}}) {
            sway = detail::rotate(sway, axis, d[dof].q, d[dof].qd, d[dof].qdd);
        }
        if (sway.rate.norm() > 0.0 || !sway.r.isIdentity()) {
            /* palm' = S palm: w' = w_s + S w, a' = a_s + S a + w_s x (S w), all in the world. */
            const Mat3 palm    = s.pose.palm.toRotationMatrix();
            const Vec3 w       = sway.r * (palm * s.palm_rate_body);
            const Vec3 a       = sway.r * (palm * s.palm_accel_body);
            const Mat3 swayed  = sway.r * palm;
            s.pose.palm        = Quat(swayed);
            s.palm_rate_body   = swayed.transpose() * (sway.rate + w);
            s.palm_accel_body  = swayed.transpose() * (sway.rate_dot + a + sway.rate.cross(w));
        }
        s.pose.wrist += Vec3(d[SWAY_X].q, d[SWAY_Y].q, d[SWAY_Z].q);
        s.wrist_vel += Vec3(d[SWAY_X].qd, d[SWAY_Y].qd, d[SWAY_Z].qd);
        s.wrist_acc += Vec3(d[SWAY_X].qdd, d[SWAY_Y].qdd, d[SWAY_Z].qdd);
        return s;
    }

  private:
    std::shared_ptr<const Trajectory> trajectory_;
    Tremor tremor_;
};

} /* namespace hand */

#endif /* HAND_MOTION_HPP */
