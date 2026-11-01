/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * A pose sequence: holds and A -> B moves, evaluated at any time with analytic
 * derivatives. The palm turns about the fixed axis of the relative rotation;
 * the wrist position and the joint angles move linearly in the progress u.
 */

#ifndef HAND_TRAJECTORY_HPP
#define HAND_TRAJECTORY_HPP

#include <vector>

#include "hand/chain.hpp"
#include "hand/profile.hpp"

namespace hand {

class Trajectory {
  public:
    explicit Trajectory(const Pose &start) : start_(start), end_pose_(start) {}

    Trajectory &move(
        double duration_s, const Pose &to, Profile profile = Profile::MIN_JERK, Shape shape = {}
    ) {
        Pose from = this->end_pose_;
        /* Same rotation, but the short way round. */
        if (from.palm.dot(to.palm) < 0.0) { from.palm.coeffs() = -from.palm.coeffs(); }
        const Eigen::AngleAxisd turn(from.palm.conjugate() * to.palm);
        this->segments_.push_back(Segment{
            .t0        = this->duration_,
            .duration  = duration_s,
            .from      = from,
            .to        = to,
            .turn_axis = turn.axis(),
            .turn_rad  = turn.angle(),
            .profile   = profile,
            .shape     = shape,
        });
        this->duration_ += duration_s;
        this->end_pose_  = to;
        return *this;
    }

    Trajectory &hold(double duration_s) { return this->move(duration_s, this->end_pose_); }

    [[nodiscard]] double duration() const { return this->duration_; }

    [[nodiscard]] State at(double t) const {
        const Segment *seg = nullptr;
        for (const Segment &s : this->segments_) {
            if (t < s.t0 + s.duration) {
                seg = &s;
                break;
            }
        }
        if (seg == nullptr) { return State{.pose = this->end_pose_}; }
        if (t < seg->t0) { return State{.pose = this->start_}; }

        const Progress p = progress(seg->profile, t - seg->t0, seg->duration, seg->shape);
        const Pose &a = seg->from, &b = seg->to;
        const Vec3 turn = seg->turn_axis * seg->turn_rad;

        auto joints = [&](const Joints &base, double k)
        {
            return Joints{
                .wrist_flex = base.wrist_flex + ((b.joints.wrist_flex - a.joints.wrist_flex) * k),
                .wrist_dev  = base.wrist_dev + ((b.joints.wrist_dev - a.joints.wrist_dev) * k),
                .mcp_abd    = base.mcp_abd + ((b.joints.mcp_abd - a.joints.mcp_abd) * k),
                .mcp_flex   = base.mcp_flex + ((b.joints.mcp_flex - a.joints.mcp_flex) * k),
                .pip_flex   = base.pip_flex + ((b.joints.pip_flex - a.joints.pip_flex) * k),
            };
        };

        return State{
            .pose =
                Pose{
                    .palm   = a.palm * Quat(Eigen::AngleAxisd(seg->turn_rad * p.u, seg->turn_axis)),
                    .wrist  = a.wrist + (b.wrist - a.wrist) * p.u,
                    .joints = joints(a.joints, p.u),
                    .support = a.support + ((b.support - a.support) * p.u),
                },
            .palm_rate_body  = turn * p.du,
            .palm_accel_body = turn * p.ddu,
            .wrist_vel       = (b.wrist - a.wrist) * p.du,
            .wrist_acc       = (b.wrist - a.wrist) * p.ddu,
            .joints_vel      = joints(Joints{}, p.du),
            .joints_acc      = joints(Joints{}, p.ddu),
        };
    }

  private:
    struct Segment {
        double t0;
        double duration;
        Pose from;
        Pose to;
        Vec3 turn_axis;
        double turn_rad;
        Profile profile;
        Shape shape;
    };

    Pose start_;
    Pose end_pose_;
    double duration_ = 0.0;
    std::vector<Segment> segments_;
};

} /* namespace hand */

#endif /* HAND_TRAJECTORY_HPP */
