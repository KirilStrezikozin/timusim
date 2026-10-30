/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Kinematic chain wrist (flexion, deviation) -> palm -> MCP (abduction,
 * flexion) -> PIP (flexion) -> DIP (flexion, coupled to PIP), and the ideal
 * readings of a palm and a distal finger sensor. SI units and radians throughout; only SensorTruth's
 * specific_force_g and rate_dps use the IIMU units.
 *
 * World frame: z up. Palm frame: x across the knuckles, y along the fingers,
 * z out of the back of the hand, so the identity palm lies face down. Its
 * origin is the wrist pivot; Pose::palm is the hand at zero wrist angles, which
 * rotate the hand about the flexion axis, then the abduction axis (deviation).
 */

#ifndef HAND_CHAIN_HPP
#define HAND_CHAIN_HPP

#include <Eigen/Geometry>

namespace hand {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

inline constexpr double STANDARD_GRAVITY = 9.80665;

struct Joints {
    double wrist_flex = 0.0;
    double wrist_dev  = 0.0;
    double mcp_abd    = 0.0;
    double mcp_flex = 0.0;
    double pip_flex = 0.0;
};

struct Pose {
    Quat palm  = Quat::Identity(); /* palm frame -> world */
    Vec3 wrist = Vec3::Zero();     /* palm frame origin in the world, m */
    Joints joints{};
    double support = 1.0; /* tremor drive scale: 1 a free hand, lower when braced or resting */
};

/* A pose and its first and second time derivatives. */
struct State {
    Pose pose{};
    Vec3 palm_rate_body  = Vec3::Zero(); /* rad/s, palm frame */
    Vec3 palm_accel_body = Vec3::Zero(); /* rad/s^2, palm frame */
    Vec3 wrist_vel       = Vec3::Zero();
    Vec3 wrist_acc       = Vec3::Zero();
    Joints joints_vel{};
    Joints joints_acc{};
};

/* Lengths in m. Segment frames match the palm frame with the finger straight. */
struct Geometry {
    Vec3 palm_sensor;   /* palm frame */
    Vec3 mcp;           /* MCP centre, palm frame */
    double proximal_m;  /* MCP -> PIP */
    double middle_m;    /* PIP -> DIP */
    Vec3 finger_sensor; /* from DIP, distal segment frame */
    double dip_per_pip; /* DIP flexion = dip_per_pip * PIP flexion */
    /* Roll of the distal segment about its length at 90 deg MCP flexion, growing with the
     * square of the flexion (the glove's fingers: about 5 deg). */
    double twist_at_90_rad = 0.0;

    Mat3 palm_mount   = Mat3::Identity(); /* sensor frame -> palm frame */
    Mat3 finger_mount = Mat3::Identity(); /* sensor frame -> distal segment frame */
    /* Positive flexion curls the finger toward the palm. */
    Vec3 flex_axis = -Vec3::UnitX();
    Vec3 abd_axis  = Vec3::UnitZ();
};

struct SensorTruth {
    Quat orientation; /* sensor frame -> world */
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;     /* world, m/s^2 */
    Vec3 rate_world;       /* rad/s */
    Vec3 specific_force_g; /* sensor frame, +1 g on the up axis at rest */
    Vec3 rate_dps;         /* sensor frame */
};

struct HandTruth {
    SensorTruth palm;
    SensorTruth finger;
};

namespace detail {

/* A rigid body's frame and motion, all in world coordinates. */
struct Link {
    Mat3 r;
    Vec3 rate, rate_dot;
    Vec3 origin, vel, acc;
};

inline Link translate(const Link &l, const Vec3 &offset_local) {
    const Vec3 d = l.r * offset_local;
    Link out     = l;
    out.origin   = l.origin + d;
    out.vel      = l.vel + l.rate.cross(d);
    out.acc      = l.acc + l.rate_dot.cross(d) + l.rate.cross(l.rate.cross(d));
    return out;
}

inline Link rotate(const Link &l, const Vec3 &axis_local, double q, double qd, double qdd) {
    const Vec3 axis = l.r * axis_local;
    Link out        = l;
    out.r           = l.r * Eigen::AngleAxisd(q, axis_local).toRotationMatrix();
    out.rate        = l.rate + axis * qd;
    out.rate_dot    = l.rate_dot + axis * qdd + l.rate.cross(axis * qd);
    return out;
}

inline SensorTruth sense(const Link &l, const Vec3 &position_local, const Mat3 &mount) {
    const Link p = translate(l, position_local);
    const Mat3 r = p.r * mount;
    return SensorTruth{
        .orientation      = Quat(r),
        .position         = p.origin,
        .velocity         = p.vel,
        .acceleration     = p.acc,
        .rate_world       = p.rate,
        .specific_force_g = r.transpose() * (p.acc + STANDARD_GRAVITY * Vec3::UnitZ()) / STANDARD_GRAVITY,
        .rate_dps         = r.transpose() * p.rate * (180.0 / M_PI),
    };
}

} /* namespace detail */

inline HandTruth evaluate(const Geometry &g, const State &s) {
    using detail::Link;
    const Mat3 rp = s.pose.palm.toRotationMatrix();
    const Link palm{
        .r        = rp,
        .rate     = rp * s.palm_rate_body,
        .rate_dot = rp * s.palm_accel_body,
        .origin   = s.pose.wrist,
        .vel      = s.wrist_vel,
        .acc      = s.wrist_acc,
    };

    const Joints &q = s.pose.joints, &qd = s.joints_vel, &qdd = s.joints_acc;
    Link hand = detail::rotate(palm, g.flex_axis, q.wrist_flex, qd.wrist_flex, qdd.wrist_flex);
    hand      = detail::rotate(hand, g.abd_axis, q.wrist_dev, qd.wrist_dev, qdd.wrist_dev);
    Link l    = detail::translate(hand, g.mcp);
    l      = detail::rotate(l, g.abd_axis, q.mcp_abd, qd.mcp_abd, qdd.mcp_abd);
    l      = detail::rotate(l, g.flex_axis, q.mcp_flex, qd.mcp_flex, qdd.mcp_flex);
    l      = detail::translate(l, Vec3(0.0, g.proximal_m, 0.0));
    l      = detail::rotate(l, g.flex_axis, q.pip_flex, qd.pip_flex, qdd.pip_flex);
    l      = detail::translate(l, Vec3(0.0, g.middle_m, 0.0));
    l      = detail::rotate(
        l, g.flex_axis, g.dip_per_pip * q.pip_flex, g.dip_per_pip * qd.pip_flex, g.dip_per_pip * qdd.pip_flex
    );
    if (g.twist_at_90_rad != 0.0) {
        const double k = g.twist_at_90_rad / (M_PI / 2 * M_PI / 2);
        const double m = q.mcp_flex, md = qd.mcp_flex, mdd = qdd.mcp_flex;
        l = detail::rotate(l, Vec3::UnitY(), k * m * m, 2 * k * m * md, 2 * k * ((md * md) + (m * mdd)));
    }

    return HandTruth{
        .palm   = detail::sense(hand, g.palm_sensor, g.palm_mount),
        .finger = detail::sense(l, g.finger_sensor, g.finger_mount),
    };
}

} /* namespace hand */

#endif /* HAND_CHAIN_HPP */
