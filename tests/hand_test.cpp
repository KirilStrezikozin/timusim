/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cmath>
#include <cstdio>

#include "hand/chain.hpp"
#include "hand/trajectory.hpp"

namespace {

using hand::Quat;
using hand::Vec3;

int g_failures = 0;

void check(bool ok, const char *what, double value) {
    std::printf("[%s] %s (%.3g)\n", ok ? "PASS" : "FAIL", what, value);
    if (!ok) { ++g_failures; }
}

double rad(double deg) { return deg * M_PI / 180.0; }

/* Test fixture only; the model's geometry comes from measurements. */
hand::Geometry test_geometry() {
    hand::Geometry g{
        .palm_sensor   = Vec3(0.01, 0.05, 0.02),
        .mcp           = Vec3(0.0, 0.09, 0.0),
        .proximal_m    = 0.045,
        .middle_m      = 0.025,
        .finger_sensor = Vec3(0.0, 0.01, 0.008),
        .dip_per_pip   = 0.7,
    };
    g.finger_mount    = Eigen::AngleAxisd(rad(10.0), Vec3::UnitZ()).toRotationMatrix();
    g.twist_at_90_rad = rad(5.0);
    return g;
}

hand::Pose pose(const Quat &palm, Vec3 wrist, double abd, double mcp, double pip) {
    return hand::Pose{
        .palm   = palm,
        .wrist  = wrist,
        .joints = {.mcp_abd = rad(abd), .mcp_flex = rad(mcp), .pip_flex = rad(pip)},
    };
}

hand::Trajectory test_trajectory() {
    const Quat tilted(Eigen::AngleAxisd(rad(70.0), Vec3(0.3, 1.0, 0.2).normalized()));
    hand::Trajectory tr(pose(Quat::Identity(), Vec3::Zero(), 0, 0, 0));
    tr.hold(0.5)
        .move(0.8, pose(tilted, Vec3(0.05, -0.02, 0.1), 10, 60, 40))
        .move(0.6, pose(tilted, Vec3(0.05, -0.02, 0.1), -5, 20, 80), hand::Profile::COSINE)
        .move(1.0, pose(Quat::Identity(), Vec3::Zero(), 0, 0, 0))
        .hold(0.5);
    return tr;
}

void profiles() {
    for (const auto profile : {hand::Profile::MIN_JERK, hand::Profile::COSINE}) {
        const double duration = 0.7;
        const double h        = 1e-5;
        double worst          = 0.0;
        for (double t = 0.05; t < duration - 0.05; t += 0.01) {
            const auto p0 = hand::progress(profile, t - h, duration);
            const auto p1 = hand::progress(profile, t, duration);
            const auto p2 = hand::progress(profile, t + h, duration);
            worst         = std::max(worst, std::abs((p2.u - p0.u) / (2 * h) - p1.du));
            worst         = std::max(worst, std::abs((p2.du - p0.du) / (2 * h) - p1.ddu) * 1e-2);
        }
        const auto end = hand::progress(profile, duration, duration);
        check(
            worst < 1e-5 && hand::progress(profile, 0.0, duration).u == 0.0 && end.u == 1.0,
            profile == hand::Profile::MIN_JERK ? "min-jerk derivatives and ends" : "cosine derivatives and ends",
            worst
        );
    }
}

void static_gravity() {
    const auto g = test_geometry();
    hand::State s{.pose = pose(Quat::Identity(), Vec3::Zero(), 0, 0, 0)};
    auto t       = hand::evaluate(g, s);
    const Vec3 finger_up = g.finger_mount.transpose() * Vec3::UnitZ();
    check(
        (t.palm.specific_force_g - Vec3::UnitZ()).norm() < 1e-12
            && (t.finger.specific_force_g - finger_up).norm() < 1e-12 && t.finger.rate_dps.norm() == 0.0,
        "flat hand reads +1 g on z",
        (t.palm.specific_force_g - Vec3::UnitZ()).norm()
    );

    /* MCP at 90 deg points the finger down, so the distal frame's -y is up. */
    s.pose = pose(Quat::Identity(), Vec3::Zero(), 0, 90, 0);
    t      = hand::evaluate(g, s);
    const Vec3 expected = g.finger_mount.transpose() * -Vec3::UnitY();
    check((t.finger.specific_force_g - expected).norm() < 1e-12, "finger curled 90 deg reads +1 g on -y", 0.0);

    const Quat palm(Eigen::AngleAxisd(rad(35.0), Vec3(1.0, -2.0, 0.5).normalized()));
    s.pose = pose(palm, Vec3(0.1, 0.2, 0.3), 0, 0, 0);
    t      = hand::evaluate(g, s);
    check(
        (t.palm.specific_force_g - palm.conjugate() * Vec3::UnitZ()).norm() < 1e-12,
        "tilted palm reads gravity rotated into its frame",
        0.0
    );
}

/* The analytic rates and accelerations against finite differences of the poses. */
void derivatives() {
    const auto g  = test_geometry();
    const auto tr = test_trajectory();
    const double h = 1e-5;
    double worst_rate = 0.0, worst_vel = 0.0, worst_acc = 0.0;
    for (double t = 0.51; t < tr.duration() - 0.51; t += 0.0137) {
        const auto m = hand::evaluate(g, tr.at(t - h));
        const auto c = hand::evaluate(g, tr.at(t));
        const auto p = hand::evaluate(g, tr.at(t + h));
        for (const auto *s : {&c.palm, &c.finger}) {
            const auto *sm = s == &c.palm ? &m.palm : &m.finger;
            const auto *sp = s == &c.palm ? &p.palm : &p.finger;
            const Eigen::AngleAxisd d(sm->orientation.conjugate() * sp->orientation);
            const Vec3 rate_body = d.axis() * d.angle() / (2 * h) * (180.0 / M_PI);
            worst_rate = std::max(worst_rate, (rate_body - s->rate_dps).norm());
            worst_vel  = std::max(worst_vel, ((sp->position - sm->position) / (2 * h) - s->velocity).norm());
            const Vec3 acc = (sp->position - 2 * s->position + sm->position) / (h * h);
            worst_acc      = std::max(worst_acc, (acc - s->acceleration).norm());
        }
    }
    check(worst_rate < 1e-4, "rate matches finite differences, dps", worst_rate);
    check(worst_vel < 1e-6, "velocity matches finite differences, m/s", worst_vel);
    check(worst_acc < 1e-3, "acceleration matches finite differences, m/s^2", worst_acc);
}

/* Integrating the finger's body rate over the whole trajectory reproduces its pose. */
void integrated_rate() {
    const auto g  = test_geometry();
    const auto tr = test_trajectory();
    const double dt = 1e-4;
    Quat q          = hand::evaluate(g, tr.at(0.0)).finger.orientation;
    for (double t = 0.0; t < tr.duration() - dt / 2; t += dt) {
        const Vec3 w = hand::evaluate(g, tr.at(t + dt / 2)).finger.rate_dps * (M_PI / 180.0);
        q            = (q * Quat(Eigen::AngleAxisd(w.norm() * dt, w.norm() > 0 ? w.normalized() : Vec3::UnitX())))
                .normalized();
        if (std::abs(t - 1.3) < dt / 2) {
            const Quat truth = hand::evaluate(g, tr.at(t + dt)).finger.orientation;
            const double err = Eigen::AngleAxisd(truth.conjugate() * q).angle();
            check(err < 1e-6, "integrated rate matches the pose mid-trajectory, rad", err);
        }
    }
    const Quat truth = hand::evaluate(g, tr.at(tr.duration())).finger.orientation;
    const double err = Eigen::AngleAxisd(truth.conjugate() * q).angle();
    check(err < 1e-6, "integrated rate returns to the start pose, rad", err);

    /* A flexion-only move of an untwisted finger: the rate about the flexion axis integrates
     * to the joint sum. */
    auto straight            = g;
    straight.twist_at_90_rad = 0.0;
    hand::Trajectory flex(pose(Quat::Identity(), Vec3::Zero(), 0, 0, 0));
    flex.move(0.5, pose(Quat::Identity(), Vec3::Zero(), 0, 50, 70));
    const Vec3 axis = straight.finger_mount.transpose() * straight.flex_axis;
    double angle    = 0.0;
    for (double t = 0.0; t < 0.5 - dt / 2; t += dt) {
        angle += hand::evaluate(straight, flex.at(t + dt / 2)).finger.rate_dps.dot(axis) * dt;
    }
    const double expected = 50 + 70 * (1 + g.dip_per_pip);
    check(std::abs(angle - expected) < 1e-4, "flexion rate integrates to MCP + PIP + DIP, deg", angle - expected);
}

} // namespace

int main() {
    profiles();
    static_gravity();
    derivatives();
    integrated_rate();
    return g_failures == 0 ? 0 : 1;
}
