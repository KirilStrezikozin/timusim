/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#include "hand/chain.hpp"
#include "hand/motion.hpp"

namespace {

using hand::Vec3;

int g_failures = 0;

void check(bool ok, const char *what, double value) {
    std::printf("[%s] %s (%.4g)\n", ok ? "PASS" : "FAIL", what, value);
    if (!ok) { ++g_failures; }
}

/* Lakie et al. 2012: hand about the wrist, 120 ms muscle filter. */
constexpr hand::Oscillator WRIST{
    .inertia = 0.0025, .stiffness = 7.5, .damping = 0.03, .torque_density = 0.01, .muscle_tau_s = 0.12};

/* var(q) = integral over f of sigma^2 |muscle(f)|^2 |joint(f)|^2. */
double angle_variance(const hand::Oscillator &o) {
    double var    = 0.0;
    const double df = 0.001;
    for (double f = df / 2; f < 500.0; f += df) {
        const double w      = 2 * M_PI * f;
        const double muscle = 1.0 / (1.0 + (w * o.muscle_tau_s * w * o.muscle_tau_s));
        const double joint  = 1.0 / (std::pow(o.stiffness - (o.inertia * w * w), 2) + std::pow(o.damping * w, 2));
        var += o.torque_density * o.torque_density * muscle * joint * df;
    }
    return var;
}

/* Peak of a Welch-free periodogram average: 4 s segments, Hann, no overlap. */
double peak_hz(const std::vector<double> &x, double fs) {
    const std::size_t n = static_cast<std::size_t>(4.0 * fs);
    std::vector<double> power(n / 2, 0.0);
    for (std::size_t start = 0; start + n <= x.size(); start += n) {
        for (std::size_t k = 1; k < n / 2; ++k) {
            std::complex<double> acc = 0.0;
            for (std::size_t i = 0; i < n; ++i) {
                const double w = 0.5 - (0.5 * std::cos(2 * M_PI * static_cast<double>(i) / static_cast<double>(n)));
                acc += w * x[start + i] * std::polar(1.0, -2 * M_PI * static_cast<double>(k * i) / static_cast<double>(n));
            }
            power[k] += std::norm(acc);
        }
    }
    std::size_t best = 1;
    for (std::size_t k = 1; k < n / 2; ++k) {
        if (power[k] > power[best]) { best = k; }
    }
    return static_cast<double>(best) * fs / static_cast<double>(n);
}

std::vector<hand::JointMotion> run(const hand::Oscillator &o, double seconds, double fs, double support = 1.0) {
    hand::Tremor::Oscillators osc{};
    osc[hand::WRIST_FLEX] = o;
    hand::Pose start;
    start.support = support;
    hand::Trajectory tr(start);
    tr.hold(seconds);
    hand::Motion motion(tr, osc, 3);
    std::vector<hand::JointMotion> out;
    for (int i = 0; i < static_cast<int>(seconds * fs); ++i) {
        const auto s = motion.at(i / fs);
        out.push_back({s.pose.joints.wrist_flex, s.joints_vel.wrist_flex, s.joints_acc.wrist_flex});
    }
    return out;
}

void resonance_and_variance() {
    const double fs = 200.0;
    const auto x    = run(WRIST, 400.0, fs);
    std::vector<double> rate, angle;
    for (std::size_t i = static_cast<std::size_t>(10 * fs); i < x.size(); ++i) {
        rate.push_back(x[i].qd);
        angle.push_back(x[i].q);
    }
    const double f0 = std::sqrt(WRIST.stiffness / WRIST.inertia) / (2 * M_PI);
    /* Above the muscle corner the drive falls as 1/f, so the rate peak sits a little under f0. */
    const double fp = peak_hz(std::vector<double>(rate.begin(), rate.begin() + static_cast<long>(80 * fs)), fs);
    check(std::abs(fp - f0) < 0.5, "rate spectrum peaks near sqrt(K/I) (8.72 Hz), Hz", fp);

    double var = 0.0;
    for (double a : angle) { var += a * a; }
    var /= static_cast<double>(angle.size());
    const double expected = angle_variance(WRIST);
    check(std::abs(var / expected - 1) < 0.1, "angle variance matches the muscle and joint transfer", var / expected);
}

void added_mass() {
    hand::Oscillator loaded = WRIST;
    loaded.inertia += 0.124 * 0.07 * 0.07;
    const double fs = 200.0;
    const auto x    = run(loaded, 90.0, fs);
    std::vector<double> rate;
    for (std::size_t i = static_cast<std::size_t>(10 * fs); i < x.size(); ++i) { rate.push_back(x[i].qd); }
    const double expected = std::sqrt(loaded.stiffness / loaded.inertia) / (2 * M_PI);
    const double fp       = peak_hz(rate, fs);
    check(std::abs(fp - expected) < 0.5, "added mass moves the peak to sqrt(K/(I + m r^2)), Hz", fp);
}

/* Over 0.1 s windows, the trapezoid integral of q'' gives the change in q' and that of q' the change in q. */
void derivatives_agree() {
    const double fs = 4000.0;
    const auto x    = run(WRIST, 5.0, fs);
    const auto win  = static_cast<std::size_t>(0.1 * fs);
    double worst_q = 0.0, worst_qd = 0.0, scale_q = 0.0, scale_qd = 0.0;
    for (std::size_t start = 0; start + win < x.size(); start += win) {
        double iq = 0.0, iqd = 0.0;
        for (std::size_t i = start; i < start + win; ++i) {
            iq += (x[i].qd + x[i + 1].qd) / (2 * fs);
            iqd += (x[i].qdd + x[i + 1].qdd) / (2 * fs);
        }
        worst_q  = std::max(worst_q, std::abs(iq - (x[start + win].q - x[start].q)));
        worst_qd = std::max(worst_qd, std::abs(iqd - (x[start + win].qd - x[start].qd)));
        scale_q  = std::max(scale_q, std::abs(x[start + win].q - x[start].q));
        scale_qd = std::max(scale_qd, std::abs(x[start + win].qd - x[start].qd));
    }
    check(worst_q / scale_q < 0.01 && worst_qd / scale_qd < 0.01, "q' integrates to q and q'' to q'",
          std::max(worst_q / scale_q, worst_qd / scale_qd));
}

void off_and_support() {
    const auto none = run(hand::Oscillator{}, 2.0, 200.0);
    double peak = 0.0;
    for (const auto &m : none) { peak = std::max(peak, std::abs(m.q)); }
    check(peak == 0.0, "zero torque noise leaves the trajectory alone", peak);

    const auto braced = run(WRIST, 60.0, 200.0, 0.3);
    const auto free   = run(WRIST, 60.0, 200.0, 1.0);
    double vb = 0.0, vf = 0.0;
    for (std::size_t i = 2000; i < free.size(); ++i) {
        vb += braced[i].qd * braced[i].qd;
        vf += free[i].qd * free[i].qd;
    }
    check(std::abs(std::sqrt(vb / vf) - 0.3) < 0.05, "support scales the tremor amplitude", std::sqrt(vb / vf));
}

/* Wrist tremor reaches the palm sensor as rotation about the wrist with its lever-arm acceleration. */
void palm_sensor_sees_wrist() {
    const hand::Geometry g{
        .palm_sensor   = Vec3(0.0, 0.05, 0.02),
        .mcp           = Vec3(0.0, 0.09, 0.0),
        .proximal_m    = 0.045,
        .middle_m      = 0.025,
        .finger_sensor = Vec3(0.0, 0.01, 0.008),
        .dip_per_pip   = 0.7,
    };
    hand::State s;
    s.pose.joints.wrist_flex = 0.0;
    s.joints_vel.wrist_flex  = 0.5;
    s.joints_acc.wrist_flex  = 10.0;
    const auto t = hand::evaluate(g, s);
    const double rate_rad = t.palm.rate_dps.dot(g.flex_axis) * M_PI / 180.0;
    /* Tangential acceleration alpha x r and centripetal w^2 r about the flexion axis. */
    const Vec3 expected = g.flex_axis.cross(g.palm_sensor) * 10.0
                        + (g.flex_axis * 0.5).cross((g.flex_axis * 0.5).cross(g.palm_sensor));
    check(std::abs(rate_rad - 0.5) < 1e-12 && (t.palm.acceleration - expected).norm() < 1e-12,
          "palm sensor rate and lever-arm acceleration from wrist motion", (t.palm.acceleration - expected).norm());
}

/* Trajectory, tremor and sway together: sensor rates and accelerations against finite
 * differences of the composed poses. A second-order muscle filter keeps q'' smooth enough
 * for finite differences; the values are a test fixture. */
void composed_motion() {
    const hand::Geometry g{
        .palm_sensor   = Vec3(0.01, 0.05, 0.02),
        .mcp           = Vec3(0.0, 0.09, 0.0),
        .proximal_m    = 0.045,
        .middle_m      = 0.025,
        .finger_sensor = Vec3(0.0, 0.01, 0.008),
        .dip_per_pip   = 0.7,
    };
    hand::Tremor::Oscillators osc{};
    hand::Oscillator wrist = WRIST;
    wrist.muscle_tau_s     = 0.05;
    wrist.muscle_order     = 2;
    osc[hand::WRIST_FLEX]  = wrist;
    osc[hand::WRIST_DEV]   = wrist;
    osc[hand::MCP_FLEX]   = {.inertia = 5e-5, .stiffness = 0.03, .damping = 1e-3, .torque_density = 1e-4, .muscle_tau_s = 0.05, .muscle_order = 2};
    osc[hand::PIP_FLEX]   = {.inertia = 2e-5, .stiffness = 0.012, .damping = 3e-4, .torque_density = 5e-5, .muscle_tau_s = 0.05, .muscle_order = 2};
    for (auto dof : {hand::SWAY_RX, hand::SWAY_RY, hand::SWAY_RZ}) {
        osc[dof] = {.inertia = 1.0, .stiffness = 10.0, .damping = 4.0, .torque_density = 0.05, .muscle_tau_s = 0.3, .muscle_order = 2};
    }
    for (auto dof : {hand::SWAY_X, hand::SWAY_Y, hand::SWAY_Z}) {
        osc[dof] = {.inertia = 1.0, .stiffness = 10.0, .damping = 4.0, .torque_density = 0.005, .muscle_tau_s = 0.3, .muscle_order = 2};
    }
    hand::Pose start;
    hand::Pose bent;
    bent.palm            = hand::Quat(Eigen::AngleAxisd(0.8, Vec3(0.2, 1.0, 0.1).normalized()));
    bent.joints.mcp_flex = 0.9;
    bent.joints.pip_flex = 0.6;
    hand::Trajectory tr(start);
    tr.hold(1.0).move(1.0, bent).hold(1.0);
    hand::Motion motion(tr, osc, 9);

    /* Over several tremor steps: inside one, the Hermite position curve's own second
     * derivative differs from the interpolated q'' the sensors get. */
    const double h = 2e-3;
    double worst_rate = 0.0, worst_acc = 0.0, scale_acc = 0.0;
    for (double t = 0.5; t < 2.9; t += 0.0371) {
        const auto m = hand::evaluate(g, motion.at(t - h));
        const auto c = hand::evaluate(g, motion.at(t));
        const auto p = hand::evaluate(g, motion.at(t + h));
        for (int k = 0; k < 2; ++k) {
            const auto &sm = k == 0 ? m.palm : m.finger;
            const auto &sc = k == 0 ? c.palm : c.finger;
            const auto &sp = k == 0 ? p.palm : p.finger;
            const Eigen::AngleAxisd d(sm.orientation.conjugate() * sp.orientation);
            worst_rate     = std::max(worst_rate, (d.axis() * d.angle() / (2 * h) * (180.0 / M_PI) - sc.rate_dps).norm());
            const Vec3 acc = (sp.position - 2 * sc.position + sm.position) / (h * h);
            worst_acc      = std::max(worst_acc, (acc - sc.acceleration).norm());
            scale_acc      = std::max(scale_acc, sc.acceleration.norm());
        }
    }
    check(worst_rate < 0.5, "composed motion: sensor rate matches finite differences, dps", worst_rate);
    check(worst_acc / scale_acc < 0.01, "composed motion: acceleration matches finite differences", worst_acc / scale_acc);
}

} // namespace

int main() {
    composed_motion();
    resonance_and_variance();
    added_mass();
    derivatives_agree();
    off_and_support();
    palm_sensor_sees_wrist();
    return g_failures == 0 ? 0 : 1;
}
