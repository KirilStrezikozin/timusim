/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Two SimIMUs on a simulated hand, polled through IIMU on the host virtual clock.
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "chip/sim_imu.hpp"
#include "hand/chain.hpp"
#include "hand/trajectory.hpp"
#include "imu/conformance.hpp"
#include "time/sleep.h"

namespace {

using hand::Vec3;
using imu::conformance::vector_type;

int g_failures = 0;

void check(bool ok, const char *what, double value) {
    std::printf("[%s] %s (%.4g)\n", ok ? "PASS" : "FAIL", what, value);
    if (!ok) { ++g_failures; }
}

void must(const tl::expected<void, imu::IMUError> &r) {
    if (!r) {
        std::printf("[FAIL] unexpected IMU error\n");
        std::exit(1);
    }
}

double rad(double deg) { return deg * M_PI / 180.0; }

/* Test fixtures only. */
const hand::Geometry GEOMETRY{
    .palm_sensor   = Vec3(0.0, 0.05, 0.02),
    .mcp           = Vec3(0.0, 0.09, 0.0),
    .proximal_m    = 0.045,
    .middle_m      = 0.025,
    .finger_sensor = Vec3(0.0, 0.01, 0.008),
    .dip_per_pip   = 0.7,
};

chip::lsm9ds1::Params params() {
    chip::lsm9ds1::Params p;
    p.odr_scale             = 461.534 / 476.0;
    p.accel.white_density   = Vec3::Constant(100e-6);
    p.gyro.white_density    = Vec3::Constant(0.015);
    p.gyro.turn_on_sigma    = Vec3::Constant(1.0);
    p.accel_aa              = {chip::FilterKind::BUTTERWORTH, 2};
    p.accel_lp              = {chip::FilterKind::ONE_POLE, 1};
    p.gyro_lpf1             = {chip::FilterKind::BUTTERWORTH, 2};
    return p;
}

chip::SimIMU::Config config() {
    chip::SimIMU::Config c;
    c.imu_odr       = LSM9DS1_IMU_476Hz;
    c.accel_range   = LSM9DS1_4g;
    c.gyro_range    = LSM9DS1_2000dps;
    c.gyro_out_path = LSM9DS1_LPF1_OUT;
    return c;
}

hand::Pose pose(double turn_deg, Vec3 axis, double mcp, double pip) {
    return hand::Pose{
        .palm   = hand::Quat(Eigen::AngleAxisd(rad(turn_deg), axis.normalized())),
        .wrist  = Vec3::Zero(),
        .joints = {.mcp_abd = 0.0, .mcp_flex = rad(mcp), .pip_flex = rad(pip)},
    };
}

hand::Trajectory routine() {
    hand::Trajectory tr(pose(0, Vec3::UnitX(), 0, 0));
    tr.hold(5.0);
    for (int i = 0; i < 4; ++i) {
        tr.move(1.2, pose(60, Vec3(1, 0.5, 0), 40, 60))
            .move(1.2, pose(-50, Vec3(0.2, 1, 0.3), 10, 20))
            .move(1.2, pose(70, Vec3(0, 0.4, 1), 70, 90))
            .move(1.2, pose(0, Vec3::UnitX(), 0, 0));
    }
    return tr;
}

struct Rig {
    hand::Trajectory trajectory = routine();
    chip::SimIMU palm{config(), params(), 1, [this](double t) {
                          const auto s = hand::evaluate(GEOMETRY, this->trajectory.at(t)).palm;
                          return std::pair{s.specific_force_g, s.rate_dps};
                      }};
    chip::SimIMU finger{config(), params(), 2, [this](double t) {
                            const auto s = hand::evaluate(GEOMETRY, this->trajectory.at(t)).finger;
                            return std::pair{s.specific_force_g, s.rate_dps};
                        }};
};

void polled_at_200hz() {
    vclock::set_us(0);
    Rig rig;
    imu::IIMU *imus[2] = {&rig.palm, &rig.finger};
    for (imu::IIMU *i : imus) { check(i->init().has_value(), "init", 0); }

    imu::conformance::RestCheck rest[2] = {
        imu::conformance::RestCheck(vector_type(0.0F, 0.0F, 1.0F)),
        imu::conformance::RestCheck(vector_type(0.0F, 0.0F, 1.0F)),
    };
    imu::conformance::HandednessCheck handed[2];
    int polls = 0, fresh = 0;
    while (static_cast<double>(vclock::now_us()) * 1e-6 < rig.trajectory.duration()) {
        platform_sleep_ms(5);
        ++polls;
        for (int k = 0; k < 2; ++k) {
            if (!imus[k]->get_is_data_ready().value()) { continue; }
            ++fresh;
            vector_type a, g;
            must(imus[k]->get_accel_data(a));
            must(imus[k]->get_gyro_data(g));
            rest[k].update(a, g);
            handed[k].update(a, g - rest[k].gyro_bias_dps(), 0.005F);
        }
    }
    check(fresh == 2 * polls, "at 200 Hz polling of a 461.5 Hz chip, every poll has a new sample", fresh);
    for (int k = 0; k < 2; ++k) {
        const auto r = rest[k].result();
        const auto h = handed[k].result();
        std::printf(
            "  %s: rest |a|-1=%.4f g bias=(%.2f %.2f %.2f) dps, handed corr=%.3f gain=%.3f blocks=%u\n",
            k == 0 ? "palm" : "finger", r.norm_error_g, r.mean_gyro_dps.x(), r.mean_gyro_dps.y(), r.mean_gyro_dps.z(),
            h.correlation, h.gain, static_cast<unsigned>(h.blocks)
        );
        check(r.pass && h.pass, k == 0 ? "palm SimIMU passes conformance" : "finger SimIMU passes conformance", h.correlation);
    }
}

void fifo() {
    vclock::set_us(0);
    Rig rig;
    must(rig.finger.init());
    must(rig.finger.enable_fifo());
    imu::IIMU::RawSample buf[chip::SimIMU::FIFO_SLOTS];
    std::size_t total = 0;
    bool any_overrun  = false;
    for (int i = 0; i < 500; ++i) {
        platform_sleep_ms(20);
        bool overrun = false;
        total += rig.finger.read_fifo({buf, chip::SimIMU::FIFO_SLOTS}, overrun).value();
        any_overrun = any_overrun || overrun;
    }
    const double expected = 10.0 * rig.finger.odr_hz();
    check(!any_overrun && std::abs(static_cast<double>(total) - expected) <= 1, "FIFO drained every 20 ms keeps every sample", total);

    platform_sleep_ms(100);
    bool overrun = false;
    const auto n = rig.finger.read_fifo({buf, chip::SimIMU::FIFO_SLOTS}, overrun).value();
    check(overrun && n == chip::SimIMU::FIFO_SLOTS, "a 100 ms gap overruns the 32-slot FIFO", n);
}

void errors() {
    vclock::set_us(0);
    Rig rig;
    vector_type v;
    check(rig.palm.get_accel_data(v).error() == imu::IMUError::NOT_INITIALIZED, "reads before init fail", 0);
    must(rig.palm.init());
    check(rig.palm.get_mag_data(v).error() == imu::IMUError::UNSUPPORTED_OPERATION, "magnetometer is unsupported", 0);
    check(std::abs(rig.palm.get_temperature_c().value() - 25.0F) < 1e-6F, "temperature is the model's", 25.0);

    auto bad    = config();
    bad.imu_odr = LSM9DS1_IMU_14Hz9;
    chip::SimIMU s(bad, params(), 1, [](double) { return std::pair{Vec3::UnitZ().eval(), Vec3::Zero().eval()}; });
    check(s.init().error() == imu::IMUError::INVALID_CONFIG, "an unsupported config fails init", 0);
}

/* Each init is a power-up: a new turn-on bias. */
void power_cycles() {
    vclock::set_us(0);
    chip::SimIMU s(config(), params(), 5, [](double) { return std::pair{Vec3::UnitZ().eval(), Vec3::Zero().eval()}; });
    auto mean_gyro_x = [&]() {
        must(s.init());
        double sum = 0.0;
        for (int i = 0; i < 200; ++i) {
            platform_sleep_ms(5);
            vector_type g;
            must(s.get_gyro_data(g));
            sum += g.x();
        }
        return sum / 200.0;
    };
    const double a = mean_gyro_x();
    const double b = mean_gyro_x();
    check(std::abs(a - b) > 0.05, "re-init draws a new turn-on bias", a - b);
}

} // namespace

int main() {
    polled_at_200hz();
    fifo();
    errors();
    power_cycles();
    return g_failures == 0 ? 0 : 1;
}
