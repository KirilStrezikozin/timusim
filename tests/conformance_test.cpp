/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdio>
#include <functional>
#include <random>

#include <Eigen/Geometry>

#include "imu/conformance.hpp"
#include "imu/lsm9ds1/i2c.hpp"
#include "time/sleep.h"

namespace {

using imu::conformance::vector_type;

int g_failures = 0;

void check(bool ok, const char *what) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) { ++g_failures; }
}

struct Sample {
    vector_type accel_g;
    vector_type gyro_dps;
};

using Mutation = std::function<Sample(Sample)>;

constexpr double ODR_HZ = 476.0;

/* 5 s face up and still, then 20 s of slow rotation about a wandering axis. */
template <typename Feed> void drive(const Mutation &mutate, double noise_g, Feed feed) {
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 1.0);
    const double dt        = 1.0 / ODR_HZ;
    Eigen::Quaterniond q   = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();

    for (int i = 0; i < static_cast<int>(25.0 * ODR_HZ); ++i) {
        const double t = i * dt;
        Eigen::Vector3d omega_rad = Eigen::Vector3d::Zero();
        if (t >= 5.0) {
            omega_rad = Eigen::Vector3d(std::sin(0.7 * t), std::cos(0.5 * t), 0.6 * std::sin(0.3 * t))
                      * 1.2;
        }
        q = (q * Eigen::Quaterniond(Eigen::AngleAxisd(omega_rad.norm() * dt,
                                                      omega_rad.norm() > 0.0 ? omega_rad.normalized()
                                                                             : up)))
                .normalized();

        const Eigen::Vector3d accel = q.conjugate() * up;
        const Eigen::Vector3d gyro  = omega_rad * (180.0 / M_PI);

        Sample s{
            vector_type(static_cast<float>(accel.x() + noise_g * noise(rng)),
                        static_cast<float>(accel.y() + noise_g * noise(rng)),
                        static_cast<float>(accel.z() + noise_g * noise(rng))),
            vector_type(static_cast<float>(gyro.x()), static_cast<float>(gyro.y()),
                        static_cast<float>(gyro.z())),
        };
        feed(mutate(s), static_cast<float>(dt));
    }
}

struct Outcome {
    imu::conformance::RestResult rest;
    imu::conformance::HandednessResult handed;
};

Outcome run(const Mutation &mutate, double noise_g = 0.002) {
    imu::conformance::RestCheck rest(vector_type(0.0F, 0.0F, 1.0F));
    imu::conformance::HandednessCheck handed;
    drive(mutate, noise_g, [&](const Sample &s, float dt) {
        rest.update(s.accel_g, s.gyro_dps);
        handed.update(s.accel_g, s.gyro_dps - rest.gyro_bias_dps(), dt);
    });
    const Outcome o{rest.result(), handed.result()};
    std::printf(
        "  rest: n=%u |a|-1=%.4f g up_err=%.2f deg   handed: blocks=%u accel_only=%u corr=%.3f gain=%.3f\n",
        static_cast<unsigned>(o.rest.samples), o.rest.norm_error_g, o.rest.up_error_deg,
        static_cast<unsigned>(o.handed.blocks), static_cast<unsigned>(o.handed.accel_only_blocks),
        o.handed.correlation, o.handed.gain
    );
    return o;
}

} // namespace

int main() {
    {
        const auto o = run([](Sample s) { return s; });
        check(o.rest.pass && o.handed.pass, "conforming stream passes");
    }
    {
        const auto o = run([](Sample s) { return s; }, 0.02);
        check(o.rest.pass && o.handed.pass, "conforming stream with 20 mg noise passes");
    }
    {
        const auto o = run([](Sample s) {
            s.accel_g.z()  = -s.accel_g.z();
            s.gyro_dps.z() = -s.gyro_dps.z();
            return s;
        });
        check(!o.handed.pass && o.handed.correlation < -0.8F, "left-handed frame fails with negative correlation");
    }
    {
        const auto o = run([](Sample s) {
            s.gyro_dps += vector_type(20.0F, -10.0F, 5.0F);
            return s;
        });
        check(o.rest.pass && o.handed.pass, "gyro zero-rate offset of 23 dps passes");
    }
    {
        const auto o = run([](Sample s) {
            s.gyro_dps = s.gyro_dps * 0.0174532925F;
            return s;
        });
        check(!o.handed.pass && o.handed.accel_only_blocks > o.handed.blocks, "gyro in rad/s fails on accel-only rotation");
    }
    {
        const auto o = run([](Sample s) {
            s.accel_g = s.accel_g * -1.0F;
            return s;
        });
        check(!o.rest.pass && o.handed.pass, "negated accel passes handedness but fails rest");
    }
    {
        const auto o = run([](Sample s) {
            s.accel_g = s.accel_g * 1.2F;
            return s;
        });
        check(!o.rest.pass, "20% accel scale error fails rest");
    }
    {
        vclock::set_us(0);
        platform_sleep_ms(3);
        platform_sleep_us(250);
        check(vclock::now_us() == 3250, "host sleeps advance the virtual clock");
    }

    std::printf("%s (%d failed checks)\n", g_failures == 0 ? "ALL CHECKS PASSED" : "CHECKS FAILED",
                g_failures);
    return g_failures == 0 ? 0 : 1;
}
