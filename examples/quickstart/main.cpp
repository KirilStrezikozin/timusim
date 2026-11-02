/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Runs a scenario through the simulated palm and finger IMUs and writes a CSV: each output
 * sample in g and dps, next to the ideal input the chip received at that time.
 *
 *   timusim-quickstart SCENARIO.toml OUT.csv
 */

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <utility>

#include "chip/sim_imu.hpp"
#include "hand/jolt.hpp"
#include "hand/motion.hpp"
#include "scenario.hpp"
#include "time/sleep.h"

namespace {

constexpr std::uint32_t POLL_US = 2000;

chip::SimIMU make_imu(const sim::Sensor &sensor, std::uint64_t seed, chip::SimIMU::Truth truth) {
    return std::visit(
        [&](const auto &config) { return chip::SimIMU(config, sensor.params, seed, std::move(truth)); }, sensor.config
    );
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: timusim-quickstart SCENARIO.toml OUT.csv\n");
        return 2;
    }
    auto loaded = sim::load_scenario(argv[1]);
    if (!loaded) {
        std::fprintf(stderr, "%s\n", loaded.error().c_str());
        return 1;
    }
    sim::Scenario s = std::move(*loaded);

    hand::Motion motion(s.trajectory, s.tremor, (2 * s.seed) + 7, s.drives);
    const hand::Trajectory &scripted = motion.trajectory();
    hand::Jolt palm_jolt(s.palm_jolt, (2 * s.seed) + 11, [&scripted](double t) {
        const auto v = scripted.at(t).joints_vel;
        return std::hypot(v.wrist_flex, v.wrist_dev);
    });
    hand::Jolt finger_jolt(s.finger_jolt, (2 * s.seed) + 13, [&scripted](double t) {
        const auto v = scripted.at(t).joints_vel;
        return std::sqrt(std::pow(v.wrist_flex + v.mcp_flex + v.pip_flex, 2) + std::pow(v.wrist_dev, 2) + std::pow(v.mcp_abd, 2));
    });

    /* The chip input at time t: the chain's reading, plus the sensor rocking on its mount. */
    auto ideal = [&](bool is_finger) {
        return [&, is_finger](double t) {
            const auto h  = hand::evaluate(s.geometry, motion.at(t));
            const auto &x = is_finger ? h.finger : h.palm;
            const auto j  = (is_finger ? finger_jolt : palm_jolt).at(t);
            return std::pair{hand::Vec3(x.specific_force_g - j.angle_rad.cross(x.specific_force_g)), hand::Vec3(x.rate_dps + j.rate_dps)};
        };
    };

    vclock::set_us(0);
    chip::SimIMU palm   = make_imu(s.palm, 2 * s.seed, ideal(false));
    chip::SimIMU finger = make_imu(s.finger, (2 * s.seed) + 1, ideal(true));
    if (!palm.init() || !palm.enable_fifo() || !finger.init() || !finger.enable_fifo()) {
        std::fprintf(stderr, "invalid chip config\n");
        return 1;
    }

    const std::filesystem::path out_path(argv[2]);
    if (out_path.has_parent_path()) { std::filesystem::create_directories(out_path.parent_path()); }
    std::FILE *out = std::fopen(argv[2], "w");
    if (out == nullptr) {
        std::fprintf(stderr, "cannot write %s\n", argv[2]);
        return 1;
    }
    std::fprintf(out, "sensor,t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,ideal_ax_g,ideal_ay_g,ideal_az_g,ideal_gx_dps,ideal_gy_dps,ideal_gz_dps\n");

    struct Stream {
        const char *name;
        chip::SimIMU *imu;
        bool is_finger;
        std::uint32_t count = 0;
    } streams[2] = {{"palm", &palm, false}, {"finger", &finger, true}};

    imu::IIMU::RawSample batch[32];
    const auto end_us = static_cast<std::uint64_t>(motion.duration() * 1e6);
    while (vclock::now_us() < end_us) {
        platform_sleep_us(POLL_US);
        for (Stream &st : streams) {
            bool overrun = false;
            const auto n = st.imu->read_fifo({batch, 32}, overrun).value();
            const auto scale = st.imu->get_raw_scale().value();
            const double g   = scale.accel_g_per_lsb;
            const double dps = scale.gyro_dps_per_lsb;
            for (std::size_t k = 0; k < n; ++k) {
                const double t   = st.imu->sample_time_s(st.count++);
                const auto [f, w] = ideal(st.is_finger)(t);
                const auto &r    = batch[k];
                std::fprintf(
                    out, "%s,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n", st.name, t, r.accel[0] * g,
                    r.accel[1] * g, r.accel[2] * g, r.gyro[0] * dps, r.gyro[1] * dps, r.gyro[2] * dps, f.x(), f.y(), f.z(),
                    w.x(), w.y(), w.z()
                );
            }
        }
    }
    std::fclose(out);
    std::printf(
        "%s: palm %u samples at %.1f Hz, finger %u samples at %.1f Hz, %.1f s\n", argv[2], streams[0].count, palm.odr_hz(),
        streams[1].count, finger.odr_hz(), motion.duration()
    );
    return 0;
}
