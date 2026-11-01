/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * timusim-sim: runs a scenario through two simulated chips and writes the
 * capture stream the stm32-capture firmware writes, plus a truth CSV. A sensor
 * with an `input` CSV takes that recorded input instead of the hand model's.
 *
 *   timusim-sim SCENARIO.toml -o OUT.bin [--truth OUT.csv | --no-truth] [--seed N]
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#include "capture/protocol.hpp"
#include "chip/sim_imu.hpp"
#include "scenario.hpp"
#include "hand/motion.hpp"
#include "time/sleep.h"

namespace {

/* The firmware drains both FIFOs in a tight loop; 2 ms keeps batches small as on the board. */
constexpr std::uint32_t POLL_US          = 2000;
constexpr std::uint64_t HEADER_PERIOD_US = 5'000'000;
constexpr std::uint64_t TEMP_PERIOD_US   = 1'000'000;

struct Stream {
    const char *name;
    chip::SimIMU *imu;
    std::uint32_t next_index = 0;
};

std::string format(const char *fmt, auto... args) {
    char buf[512];
    std::snprintf(buf, sizeof(buf), fmt, args...);
    return buf;
}

class Writer {
  public:
    explicit Writer(std::ofstream &out) : out_(out) {}

    void text(capture::PacketType type, const std::string &s) {
        capture::write_text(this->w_, type, {s.data(), s.size()});
        this->flush_();
    }
    void samples(std::uint8_t id, std::uint32_t first, std::uint32_t t_us, bool overrun,
                 span<const imu::IIMU::RawSample> batch) {
        capture::write_samples(this->w_, id, first, t_us, overrun, batch);
        this->flush_();
    }
    void temp(std::uint8_t id, std::uint32_t t_us, float c) {
        capture::write_temp(this->w_, id, t_us, c);
        this->flush_();
    }

  private:
    void flush_() {
        const auto frame = this->w_.finish();
        this->out_.write(reinterpret_cast<const char *>(frame.data()), static_cast<std::streamsize>(frame.size()));
    }

    std::ofstream &out_;
    capture::PacketWriter w_;
};

void send_header(Writer &w, const sim::Scenario &s, const Stream (&streams)[2]) {
    w.text(
        capture::PacketType::HEADER,
        format(
            "fw=timusim-sim\nconfig=%s\nuid=sim\nseed=%llu\nchip_transfer=%d\nnoise=%d\ntremor=%d\nsensors=2\nt_us=%llu\n",
            s.name.c_str(), static_cast<unsigned long long>(s.seed), s.realism.chip_transfer ? 1 : 0,
            s.realism.noise ? 1 : 0, s.realism.tremor ? 1 : 0, static_cast<unsigned long long>(vclock::now_us())
        )
    );
    for (int i = 0; i < 2; ++i) {
        const Stream &st = streams[i];
        const auto &c    = st.imu->path().value();
        const auto scale = st.imu->get_raw_scale().value();
        w.text(
            capture::PacketType::HEADER,
            format(
                "s%d.name=%s\ns%d.chip=%s\ns%d.odr_hz=%g\ns%d.accel_fs_g=%g\ns%d.gyro_fs_dps=%g\n"
                "s%d.accel_lsb_g=%.9g\ns%d.gyro_lsb_dps=%.9g\ns%d.accel_bw=%d\ns%d.gyro_bw=%d\ns%d.gyro_out_path=%d\n"
                "s%d.next_index=%u\n",
                i, st.name, i, c.chip, i, c.nominal_odr_hz, i, c.accel_fs_g, i, c.gyro_fs_dps, i,
                static_cast<double>(scale.accel_g_per_lsb), i, static_cast<double>(scale.gyro_dps_per_lsb), i,
                c.accel_bw, i, c.gyro_bw, i, c.gyro_out_path, i, st.next_index
            )
        );
    }
}

int usage() {
    std::fprintf(stderr, "usage: timusim-sim SCENARIO.toml -o OUT.bin [--truth OUT.csv | --no-truth] [--seed N]\n");
    return 2;
}

} // namespace

int main(int argc, char **argv) {
    const char *scenario_path = nullptr;
    std::string out_path, truth_path;
    std::optional<std::uint64_t> seed;
    bool write_truth = true;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else if (std::strcmp(argv[i], "--truth") == 0 && i + 1 < argc) {
            truth_path = argv[++i];
        } else if (std::strcmp(argv[i], "--no-truth") == 0) {
            write_truth = false;
        } else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            seed = std::strtoull(argv[++i], nullptr, 10);
        } else if (argv[i][0] != '-' && scenario_path == nullptr) {
            scenario_path = argv[i];
        } else {
            return usage();
        }
    }
    if (scenario_path == nullptr || out_path.empty()) { return usage(); }
    if (truth_path.empty()) { truth_path = std::filesystem::path(out_path).replace_extension(".truth.csv").string(); }

    auto loaded = sim::load_scenario(scenario_path);
    if (!loaded) {
        std::fprintf(stderr, "%s\n", loaded.error().c_str());
        return 1;
    }
    sim::Scenario s = std::move(*loaded);
    if (seed) { s.seed = *seed; }

    hand::Motion motion(s.trajectory, s.tremor, (2 * s.seed) + 7, s.drives);
    /* Segment speeds for the mount jolts, from the scripted joint rates. */
    const hand::Trajectory &scripted = motion.trajectory();
    hand::Jolt jolts[2] = {
        hand::Jolt(s.palm_jolt, (2 * s.seed) + 11, [&scripted](double t) {
            const auto v = scripted.at(t).joints_vel;
            return std::hypot(v.wrist_flex, v.wrist_dev);
        }),
        hand::Jolt(s.finger_jolt, (2 * s.seed) + 13, [&scripted](double t) {
            const auto v = scripted.at(t).joints_vel;
            return std::sqrt(std::pow(v.wrist_flex + v.mcp_flex + v.pip_flex, 2) + std::pow(v.wrist_dev, 2) + std::pow(v.mcp_abd, 2));
        }),
    };
    auto truth_of = [&](bool finger) {
        return [&s, &motion, &jolts, finger](double t) {
            const auto h = hand::evaluate(s.geometry, motion.at(t));
            const auto &x = finger ? h.finger : h.palm;
            const auto j = jolts[finger ? 1 : 0].at(t);
            /* The chip turned by the small angle j sees the force turned back: f - angle x f. */
            return std::pair{hand::Vec3(x.specific_force_g - j.angle_rad.cross(x.specific_force_g)), hand::Vec3(x.rate_dps + j.rate_dps)};
        };
    };
    vclock::set_us(0);
    auto make = [](const sim::Sensor &sensor, std::uint64_t chip_seed, chip::SimIMU::Truth truth) {
        return std::visit(
            [&](const auto &config) { return chip::SimIMU(config, sensor.params, chip_seed, std::move(truth)); },
            sensor.config
        );
    };
    auto input_or = [](const sim::Sensor &sensor, chip::SimIMU::Truth truth) -> chip::SimIMU::Truth {
        if (!sensor.input) { return truth; }
        return [&input = *sensor.input](double t) { return input.at(t); };
    };
    chip::SimIMU palm   = make(s.palm, 2 * s.seed, input_or(s.palm, truth_of(false)));
    chip::SimIMU finger = make(s.finger, (2 * s.seed) + 1, input_or(s.finger, truth_of(true)));
    Stream streams[2]   = {{"palm", &palm}, {"finger", &finger}};
    for (Stream &st : streams) {
        if (!st.imu->init() || !st.imu->enable_fifo()) {
            std::fprintf(stderr, "%s: invalid chip config\n", st.name);
            return 1;
        }
    }

    std::ofstream out(out_path, std::ios::binary);
    std::FILE *truth = write_truth ? std::fopen(truth_path.c_str(), "w") : nullptr;
    if (!out || (write_truth && truth == nullptr)) {
        std::fprintf(stderr, "cannot write %s or %s\n", out_path.c_str(), truth_path.c_str());
        return 1;
    }
    if (truth != nullptr) {
        std::fprintf(
            truth, "sensor,index,t_s,fx_g,fy_g,fz_g,wx_dps,wy_dps,wz_dps,qw,qx,qy,qz,px_m,py_m,pz_m,mcp_abd_deg,"
                   "mcp_flex_deg,pip_flex_deg\n"
        );
    }

    Writer w(out);
    w.text(capture::PacketType::TEXT, "Starting up...");
    send_header(w, s, streams);
    std::uint64_t last_header = 0, last_temp = 0;
    const auto end_us = static_cast<std::uint64_t>(motion.duration() * 1e6);
    imu::IIMU::RawSample batch[capture::MAX_BATCH];
    while (vclock::now_us() < end_us) {
        platform_sleep_us(POLL_US);
        const std::uint64_t now = vclock::now_us();
        for (std::uint8_t id = 0; id < 2; ++id) {
            Stream &st   = streams[id];
            bool overrun = false;
            const auto n = st.imu->read_fifo({batch, capture::MAX_BATCH}, overrun).value();
            if (n == 0 && !overrun) { continue; }
            w.samples(id, st.next_index, static_cast<std::uint32_t>(now), overrun, {batch, n});
            for (std::size_t k = 0; truth != nullptr && k < n; ++k) {
                const std::uint32_t index = st.next_index + static_cast<std::uint32_t>(k);
                const double t            = st.imu->sample_time_s(index);
                const auto state          = motion.at(t);
                const auto h              = hand::evaluate(s.geometry, state);
                const auto &x             = id == 1 ? h.finger : h.palm;
                const auto &q             = x.orientation;
                const auto &j             = state.pose.joints;
                std::fprintf(
                    truth, "%s,%u,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n",
                    st.name, index, t, x.specific_force_g.x(), x.specific_force_g.y(), x.specific_force_g.z(),
                    x.rate_dps.x(), x.rate_dps.y(), x.rate_dps.z(), q.w(), q.x(), q.y(), q.z(), x.position.x(),
                    x.position.y(), x.position.z(), j.mcp_abd * 180.0 / M_PI, j.mcp_flex * 180.0 / M_PI,
                    j.pip_flex * 180.0 / M_PI
                );
            }
            st.next_index += static_cast<std::uint32_t>(n);
        }
        if (now - last_temp >= TEMP_PERIOD_US) {
            last_temp = now;
            for (std::uint8_t id = 0; id < 2; ++id) {
                w.temp(id, static_cast<std::uint32_t>(now), streams[id].imu->get_temperature_c().value());
            }
        }
        if (now - last_header >= HEADER_PERIOD_US) {
            last_header = now;
            send_header(w, s, streams);
        }
    }
    if (truth != nullptr) { std::fclose(truth); }
    std::printf(
        "%s: palm %u samples at %.3f Hz, finger %u samples at %.3f Hz, %.1f s\n", out_path.c_str(),
        streams[0].next_index, palm.odr_hz(), streams[1].next_index, finger.odr_hz(), motion.duration()
    );
    return 0;
}
