/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Scenario and chip-parameter files (TOML). sim/scenarios/README.md lists the keys.
 */

#ifndef SIM_SCENARIO_HPP
#define SIM_SCENARIO_HPP

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <tl/expected.hpp>
#include <toml.hpp>

#include "chip/lsm6dsl.hpp"
#include "chip/lsm9ds1.hpp"
#include "hand/jolt.hpp"
#include "hand/chain.hpp"
#include "hand/trajectory.hpp"
#include "hand/tremor.hpp"
#include "replay.hpp"

namespace sim {

using Error = std::string;

struct Sensor {
    std::variant<chip::lsm9ds1::Config, chip::lsm6dsl::Config> config;
    chip::Params params;
    /* Replaces the hand model's truth for this sensor when set. */
    std::optional<Replay> input;
};

/* Ladder rungs, each on top of the previous one. */
struct Realism {
    bool chip_transfer = true; /* filters; off leaves the ideal values, quantized and clipped */
    bool noise         = true; /* white noise, zero level and bias drift */
    bool tremor        = true; /* joint tremor, when [tremor] sets any torque noise */
};

struct Scenario {
    std::string name;
    std::uint64_t seed = 1;
    Realism realism{};
    hand::Geometry geometry{};
    hand::Trajectory trajectory{hand::Pose{}};
    hand::Tremor::Oscillators tremor{};
    std::vector<hand::SharedDrive> drives;
    hand::JoltParams palm_jolt{};
    hand::JoltParams finger_jolt{};
    Sensor palm{};
    Sensor finger{};
};

namespace detail {

inline double rad(double deg) {
    return deg * M_PI / 180.0;
}

inline tl::expected<hand::Vec3, Error> vec3(
    const toml::node_view<const toml::node> &n, const std::string &key
) {
    const auto *arr = n.as_array();
    if (arr == nullptr || arr->size() != 3) {
        return tl::make_unexpected(key + ": expected [x, y, z]");
    }
    hand::Vec3 v;
    for (std::size_t i = 0; i < 3; ++i) {
        auto x = (*arr)[i].value<double>();
        if (!x) { return tl::make_unexpected(key + ": expected numbers"); }
        v[static_cast<Eigen::Index>(i)] = *x;
    }
    return v;
}

/* { axis = [x, y, z], deg = a }; absent is the identity. */
inline tl::expected<hand::Quat, Error> rotation(
    const toml::node_view<const toml::node> &n, const std::string &key
) {
    if (!n) { return hand::Quat::Identity(); }
    auto axis = vec3(n["axis"], key + ".axis");
    if (!axis) { return tl::make_unexpected(axis.error()); }
    const double deg = n["deg"].value_or(0.0);
    if (axis->norm() == 0.0) { return tl::make_unexpected(key + ".axis: zero vector"); }
    return hand::Quat(Eigen::AngleAxisd(rad(deg), axis->normalized()));
}

template <typename T> struct Choice {
    const char *name;
    T value;
};

template <typename T, std::size_t N>
tl::expected<T, Error> choose(
    const toml::node_view<const toml::node> &n,
    const Choice<T> (&choices)[N],
    T fallback,
    const std::string &key
) {
    if (!n) { return fallback; }
    if (auto s = n.value<std::string>()) {
        for (const auto &c : choices) {
            if (*s == c.name) { return c.value; }
        }
    }
    if (auto x = n.value<double>()) {
        for (const auto &c : choices) {
            char *end = nullptr;
            if (std::strtod(c.name, &end) == *x && *end == '\0') { return c.value; }
        }
    }
    return tl::make_unexpected(key + ": unknown value");
}

/* Defaults are the capture firmware's: 476 Hz, +-4 g, +-2000 dps, gyro LPF1 only. */
inline tl::expected<chip::lsm9ds1::Config, Error> lsm9ds1_config(
    const toml::node_view<const toml::node> &n, const std::string &key
) {
    static constexpr Choice<lsm9ds1_imu_odr_t> ODR[] = {
        {"119", LSM9DS1_IMU_119Hz},
        {"238", LSM9DS1_IMU_238Hz},
        {"476", LSM9DS1_IMU_476Hz},
        {"952", LSM9DS1_IMU_952Hz}
    };
    static constexpr Choice<lsm9ds1_xl_fs_t> XL_FS[] = {
        {"2", LSM9DS1_2g}, {"4", LSM9DS1_4g}, {"8", LSM9DS1_8g}, {"16", LSM9DS1_16g}
    };
    static constexpr Choice<lsm9ds1_gy_fs_t> GY_FS[] = {
        {"245", LSM9DS1_245dps}, {"500", LSM9DS1_500dps}, {"2000", LSM9DS1_2000dps}
    };
    static constexpr Choice<lsm9ds1_gy_out_path_t> PATH[] = {
        {"lpf1", LSM9DS1_LPF1_OUT}, {"lpf1_lpf2", LSM9DS1_LPF1_LPF2_OUT}
    };
    static constexpr Choice<lsm9ds1_gy_lp_bw_t> GY_BW[] = {
        {"strong", LSM9DS1_LP_STRONG},
        {"medium", LSM9DS1_LP_MEDIUM},
        {"light", LSM9DS1_LP_LIGHT},
        {"ultra_light", LSM9DS1_LP_ULTRA_LIGHT}
    };
    static constexpr Choice<lsm9ds1_xl_lp_bw_t> XL_BW[] = {
        {"off", LSM9DS1_LP_DISABLE},
        {"odr_div_50", LSM9DS1_LP_ODR_DIV_50},
        {"odr_div_100", LSM9DS1_LP_ODR_DIV_100},
        {"odr_div_9", LSM9DS1_LP_ODR_DIV_9},
        {"odr_div_400", LSM9DS1_LP_ODR_DIV_400}
    };

    chip::lsm9ds1::Config c;
    auto odr  = choose(n["odr_hz"], ODR, LSM9DS1_IMU_476Hz, key + ".odr_hz");
    auto xfs  = choose(n["accel_range_g"], XL_FS, LSM9DS1_4g, key + ".accel_range_g");
    auto gfs  = choose(n["gyro_range_dps"], GY_FS, LSM9DS1_2000dps, key + ".gyro_range_dps");
    auto path = choose(n["gyro_out_path"], PATH, LSM9DS1_LPF1_OUT, key + ".gyro_out_path");
    auto gbw  = choose(n["gyro_bw"], GY_BW, LSM9DS1_LP_ULTRA_LIGHT, key + ".gyro_bw");
    auto xbw  = choose(n["accel_bw"], XL_BW, LSM9DS1_LP_ODR_DIV_9, key + ".accel_bw");
    if (!odr) { return tl::make_unexpected(odr.error()); }
    if (!xfs) { return tl::make_unexpected(xfs.error()); }
    if (!gfs) { return tl::make_unexpected(gfs.error()); }
    if (!path) { return tl::make_unexpected(path.error()); }
    if (!gbw) { return tl::make_unexpected(gbw.error()); }
    if (!xbw) { return tl::make_unexpected(xbw.error()); }
    c.imu_odr       = *odr;
    c.accel_range   = *xfs;
    c.gyro_range    = *gfs;
    c.gyro_out_path = *path;
    c.gyro_bw       = *gbw;
    c.accel_bw      = *xbw;
    return c;
}

/* Defaults are the capture firmware's palm settings: 416 Hz, +-4 g, +-2000 dps, gyro LPF1
 * FTYPE 00, accel LPF2 at ODR/9 fed from LPF1 at ODR/2. */
inline tl::expected<chip::lsm6dsl::Config, Error> lsm6dsl_config(
    const toml::node_view<const toml::node> &n, const std::string &key
) {
    static constexpr Choice<lsm6dsl_odr_xl_t> ODR[] = {
        {"104", LSM6DSL_XL_ODR_104Hz},
        {"208", LSM6DSL_XL_ODR_208Hz},
        {"416", LSM6DSL_XL_ODR_416Hz},
        {"833", LSM6DSL_XL_ODR_833Hz}
    };
    static constexpr Choice<lsm6dsl_fs_xl_t> XL_FS[] = {
        {"2", LSM6DSL_2g}, {"4", LSM6DSL_4g}, {"8", LSM6DSL_8g}, {"16", LSM6DSL_16g}
    };
    static constexpr Choice<lsm6dsl_fs_g_t> GY_FS[] = {
        {"125", LSM6DSL_125dps},
        {"250", LSM6DSL_250dps},
        {"500", LSM6DSL_500dps},
        {"1000", LSM6DSL_1000dps},
        {"2000", LSM6DSL_2000dps}
    };
    static constexpr Choice<lsm6dsl_lpf1_sel_g_t> GY_BW[] = {
        {"lp2_only", LSM6DSL_LP2_ONLY},
        {"lp1_strong", LSM6DSL_HP_DISABLE_LP1_STRONG},
        {"lp1_normal", LSM6DSL_HP_DISABLE_LP1_NORMAL},
        {"lp1_light", LSM6DSL_HP_DISABLE_LP1_LIGHT},
        {"lp1_aggressive", LSM6DSL_HP_DISABLE_LP1_AGGRESSIVE}
    };
    static constexpr Choice<lsm6dsl_input_composite_t> XL_BW[] = {
        {"low_latency_odr_div_50", LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_50},
        {"low_latency_odr_div_100", LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_100},
        {"low_latency_odr_div_9", LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9},
        {"low_latency_odr_div_400", LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_400},
        {"low_noise_odr_div_50", LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_50},
        {"low_noise_odr_div_100", LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100},
        {"low_noise_odr_div_9", LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_9},
        {"low_noise_odr_div_400", LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_400}
    };

    chip::lsm6dsl::Config c;
    auto odr = choose(n["odr_hz"], ODR, LSM6DSL_XL_ODR_416Hz, key + ".odr_hz");
    auto xfs = choose(n["accel_range_g"], XL_FS, LSM6DSL_4g, key + ".accel_range_g");
    auto gfs = choose(n["gyro_range_dps"], GY_FS, LSM6DSL_2000dps, key + ".gyro_range_dps");
    auto gbw = choose(n["gyro_bw"], GY_BW, LSM6DSL_HP_DISABLE_LP1_STRONG, key + ".gyro_bw");
    auto xbw = choose(n["accel_bw"], XL_BW, LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9, key + ".accel_bw");
    if (!odr) { return tl::make_unexpected(odr.error()); }
    if (!xfs) { return tl::make_unexpected(xfs.error()); }
    if (!gfs) { return tl::make_unexpected(gfs.error()); }
    if (!gbw) { return tl::make_unexpected(gbw.error()); }
    if (!xbw) { return tl::make_unexpected(xbw.error()); }
    c.accel_odr    = *odr;
    c.gyro_odr     = static_cast<lsm6dsl_odr_g_t>(*odr);
    c.accel_range  = *xfs;
    c.gyro_range   = *gfs;
    c.gyro_bw      = *gbw;
    c.accel_lp2_bw = *xbw;
    return c;
}

/* model = "lsm9ds1" (default) or "lsm6dsl"; the other keys are that chip's. */
inline tl::expected<decltype(Sensor::config), Error> chip_config(
    const toml::node_view<const toml::node> &n, const std::string &key
) {
    const std::string model = n["model"].value_or(std::string("lsm9ds1"));
    if (model == "lsm9ds1") {
        auto c = lsm9ds1_config(n, key);
        if (!c) { return tl::make_unexpected(c.error()); }
        return *c;
    }
    if (model == "lsm6dsl") {
        auto c = lsm6dsl_config(n, key);
        if (!c) { return tl::make_unexpected(c.error()); }
        return *c;
    }
    return tl::make_unexpected(key + ".model: lsm9ds1 or lsm6dsl");
}

inline tl::expected<void, Error> noise(
    const toml::node_view<const toml::node> &n, chip::Noise &out, const std::string &key
) {
    struct Field {
        const char *name;
        hand::Vec3 *dst;
    };
    const Field fields[] = {
        {"white_density", &out.white_density},
        {"turn_on_sigma", &out.turn_on_sigma},
        {"rate_random_walk", &out.rate_random_walk},
    };
    for (const auto &f : fields) {
        if (!n[f.name]) { continue; }
        auto v = vec3(n[f.name], key + "." + f.name);
        if (!v) { return tl::make_unexpected(v.error()); }
        *f.dst = *v;
    }
    /* markov = [{ sigma = [x, y, z], tau_s = [x, y, z] }, ...] */
    if (n["markov"] && !n["markov"].is_array()) {
        return tl::make_unexpected(key + ".markov: expected an array");
    }
    if (const auto *terms = n["markov"].as_array()) {
        for (std::size_t k = 0; k < terms->size(); ++k) {
            const auto term     = toml::node_view<const toml::node>(terms->get(k));
            const std::string t = key + ".markov[" + std::to_string(k) + "]";
            auto sigma          = vec3(term["sigma"], t + ".sigma");
            if (!sigma) { return tl::make_unexpected(sigma.error()); }
            auto tau = vec3(term["tau_s"], t + ".tau_s");
            if (!tau) { return tl::make_unexpected(tau.error()); }
            if ((tau->array() <= 0.0).any()) {
                return tl::make_unexpected(t + ".tau_s: must be positive");
            }
            out.markov.push_back({.sigma = *sigma, .tau_s = *tau});
        }
    }
    return {};
}

inline tl::expected<hand::Mat3, Error> mat3(
    const toml::node_view<const toml::node> &n, const std::string &key
) {
    const auto *rows = n.as_array();
    if (rows == nullptr || rows->size() != 3) {
        return tl::make_unexpected(key + ": expected 3 rows");
    }
    hand::Mat3 m;
    for (std::size_t r = 0; r < 3; ++r) {
        auto row = vec3(n[r], key);
        if (!row) { return tl::make_unexpected(row.error()); }
        m.row(static_cast<Eigen::Index>(r)) = row->transpose();
    }
    return m;
}

/* { kind = "butterworth" | "one_pole" | "moving_average", order = n }; absent bypasses the stage.
 */
inline tl::expected<void, Error> filter(
    const toml::node_view<const toml::node> &n, chip::FilterShape &out, const std::string &key
) {
    if (!n) { return {}; }
    static constexpr Choice<chip::FilterKind> KINDS[] = {
        {"butterworth", chip::FilterKind::BUTTERWORTH},
        {"one_pole", chip::FilterKind::ONE_POLE},
        {"moving_average", chip::FilterKind::MOVING_AVERAGE}
    };
    auto kind = choose(n["kind"], KINDS, chip::FilterKind::BUTTERWORTH, key + ".kind");
    if (!kind) { return tl::make_unexpected(kind.error()); }
    out.kind  = *kind;
    out.order = n["order"].value_or(0);
    return {};
}

} /* namespace detail */

/* A chip-parameter file; keys left out keep the ideal defaults of chip::Params. */
namespace detail {

inline tl::expected<toml::table, Error> parse(const std::filesystem::path &path, const std::string &label) {
    try {
        return toml::parse_file(path.string());
    } catch (const toml::parse_error &e) {
        return tl::make_unexpected(label + ": " + std::string(e.description()));
    }
}

} /* namespace detail */

inline tl::expected<chip::Params, Error> load_params(const std::filesystem::path &path) {
    auto t = detail::parse(path, path.string());
    if (!t) { return tl::make_unexpected(t.error()); }
    const auto n = toml::node_view<const toml::node>(*t);
    chip::Params p;
    p.odr_scale  = n["odr_scale"].value_or(p.odr_scale);
    p.oversample = n["oversample"].value_or(p.oversample);
    for (auto [key, dst] :
         {std::pair{"accel_aa", &p.accel_aa},
          std::pair{"accel_lp", &p.accel_lp},
          std::pair{"gyro_lpf1", &p.gyro_lpf1},
          std::pair{"gyro_lpf2", &p.gyro_lpf2}}) {
        if (auto r = detail::filter(n[key], *dst, key); !r) {
            return tl::make_unexpected(r.error());
        }
    }
    p.temperature_c = n["temperature_c"].value_or(p.temperature_c);
    if (auto r = detail::noise(n["accel"], p.accel, "accel"); !r) {
        return tl::make_unexpected(r.error());
    }
    if (auto r = detail::noise(n["gyro"], p.gyro, "gyro"); !r) {
        return tl::make_unexpected(r.error());
    }
    /* [gyro_lpf2_by_bw]: "<BW_G 0-3>" = { kind, order }. */
    if (const auto *table = n["gyro_lpf2_by_bw"].as_table()) {
        for (const auto &[bw, value] : *table) {
            chip::FilterShape shape;
            const std::string key = "gyro_lpf2_by_bw." + std::string(bw.str());
            if (auto r = detail::filter(toml::node_view<const toml::node>(value), shape, key); !r) {
                return tl::make_unexpected(r.error());
            }
            p.gyro_lpf2_by_bw[std::atoi(std::string(bw.str()).c_str())] = shape;
        }
    }
    /* [gyro_lpf1_by_odr]: "<nominal ODR>" = { kind, order, cutoff_scale } for all axes, or an
     * array of three, x y z. */
    if (const auto *table = n["gyro_lpf1_by_odr"].as_table()) {
        for (const auto &[odr, value] : *table) {
            const std::string key = "gyro_lpf1_by_odr." + std::string(odr.str());
            const auto *axes      = value.as_array();
            if (axes != nullptr && axes->size() != 3) {
                return tl::make_unexpected(key + ": expected one table or three");
            }
            std::array<chip::Params::ScaledShape, 3> fits;
            for (std::size_t i = 0; i < 3; ++i) {
                const auto v = axes != nullptr ? toml::node_view<const toml::node>(axes->get(i))
                                               : toml::node_view<const toml::node>(value);
                if (auto r = detail::filter(v, fits[i].shape, key); !r) {
                    return tl::make_unexpected(r.error());
                }
                fits[i].cutoff_scale = v["cutoff_scale"].value_or(1.0);
            }
            p.gyro_lpf1_by_odr[std::atoi(std::string(odr.str()).c_str())] = fits;
        }
    }
    /* [accel.by_fs] / [gyro.by_fs]: "<full scale>" = [x, y, z] densities; [gyro.offset_by_fs]
     * the same for the zero-rate level, [*.drift_scale_by_fs] for the drift level. */
    struct ByFs {
        const char *kind;
        const char *table;
        std::map<double, hand::Vec3> *dst;
    };
    for (const auto &[kind, name, dst] :
         {ByFs{"accel", "by_fs", &p.accel_density_by_fs},
          ByFs{"gyro", "by_fs", &p.gyro_density_by_fs},
          ByFs{"gyro", "offset_by_fs", &p.gyro_offset_by_fs},
          ByFs{"accel", "drift_scale_by_fs", &p.accel_drift_scale_by_fs},
          ByFs{"gyro", "drift_scale_by_fs", &p.gyro_drift_scale_by_fs}}) {
        const auto *table = n[kind][name].as_table();
        if (table == nullptr) { continue; }
        for (const auto &[fs, value] : *table) {
            const std::string key = std::string(kind) + "." + name + "." + std::string(fs.str());
            auto v                = detail::vec3(toml::node_view<const toml::node>(value), key);
            if (!v) { return tl::make_unexpected(v.error()); }
            (*dst)[std::strtod(std::string(fs.str()).c_str(), nullptr)] = *v;
        }
    }
    struct VecField {
        const char *name;
        hand::Vec3 *dst;
    };
    for (const auto &f :
         {VecField{"accel_offset", &p.accel_offset},
          VecField{"gyro_offset", &p.gyro_offset},
          VecField{"gyro_negative_gain", &p.gyro_negative_gain}}) {
        if (!n[f.name]) { continue; }
        auto v = detail::vec3(n[f.name], f.name);
        if (!v) { return tl::make_unexpected(v.error()); }
        *f.dst = *v;
    }
    struct MatField {
        const char *name;
        hand::Mat3 *dst;
    };
    for (const auto &f :
         {MatField{"accel_gain", &p.accel_gain},
          MatField{"gyro_gain", &p.gyro_gain},
          MatField{"gyro_g_sensitivity", &p.gyro_g_sensitivity}}) {
        if (!n[f.name]) { continue; }
        auto m = detail::mat3(n[f.name], f.name);
        if (!m) { return tl::make_unexpected(m.error()); }
        *f.dst = *m;
    }
    return p;
}

namespace detail {

inline tl::expected<hand::Geometry, Error> geometry(const toml::node_view<const toml::node> &g) {
    for (const char *key : {"palm_sensor", "mcp", "finger_sensor"}) {
        if (!g[key]) {
            return tl::make_unexpected(std::string("geometry.") + key + " is required");
        }
    }
    for (const char *key : {"proximal_m", "middle_m", "dip_per_pip"}) {
        if (!g[key].value<double>()) {
            return tl::make_unexpected(std::string("geometry.") + key + " is required");
        }
    }
    auto palm_sensor   = vec3(g["palm_sensor"], "geometry.palm_sensor");
    auto mcp           = vec3(g["mcp"], "geometry.mcp");
    auto finger_sensor = vec3(g["finger_sensor"], "geometry.finger_sensor");
    auto palm_mount    = rotation(g["palm_mount"], "geometry.palm_mount");
    auto finger_mount  = rotation(g["finger_mount"], "geometry.finger_mount");
    for (const auto *e : {&palm_sensor, &mcp, &finger_sensor}) {
        if (!*e) { return tl::make_unexpected(e->error()); }
    }
    if (!palm_mount) { return tl::make_unexpected(palm_mount.error()); }
    if (!finger_mount) { return tl::make_unexpected(finger_mount.error()); }
    hand::Geometry geometry{
        .palm_sensor   = *palm_sensor,
        .mcp           = *mcp,
        .proximal_m    = *g["proximal_m"].value<double>(),
        .middle_m      = *g["middle_m"].value<double>(),
        .finger_sensor = *finger_sensor,
        .dip_per_pip   = *g["dip_per_pip"].value<double>(),
        .palm_mount    = palm_mount->toRotationMatrix(),
        .finger_mount  = finger_mount->toRotationMatrix(),
    };
    geometry.twist_at_90_rad = rad(g["twist_at_90_deg"].value_or(0.0));
    const auto segment       = g["finger_segment"].value_or(std::string("distal"));
    if (segment != "distal" && segment != "middle") {
        return tl::make_unexpected("geometry.finger_segment: \"distal\" or \"middle\"");
    }
    geometry.finger_segment =
        segment == "middle" ? hand::FingerSegment::Middle : hand::FingerSegment::Distal;
    if (g["contact_pivot"]) {
        auto pivot = vec3(g["contact_pivot"], "geometry.contact_pivot");
        if (!pivot) { return tl::make_unexpected(pivot.error()); }
        geometry.contact_pivot = *pivot;
    }
    if (g["finger_contact_pivot"]) {
        auto pivot = vec3(g["finger_contact_pivot"], "geometry.finger_contact_pivot");
        if (!pivot) { return tl::make_unexpected(pivot.error()); }
        geometry.finger_contact_pivot = *pivot;
    }
    return geometry;
}

inline tl::expected<Sensor, Error> sensor(
    const toml::node_view<const toml::node> &n,
    const std::string &k,
    const std::filesystem::path &dir,
    const Realism &realism
) {
    Sensor sensor;
    auto config = chip_config(n["chip"], k + ".chip");
    if (!config) { return tl::make_unexpected(config.error()); }
    sensor.config = *config;
    if (auto file = n["params"].value<std::string>()) {
        auto params = load_params(dir / *file);
        if (!params) { return tl::make_unexpected(params.error()); }
        sensor.params = *params;
    }
    auto &p = sensor.params;
    /* A unit's own measured rate, when it differs from the fitted chip's. */
    if (auto scale = n["odr_scale"].value<double>()) { p.odr_scale = *scale; }
    if (auto file = n["input"].value<std::string>()) {
        auto input = Replay::load(dir / *file);
        if (!input) { return tl::make_unexpected(k + ".input: " + input.error()); }
        sensor.input = std::move(*input);
    }
    if (!realism.chip_transfer) {
        p.accel_aa = p.accel_lp = p.gyro_lpf1 = p.gyro_lpf2 = chip::FilterShape{};
        p.gyro_lpf2_by_bw.clear();
        p.gyro_lpf1_by_odr.clear();
    }
    if (!realism.noise) {
        chip::Params ideal;
        ideal.odr_scale        = p.odr_scale;
        ideal.oversample       = p.oversample;
        ideal.accel_aa         = p.accel_aa;
        ideal.accel_lp         = p.accel_lp;
        ideal.gyro_lpf1        = p.gyro_lpf1;
        ideal.gyro_lpf2        = p.gyro_lpf2;
        ideal.gyro_lpf2_by_bw  = p.gyro_lpf2_by_bw;
        ideal.gyro_lpf1_by_odr = p.gyro_lpf1_by_odr;
        ideal.temperature_c    = p.temperature_c;
        p                      = ideal;
    }
    return sensor;
}

/* Pose fields a move leaves out keep their previous value. */
inline tl::expected<hand::Pose, Error> pose(
    const toml::node_view<const toml::node> &m, hand::Pose base, const std::string &key
) {
    if (m["palm"]) {
        auto q = rotation(m["palm"], key + ".palm");
        if (!q) { return tl::make_unexpected(q.error()); }
        base.palm = *q;
    }
    if (m["wrist"]) {
        auto w = vec3(m["wrist"], key + ".wrist");
        if (!w) { return tl::make_unexpected(w.error()); }
        base.wrist = *w;
    }
    const auto j = m["joints_deg"];
    if (auto v = j["wrist_flex"].value<double>()) { base.joints.wrist_flex = rad(*v); }
    if (auto v = j["wrist_dev"].value<double>()) { base.joints.wrist_dev = rad(*v); }
    if (auto v = j["mcp_abd"].value<double>()) { base.joints.mcp_abd = rad(*v); }
    if (auto v = j["mcp_flex"].value<double>()) { base.joints.mcp_flex = rad(*v); }
    if (auto v = j["pip_flex"].value<double>()) { base.joints.pip_flex = rad(*v); }
    if (auto v = m["support"].value<double>()) { base.support = *v; }
    return base;
}

inline tl::expected<hand::Trajectory, Error> trajectory(const toml::node_view<const toml::node> &n) {
    auto start = pose(n["start"], hand::Pose{}, "start");
    if (!start) { return tl::make_unexpected(start.error()); }
    hand::Trajectory trajectory(*start);
    hand::Pose at  = *start;
    const auto *ms = n["moves"].as_array();
    if (ms == nullptr || ms->empty()) {
        return tl::make_unexpected(std::string("moves: at least one [[moves]] entry"));
    }
    for (std::size_t i = 0; i < ms->size(); ++i) {
        const auto m        = n["moves"][i];
        const std::string k = "moves[" + std::to_string(i) + "]";
        if (auto hold = m["hold_s"].value<double>()) {
            trajectory.hold(*hold);
            continue;
        }
        auto duration = m["duration_s"].value<double>();
        if (!duration) { return tl::make_unexpected(k + ": needs hold_s or duration_s"); }
        auto to = pose(m, at, k);
        if (!to) { return tl::make_unexpected(to.error()); }
        const std::string profile = m["profile"].value_or(std::string("min_jerk"));
        if (profile != "min_jerk" && profile != "cosine" && profile != "beta") {
            return tl::make_unexpected(k + ".profile: min_jerk, cosine or beta");
        }
        hand::Shape shape;
        if (profile == "beta") {
            const auto *ab = m["shape"].as_array();
            const auto a   = ab != nullptr && ab->size() == 2 ? (*ab)[0].value<double>() : std::nullopt;
            const auto b   = ab != nullptr && ab->size() == 2 ? (*ab)[1].value<double>() : std::nullopt;
            if (!a || !b || *a < 1.0 || *b < 1.0) { return tl::make_unexpected(k + ".shape: [a, b], each >= 1"); }
            shape = {*a, *b};
        }
        trajectory.move(
            *duration, *to,
            profile == "cosine" ? hand::Profile::COSINE : profile == "beta" ? hand::Profile::BETA : hand::Profile::MIN_JERK,
            shape
        );
        at = *to;
    }
    return trajectory;
}

inline constexpr std::pair<const char *, hand::Dof> DOFS[] = {
        {"wrist_flex", hand::WRIST_FLEX},
        {"wrist_dev", hand::WRIST_DEV},
        {"mcp_abd", hand::MCP_ABD},
        {"mcp_flex", hand::MCP_FLEX},
        {"pip_flex", hand::PIP_FLEX},
        {"sway_rx", hand::SWAY_RX},
        {"sway_ry", hand::SWAY_RY},
        {"sway_rz", hand::SWAY_RZ},
        {"sway_x", hand::SWAY_X},
        {"sway_y", hand::SWAY_Y},
        {"sway_z", hand::SWAY_Z}
};

/* tremor_file first, then [tremor] keys override it field by field. */
inline tl::expected<void, Error> tremor(
    const toml::node_view<const toml::node> &n, const std::filesystem::path &dir, Scenario &s
) {
    toml::table tremor_file;
    if (auto file = n["tremor_file"].value<std::string>()) {
        auto parsed = parse(dir / *file, *file);
        if (!parsed) { return tl::make_unexpected(parsed.error()); }
        tremor_file = std::move(*parsed);
    }
    const auto from_file = toml::node_view<const toml::node>(tremor_file);
    for (const auto &[key, dof] : DOFS) {
        hand::Oscillator &o = s.tremor[dof];
        for (const auto &src : {from_file[key], n["tremor"][key]}) {
            if (!src) { continue; }
            o.inertia        = src["inertia"].value_or(o.inertia);
            o.stiffness      = src["stiffness"].value_or(o.stiffness);
            o.damping        = src["damping"].value_or(o.damping);
            o.torque_density = src["torque_density"].value_or(o.torque_density);
            o.muscle_tau_s   = src["muscle_tau_s"].value_or(o.muscle_tau_s);
            o.muscle_order   = src["muscle_order"].value_or(o.muscle_order);
        }
    }
    return {};
}

/* [[drives]]: frequency_hz and bandwidth_hz, optional harmonics (amplitudes, default [1.0]) and
 * gate ("support", default, "none" or "speed"; the older supported = false is "none"), and a gain
 * table by DOF name. */
inline tl::expected<void, Error> drives(const toml::node_view<const toml::node> &n, Scenario &s) {
    const auto *list = n["drives"].as_array();
    if (list == nullptr) { return {}; }
    for (std::size_t i = 0; i < list->size(); ++i) {
        const auto d        = n["drives"][i];
        const std::string k = "drives[" + std::to_string(i) + "]";
        hand::SharedDrive out;
        const auto f = d["frequency_hz"].value<double>();
        const auto w = d["bandwidth_hz"].value<double>();
        if (!f || !w || *f <= 0.0 || *w <= 0.0) {
            return tl::make_unexpected(k + ": needs frequency_hz and bandwidth_hz > 0");
        }
        out.frequency_hz = *f;
        out.bandwidth_hz = *w;
        out.gate         = d["supported"].value_or(true) ? hand::SharedDrive::Gate::Support : hand::SharedDrive::Gate::None;
        if (auto g = d["gate"].value<std::string>()) {
            if (*g == "support") {
                out.gate = hand::SharedDrive::Gate::Support;
            } else if (*g == "none") {
                out.gate = hand::SharedDrive::Gate::None;
            } else if (*g == "speed") {
                out.gate = hand::SharedDrive::Gate::Speed;
            } else {
                return tl::make_unexpected(k + ".gate: support, none or speed");
            }
        }
        if (const auto *h = d["harmonics"].as_array()) {
            out.harmonics.clear();
            for (std::size_t m = 0; m < h->size(); ++m) {
                const auto a = (*h)[m].value<double>();
                if (!a) { return tl::make_unexpected(k + ".harmonics: expected numbers"); }
                out.harmonics.push_back(*a);
            }
        }
        const auto *gain = d["gain"].as_table();
        if (gain == nullptr) { return tl::make_unexpected(k + ": needs a gain table"); }
        for (const auto &[key, value] : *gain) {
            const auto *dof = std::find_if(std::begin(DOFS), std::end(DOFS), [&](const auto &p) {
                return key.str() == p.first;
            });
            const auto g = value.value<double>();
            if (dof == std::end(DOFS) || !g) {
                return tl::make_unexpected(k + ".gain." + std::string(key.str()) + ": unknown DOF or not a number");
            }
            out.gain[dof->second] = *g;
        }
        s.drives.push_back(std::move(out));
    }
    return {};
}

/* Added mass at a sensor, moved by its optional offset (palm or distal phalanx frame), adds
 * m r^2 to every joint proximal to it, r from the joint axis. */
inline tl::expected<void, Error> masses(const toml::node_view<const toml::node> &n, Scenario &s) {
    const auto *masses = n["masses"].as_array();
    if (masses == nullptr) { return {}; }
    for (std::size_t i = 0; i < masses->size(); ++i) {
        const auto m        = n["masses"][i];
        const std::string k = "masses[" + std::to_string(i) + "]";
        const auto kg       = m["kg"].value<double>();
        const auto at_      = m["at"].value<std::string>();
        if (!kg || !at_ || (*at_ != "palm" && *at_ != "finger")) {
            return tl::make_unexpected(k + ": needs kg and at = \"palm\" or \"finger\"");
        }
        hand::Vec3 offset = hand::Vec3::Zero();
        if (m["offset"]) {
            auto o = vec3(m["offset"], k + ".offset");
            if (!o) { return tl::make_unexpected(o.error()); }
            offset = *o;
        }
        const hand::Geometry &g = s.geometry;
        const double segment_y = g.proximal_m
                               + (g.finger_segment == hand::FingerSegment::Distal ? g.middle_m : 0.0);
        const hand::Vec3 finger = g.mcp + hand::Vec3(0.0, segment_y, 0.0) + g.finger_sensor;
        const hand::Vec3 p = (*at_ == "palm" ? g.palm_sensor : finger) + offset;
        auto arm2          = [](const hand::Vec3 &r, const hand::Vec3 &axis)
        { return (r - r.dot(axis.normalized()) * axis.normalized()).squaredNorm(); };
        s.tremor[hand::WRIST_FLEX].inertia += *kg * arm2(p, g.flex_axis);
        s.tremor[hand::WRIST_DEV].inertia  += *kg * arm2(p, g.abd_axis);
        s.tremor[hand::SWAY_RX].inertia    += *kg * arm2(p, hand::Vec3::UnitX());
        s.tremor[hand::SWAY_RY].inertia    += *kg * arm2(p, hand::Vec3::UnitY());
        s.tremor[hand::SWAY_RZ].inertia    += *kg * arm2(p, hand::Vec3::UnitZ());
        if (*at_ == "finger") {
            s.tremor[hand::MCP_ABD].inertia  += *kg * arm2(p - g.mcp, g.abd_axis);
            s.tremor[hand::MCP_FLEX].inertia += *kg * arm2(p - g.mcp, g.flex_axis);
            s.tremor[hand::PIP_FLEX].inertia +=
                *kg * arm2(p - g.mcp - hand::Vec3(0.0, g.proximal_m, 0.0), g.flex_axis);
        }
    }
    return {};
}

} /* namespace detail */

inline tl::expected<Scenario, Error> load_scenario(const std::filesystem::path &path) {
    auto t = detail::parse(path, path.string());
    if (!t) { return tl::make_unexpected(t.error()); }
    const auto n   = toml::node_view<const toml::node>(*t);
    const auto dir = path.parent_path();
    Scenario s;
    s.name                  = n["name"].value_or(path.stem().string());
    s.seed                  = static_cast<std::uint64_t>(n["seed"].value_or(std::int64_t{1}));
    s.realism.chip_transfer = n["realism"]["chip_transfer"].value_or(true);
    s.realism.noise         = n["realism"]["noise"].value_or(true);
    s.realism.tremor        = n["realism"]["tremor"].value_or(true);

    auto geometry = detail::geometry(n["geometry"]);
    if (!geometry) { return tl::make_unexpected(geometry.error()); }
    s.geometry = *geometry;
    for (auto [key, dst] : {std::pair{"palm", &s.palm}, std::pair{"finger", &s.finger}}) {
        auto sensor = detail::sensor(n[key], key, dir, s.realism);
        if (!sensor) { return tl::make_unexpected(sensor.error()); }
        *dst = std::move(*sensor);
    }
    auto trajectory = detail::trajectory(n);
    if (!trajectory) { return tl::make_unexpected(trajectory.error()); }
    s.trajectory = std::move(*trajectory);
    if (auto r = detail::tremor(n, dir, s); !r) { return tl::make_unexpected(r.error()); }
    if (auto r = detail::masses(n, s); !r) { return tl::make_unexpected(r.error()); }
    if (auto r = detail::drives(n, s); !r) { return tl::make_unexpected(r.error()); }
    for (auto [key, dst] : {std::pair{"palm", &s.palm_jolt}, std::pair{"finger", &s.finger_jolt}}) {
        const auto j = n["jolt"][key];
        if (!j) { continue; }
        dst->frequency_hz = j["frequency_hz"].value_or(0.0);
        dst->bandwidth_hz = j["bandwidth_hz"].value_or(0.0);
        dst->level_dps    = j["level_dps"].value_or(0.0);
    }
    if (!s.realism.tremor) {
        s.tremor = {};
        s.drives.clear();
        s.palm_jolt   = {};
        s.finger_jolt = {};
    }
    return s;
}

} /* namespace sim */

#endif /* SIM_SCENARIO_HPP */
