/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

#include "chip/filter.hpp"
#include "chip/lsm6dsl.hpp"
#include "chip/lsm9ds1.hpp"

namespace {

namespace m = chip::lsm9ds1;
using m::Vec3;

int g_failures = 0;

void check(bool ok, const char *what, double value) {
    std::printf("[%s] %s (%.6g)\n", ok ? "PASS" : "FAIL", what, value);
    if (!ok) { ++g_failures; }
}

m::Model::Config default_config() {
    m::Model::Config c;
    c.imu_odr       = LSM9DS1_IMU_476Hz;
    c.accel_range   = LSM9DS1_4g;
    c.gyro_range    = LSM9DS1_2000dps;
    c.gyro_out_path = LSM9DS1_LPF1_OUT;
    return c;
}

std::vector<m::Output> run(chip::Model &model, double seconds, Vec3 f, Vec3 w) {
    std::vector<m::Output> out;
    model.advance(
        seconds,
        [&](double) { return std::pair{f, w}; },
        [&](const m::Output &o) { out.push_back(o); }
    );
    return out;
}

void filters() {
    for (const auto kind : {chip::FilterKind::BUTTERWORTH, chip::FilterKind::ONE_POLE}) {
        double worst = 0.0;
        for (int order = 1; order <= 5; ++order) {
            const chip::LowPass f({kind, order}, 53.0, 461.5);
            worst = std::max(worst, std::abs(std::norm(f.response(53.0, 461.5)) - 0.5));
            worst = std::max(worst, std::abs(std::abs(f.response(0.0, 461.5)) - 1.0));
        }
        check(
            worst < 1e-9,
            kind == chip::FilterKind::BUTTERWORTH
                ? "Butterworth orders 1-5: -3 dB at the cutoff, unity at DC"
                : "one-pole cascades 1-5: -3 dB at the cutoff, unity at DC",
            worst
        );
    }
    const chip::LowPass b({chip::FilterKind::BUTTERWORTH, 1}, 51.3, 461.5);
    const chip::LowPass o({chip::FilterKind::ONE_POLE, 1}, 51.3, 461.5);
    check(
        std::abs(b.response(230.75, 461.5)) < 1e-6 && std::abs(o.response(230.75, 461.5)) > 0.3,
        "Butterworth has a zero at Nyquist, one-pole does not",
        std::abs(o.response(230.75, 461.5))
    );

    chip::LowPass ma({chip::FilterKind::MOVING_AVERAGE, 9}, 0.0, 461.5);
    ma.reset(2.0);
    double y = 0.0;
    for (int i = 0; i < 10; ++i) {
        y = ma.step(i == 0 ? 11.0 : 2.0);
    }
    check(
        std::abs(ma.response(461.5 / 9, 461.5)) < 1e-12
            && std::abs(std::abs(ma.response(0.0, 461.5)) - 1) < 1e-12 && std::abs(y - 2.0) < 1e-12,
        "moving average: zero at fs / n, unity at DC, an impulse leaves after n samples",
        y
    );
}

void config_mapping() {
    m::Params p;
    p.odr_scale   = 461.534 / 476.0;
    auto c        = default_config();
    auto model    = m::Model::make(c, p, 1).value();
    const auto &k = model.cutoffs();
    check(
        std::abs(model.odr_hz() - 461.534) < 1e-9,
        "ODR is the nominal one times odr_scale",
        model.odr_hz()
    );
    check(
        k.accel_aa == 211.0 && std::abs(k.accel_lp - 461.534 / 9) < 1e-9
            && std::abs(k.gyro_lpf1 - 100.0 * p.odr_scale) < 1e-9 && k.gyro_lpf2 == 0.0,
        "476 Hz default: AA 211, LP ODR/9, LPF1 100 scaled, no LPF2",
        k.accel_lp
    );

    c.gyro_out_path = LSM9DS1_LPF1_LPF2_OUT;
    c.gyro_bw       = LSM9DS1_LP_STRONG;
    c.imu_odr       = LSM9DS1_IMU_238Hz;
    c.accel_bw      = LSM9DS1_LP_DISABLE;
    model           = m::Model::make(c, m::Params{}, 1).value();
    check(
        model.cutoffs().accel_aa == 105.0 && model.cutoffs().accel_lp == 0.0
            && model.cutoffs().gyro_lpf1 == 76.0 && model.cutoffs().gyro_lpf2 == 14.0,
        "238 Hz, LPF2 strong, accel LP off: AA 105, LPF1 76, LPF2 14",
        model.cutoffs().gyro_lpf2
    );

    const chip::FilterShape one_pole{chip::FilterKind::ONE_POLE, 3};
    p.gyro_lpf1_by_odr[238] = {{{one_pole, 0.7}, {one_pole, 0.8}, {one_pole, 0.9}}};
    model                   = m::Model::make(c, p, 1).value();
    check(
        std::abs(model.gyro_lpf1_hz()[0] - 76.0 * p.odr_scale * 0.7) < 1e-9
            && std::abs(model.gyro_lpf1_hz()[2] - 76.0 * p.odr_scale * 0.9) < 1e-9
            && model.cutoffs().gyro_lpf1 == 76.0 * p.odr_scale,
        "fitted LPF1 scale applies per axis on top of odr_scale at its ODR only",
        model.gyro_lpf1_hz()[0]
    );

    c.imu_odr = LSM9DS1_IMU_59Hz5;
    check(!m::Model::make(c, m::Params{}, 1).has_value(), "59.5 Hz is rejected", 0);
    c               = default_config();
    c.gyro_out_path = LSM9DS1_LPF1_HPF_OUT;
    check(!m::Model::make(c, m::Params{}, 1).has_value(), "HPF output path is rejected", 0);
}

void quantize_and_clip() {
    auto model    = m::Model::make(default_config(), m::Params{}, 1).value();
    auto out      = run(model, 0.1, Vec3(1.0, -0.5, 0.25), Vec3(100.0, 0.0, -3.0));
    const auto &r = out.back().raw;
    check(
        r.accel[0] == 8197 && r.accel[1] == -4098 && r.accel[2] == 2049 && r.gyro[0] == 1429
            && r.gyro[2] == -43,
        "codes are the value over the LSB, rounded",
        r.accel[0]
    );

    model         = m::Model::make(default_config(), m::Params{}, 1).value();
    out           = run(model, 0.1, Vec3(5.0, -5.0, 5.0), Vec3(2500.0, -2500.0, -2500.0));
    const auto &c = out.back().raw;
    /* z: +5 g is -5 g in the chip frame, clips to -32768, and the driver's negation saturates. */
    check(
        c.accel[0] == 32767 && c.accel[1] == -32768 && c.accel[2] == 32767 && c.gyro[0] == 32767
            && c.gyro[1] == -32768 && c.gyro[2] == -32767,
        "clips at full scale in the chip frame",
        c.accel[2]
    );
}

void negative_gain() {
    m::Params p;
    p.gyro_negative_gain = Vec3(1.0, 0.96, 1.0);
    auto model           = m::Model::make(default_config(), p, 1).value();
    const double lsb     = model.raw_scale().gyro_dps_per_lsb;
    const auto pos       = run(model, 0.1, Vec3::UnitZ(), Vec3(0.0, 500.0, 0.0)).back().raw.gyro[1] * lsb;
    model                = m::Model::make(default_config(), p, 1).value();
    const auto neg       = run(model, 0.1, Vec3::UnitZ(), Vec3(0.0, -500.0, 0.0)).back().raw.gyro[1] * lsb;
    check(
        std::abs(pos - 500.0) < 0.1 && std::abs(neg + 480.0) < 0.1,
        "negative rates take the negative gain, positive ones do not",
        neg
    );
}

void decimation_and_filter() {
    m::Params p;
    p.odr_scale = 461.534 / 476.0;
    p.accel_lp  = {chip::FilterKind::BUTTERWORTH, 2};
    auto model  = m::Model::make(default_config(), p, 1).value();
    std::vector<double> x;
    const double fc = model.cutoffs().accel_lp;
    model.advance(
        30.0,
        [&](double t)
        { return std::pair{Vec3(std::sin(2 * M_PI * fc * t), 0.0, 0.0), Vec3::Zero()}; },
        [&](const m::Output &o) { x.push_back(o.raw.accel[0] * model.raw_scale().accel_g_per_lsb); }
    );
    check(
        std::abs(static_cast<double>(x.size()) - 30.0 * model.odr_hz()) <= 1,
        "30 s gives 30 x ODR samples",
        x.size()
    );
    double power = 0.0;
    for (std::size_t i = x.size() / 2; i < x.size(); ++i) {
        power += x[i] * x[i];
    }
    const double gain = std::sqrt(power / static_cast<double>(x.size() - x.size() / 2) * 2.0);
    check(
        std::abs(gain - std::sqrt(0.5)) < 0.01,
        "a sine at the accel LP cutoff comes out at -3 dB",
        gain
    );
}

double std_of(const std::vector<double> &x) {
    double mean = 0.0, var = 0.0;
    for (double v : x) {
        mean += v;
    }
    mean /= static_cast<double>(x.size());
    for (double v : x) {
        var += (v - mean) * (v - mean);
    }
    return std::sqrt(var / static_cast<double>(x.size() - 1));
}

void noise() {
    m::Params p;
    p.gyro.white_density  = Vec3::Constant(0.015);
    p.accel.white_density = Vec3::Constant(100e-6);
    auto model            = m::Model::make(default_config(), p, 3).value();
    std::vector<double> g, a;
    model.advance(
        60.0,
        [&](double) { return std::pair{Vec3::UnitZ().eval(), Vec3::Zero().eval()}; },
        [&](const m::Output &o)
        {
            g.push_back(o.raw.gyro[0] * model.raw_scale().gyro_dps_per_lsb);
            a.push_back(o.raw.accel[0] * model.raw_scale().accel_g_per_lsb);
        }
    );
    const double expected_g = 0.015 * std::sqrt(model.internal_hz() / 2);
    check(
        std::abs(std_of(g) / expected_g - 1) < 0.03,
        "unfiltered gyro std = density x sqrt(internal Nyquist)",
        std_of(g) / expected_g
    );

    p.gyro_lpf1 = {chip::FilterKind::BUTTERWORTH, 2};
    model       = m::Model::make(default_config(), p, 3).value();
    g.clear();
    model.advance(
        60.0,
        [&](double) { return std::pair{Vec3::UnitZ().eval(), Vec3::Zero().eval()}; },
        [&](const m::Output &o) { g.push_back(o.raw.gyro[0] * model.raw_scale().gyro_dps_per_lsb); }
    );
    /* Equivalent noise bandwidth of an order-n Butterworth: fc (pi / 2n) / sin(pi / 2n). */
    const double enbw     = model.cutoffs().gyro_lpf1 * (M_PI / 4) / std::sin(M_PI / 4);
    const double expected = 0.015 * std::sqrt(enbw);
    check(
        std::abs(std_of(g) / expected - 1) < 0.05,
        "LPF1 gyro std = density x sqrt(noise bandwidth)",
        std_of(g) / expected
    );
}

void density_by_range() {
    m::Params p;
    p.gyro.white_density      = Vec3::Constant(0.015);
    p.gyro_density_by_fs[245] = Vec3::Constant(0.009);
    auto c                    = default_config();
    c.gyro_range              = LSM9DS1_245dps;
    auto low                  = m::Model::make(c, p, 4).value();
    auto high                 = m::Model::make(default_config(), p, 4).value();
    auto gyro_std             = [](chip::Model &model)
    {
        std::vector<double> g;
        model.advance(
            20.0,
            [&](double) { return std::pair{Vec3::UnitZ().eval(), Vec3::Zero().eval()}; },
            [&](const m::Output &o)
            { g.push_back(o.raw.gyro[0] * model.raw_scale().gyro_dps_per_lsb); }
        );
        return std_of(g);
    };
    const double ratio = gyro_std(low) / gyro_std(high);
    check(std::abs(ratio - 0.6) < 0.03, "a range's own density overrides the default", ratio);

    m::Params q;
    q.gyro_offset            = Vec3(1.4, 1.0, -0.4);
    q.gyro_offset_by_fs[245] = Vec3(1.0, 1.0, -0.4);
    auto zero                = m::Model::make(c, q, 4).value();
    std::vector<double> x;
    zero.advance(
        0.5,
        [&](double) { return std::pair{Vec3::UnitZ().eval(), Vec3::Zero().eval()}; },
        [&](const m::Output &o) { x.push_back(o.raw.gyro[0] * zero.raw_scale().gyro_dps_per_lsb); }
    );
    check(
        std::abs(x.back() - 1.0) < 0.01,
        "a range's own zero-rate level overrides the default",
        x.back()
    );
}

/* Overlapping Allan variance of a rate series at m samples per cluster. */
double allan_var(const std::vector<double> &x, std::size_t m, double fs) {
    std::vector<double> theta(x.size() + 1, 0.0);
    for (std::size_t i = 0; i < x.size(); ++i) {
        theta[i + 1] = theta[i] + (x[i] / fs);
    }
    double sum      = 0.0;
    const double t  = static_cast<double>(m) / fs;
    std::size_t cnt = 0;
    for (std::size_t i = 0; i + (2 * m) < theta.size(); ++i, ++cnt) {
        const double d  = theta[i + (2 * m)] - (2 * theta[i + m]) + theta[i];
        sum            += d * d;
    }
    return sum / static_cast<double>(cnt) / (2 * t * t);
}

double markov_avar(double tau, double sigma, double tc) {
    const double r = tau / tc;
    return sigma * sigma / (r * r) * ((2 * r) - 3 + (4 * std::exp(-r)) - std::exp(-2 * r));
}

void drift() {
    m::Params p;
    p.gyro.markov = {
        {.sigma = Vec3::Constant(0.05), .tau_s = Vec3::Constant(2.0)},
        {.sigma = Vec3::Constant(0.05), .tau_s = Vec3::Constant(20.0)}
    };
    auto c       = default_config();
    c.gyro_range = LSM9DS1_245dps;
    auto record  = [&](const m::Params &params, double seconds)
    {
        auto model = m::Model::make(c, params, 9).value();
        std::vector<double> g;
        model.advance(
            seconds,
            [&](double) { return std::pair{Vec3::UnitZ().eval(), Vec3::Zero().eval()}; },
            [&](const m::Output &o)
            { g.push_back(o.raw.gyro[1] * model.raw_scale().gyro_dps_per_lsb); }
        );
        return std::pair{g, model.odr_hz()};
    };
    const auto [g, fs] = record(p, 1500.0);
    double worst       = 0.0;
    for (double tau : {1.0, 5.0}) {
        const double expected = markov_avar(tau, 0.05, 2.0) + markov_avar(tau, 0.05, 20.0);
        const double got      = allan_var(g, static_cast<std::size_t>(std::lround(tau * fs)), fs);
        worst                 = std::max(worst, std::abs((got / expected) - 1));
    }
    check(worst < 0.15, "two Markov terms give the sum of their Allan variances", worst);

    /* Same seed, so the scaled model carries the same drift at half the level. */
    auto scaled                        = p;
    scaled.gyro_drift_scale_by_fs[245] = Vec3(1.0, 0.5, 1.0);
    const auto full                    = record(p, 200.0).first;
    const auto half                    = record(scaled, 200.0).first;
    const double ratio = std::sqrt(allan_var(half, 4615, fs) / allan_var(full, 4615, fs));
    check(std::abs(ratio - 0.5) < 0.05, "drift_scale_by_fs scales the drift of its range", ratio);
}

void lsm6dsl() {
    namespace d = chip::lsm6dsl;
    d::Config c;
    c.accel_odr    = LSM6DSL_XL_ODR_416Hz;
    c.gyro_odr     = LSM6DSL_GY_ODR_416Hz;
    c.gyro_bw      = LSM6DSL_HP_DISABLE_LP1_STRONG;
    c.accel_lp2_bw = LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9;
    auto model     = d::Model::make(c, m::Params{}, 1).value();
    const auto &k  = model.cutoffs();
    check(
        std::abs(model.internal_hz() - 16 * 416.0) < 1e-9 && k.accel_aa == 208.0
            && std::abs(k.accel_lp - 416.0 / 9) < 1e-9 && k.gyro_lpf1 == 138.0 && k.gyro_lpf2 == 0.0,
        "LSM6DSL 416 Hz: 16x internal, accel LPF1 ODR/2 then ODR/9, gyro overall 138",
        k.gyro_lpf1
    );

    c.accel_odr    = LSM6DSL_XL_ODR_833Hz;
    c.gyro_odr     = LSM6DSL_GY_ODR_833Hz;
    c.gyro_bw      = LSM6DSL_HP_DISABLE_LP1_LIGHT;
    c.accel_lp2_bw = LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_50;
    model          = d::Model::make(c, m::Params{}, 1).value();
    check(
        model.cutoffs().accel_aa == 208.25 && std::abs(model.cutoffs().accel_lp - 833.0 / 50) < 1e-9
            && model.cutoffs().gyro_lpf1 == 155.0,
        "LSM6DSL 833 Hz, FTYPE 10, low-noise ODR/50: LPF1 ODR/4, gyro 155",
        model.cutoffs().accel_aa
    );

    c.accel_lp2_bw = LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9;
    c.accel_range  = LSM6DSL_4g;
    c.gyro_range   = LSM6DSL_2000dps;
    model          = d::Model::make(c, m::Params{}, 1).value();
    auto out       = run(model, 0.1, Vec3(0.0, 0.0, 1.0), Vec3(0.0, 0.0, 70.0));
    check(
        out.back().raw.accel[2] == 8197 && out.back().raw.gyro[2] == 1000 && model.cutoffs().accel_aa == 400.0,
        "LSM6DSL: z passes through without negation; the analog 400 Hz caps the internal stage",
        out.back().raw.gyro[2]
    );

    c.gyro_bw = LSM6DSL_HP_16mHz_LP1_LIGHT;
    check(!d::Model::make(c, m::Params{}, 1).has_value(), "LSM6DSL gyro HP filter is rejected", 0);
    c.gyro_bw  = LSM6DSL_HP_DISABLE_LP1_STRONG;
    c.gyro_odr = LSM6DSL_GY_ODR_416Hz;
    check(!d::Model::make(c, m::Params{}, 1).has_value(), "LSM6DSL accel and gyro ODR must match", 0);
}

void seeding() {
    m::Params p;
    p.gyro.white_density = Vec3::Constant(0.015);
    p.gyro.turn_on_sigma = Vec3::Constant(1.0);
    auto first           = m::Model::make(default_config(), p, 11).value();
    auto again           = m::Model::make(default_config(), p, 11).value();
    auto other           = m::Model::make(default_config(), p, 12).value();
    const auto a         = run(first, 1.0, Vec3::UnitZ(), Vec3::Zero());
    const auto b         = run(again, 1.0, Vec3::UnitZ(), Vec3::Zero());
    const auto c         = run(other, 1.0, Vec3::UnitZ(), Vec3::Zero());
    bool same = true, differs = false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        same    = same && a[i].raw.gyro[1] == b[i].raw.gyro[1];
        differs = differs || a[i].raw.gyro[1] != c[i].raw.gyro[1];
    }
    check(same && differs, "same seed repeats, another seed differs", 0);
}

}  // namespace

int main() {
    filters();
    config_mapping();
    quantize_and_clip();
    negative_gain();
    decimation_and_filter();
    noise();
    density_by_range();
    drift();
    lsm6dsl();
    seeding();
    return g_failures == 0 ? 0 : 1;
}
