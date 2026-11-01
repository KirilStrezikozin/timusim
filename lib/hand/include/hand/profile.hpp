/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Time profiles of a move: progress u(s) for s = t / T in [0, 1], with its
 * analytic first and second time derivatives.
 */

#ifndef HAND_PROFILE_HPP
#define HAND_PROFILE_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace hand {

enum class Profile : std::uint8_t {
    /* 10s^3 - 15s^4 + 6s^5: zero velocity and acceleration at both ends (Flash and Hogan 1985). */
    MIN_JERK,
    /* 0.5 - 0.5 cos(pi s), the glove sim's ease: acceleration steps at the ends. */
    COSINE,
    /* Velocity s^(a-1) (1-s)^(b-1), the regularized incomplete beta I_s(a, b) as progress:
     * a = b = 3 is MIN_JERK, a < b peaks early. a, b >= 2 keep the acceleration finite. */
    BETA,
};

struct Shape {
    double a = 3.0;
    double b = 3.0;
};

namespace detail {

/* Continued fraction for I_x(a, b) (Numerical Recipes, betacf), x < (a + 1) / (a + b + 2). */
inline double beta_cf(double a, double b, double x) {
    constexpr int MAX_ITER = 200;
    constexpr double EPS = 1e-14, TINY = 1e-300;
    const double qab = a + b, qap = a + 1.0, qam = a - 1.0;
    double c = 1.0, d = 1.0 - (qab * x / qap);
    if (std::abs(d) < TINY) { d = TINY; }
    d        = 1.0 / d;
    double h = d;
    for (int m = 1; m <= MAX_ITER; ++m) {
        const int m2 = 2 * m;
        double aa    = m * (b - m) * x / ((qam + m2) * (a + m2));
        d            = 1.0 + (aa * d);
        c            = 1.0 + (aa / c);
        d            = 1.0 / (std::abs(d) < TINY ? TINY : d);
        c            = std::abs(c) < TINY ? TINY : c;
        h *= d * c;
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2));
        d  = 1.0 + (aa * d);
        c  = 1.0 + (aa / c);
        d  = 1.0 / (std::abs(d) < TINY ? TINY : d);
        c  = std::abs(c) < TINY ? TINY : c;
        const double del = d * c;
        h *= del;
        if (std::abs(del - 1.0) < EPS) { break; }
    }
    return h;
}

inline double log_beta(double a, double b) {
    return std::lgamma(a) + std::lgamma(b) - std::lgamma(a + b);
}

inline double incomplete_beta(double a, double b, double x) {
    if (x <= 0.0) { return 0.0; }
    if (x >= 1.0) { return 1.0; }
    const double front = std::exp((a * std::log(x)) + (b * std::log1p(-x)) - log_beta(a, b));
    if (x < (a + 1.0) / (a + b + 2.0)) { return front * beta_cf(a, b, x) / a; }
    return 1.0 - (front * beta_cf(b, a, 1.0 - x) / b);
}

} /* namespace detail */

struct Progress {
    double u;
    double du;  /* 1/s */
    double ddu; /* 1/s^2 */
};

inline Progress progress(Profile profile, double t, double duration, Shape shape = {}) {
    if (duration <= 0.0) { return {1.0, 0.0, 0.0}; }
    const double s = std::clamp(t / duration, 0.0, 1.0);
    const bool inside = t > 0.0 && t < duration;
    switch (profile) {
    case Profile::COSINE: {
        const double k = M_PI / duration;
        return {
            0.5 - (0.5 * std::cos(M_PI * s)),
            inside ? 0.5 * k * std::sin(M_PI * s) : 0.0,
            inside ? 0.5 * k * k * std::cos(M_PI * s) : 0.0,
        };
    }
    case Profile::BETA: {
        const double a = shape.a, b = shape.b;
        if (!inside) { return {s, 0.0, 0.0}; }
        const double v = std::exp(((a - 1.0) * std::log(s)) + ((b - 1.0) * std::log1p(-s)) - detail::log_beta(a, b));
        return {
            detail::incomplete_beta(a, b, s),
            v / duration,
            v * (((a - 1.0) / s) - ((b - 1.0) / (1.0 - s))) / (duration * duration),
        };
    }
    case Profile::MIN_JERK:
    default: {
        const double s2 = s * s;
        const double s3 = s2 * s;
        return {
            s3 * (10.0 - 15.0 * s + 6.0 * s2),
            30.0 * s2 * (1.0 - s) * (1.0 - s) / duration,
            60.0 * s * (1.0 - s) * (1.0 - 2.0 * s) / (duration * duration),
        };
    }
    }
}

} /* namespace hand */

#endif /* HAND_PROFILE_HPP */
