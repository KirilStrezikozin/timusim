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
};

struct Progress {
    double u;
    double du;  /* 1/s */
    double ddu; /* 1/s^2 */
};

inline Progress progress(Profile profile, double t, double duration) {
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
