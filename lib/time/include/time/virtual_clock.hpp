/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Process-wide virtual clock for host builds. Simulated devices read it, and
 * platform sleeps advance it.
 */

#ifndef TIME_VIRTUAL_CLOCK_HPP
#define TIME_VIRTUAL_CLOCK_HPP

#include <cstdint>

namespace vclock {

inline std::uint64_t now_us_ = 0;

[[nodiscard]] inline std::uint64_t now_us() noexcept { return now_us_; }

inline void advance_us(std::uint64_t us) noexcept { now_us_ += us; }

inline void set_us(std::uint64_t us) noexcept { now_us_ = us; }

} /* namespace vclock */

#endif /* TIME_VIRTUAL_CLOCK_HPP */
