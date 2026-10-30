/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-agnostic sleep functions.
 */

#ifndef TIME_SLEEP_H
#define TIME_SLEEP_H

#ifdef TARGET_PLATFORM_HOST

/* Sleeping on the host advances the virtual clock instead of waiting. */
#include "time/virtual_clock.hpp"
#define sleep_ms_impl(ms) vclock::advance_us(static_cast<std::uint64_t>(ms) * 1000U)
#define sleep_us_impl(us) vclock::advance_us(us)
/* */

#elif defined(TARGET_PLATFORM_ARDUINO)

/* Sleep functions with Arduino framework. */
#include <Arduino.h>
#define sleep_ms_impl(ms) delay(ms)
#define sleep_us_impl(us) delayMicroseconds(us)
/* */

#else
#error "Unsupported target platform. Please define sleep functions for your platform."
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sleep for the specified number of milliseconds.
 *
 * @param ms The number of milliseconds to sleep.
 */
static inline void platform_sleep_ms(uint32_t ms) {
    sleep_ms_impl(ms);
}

/**
 * @brief Sleep for the specified number of microseconds.
 *
 * @param us The number of microseconds to sleep.
 */
static inline void platform_sleep_us(uint32_t us) {
    sleep_us_impl(us);
}

#ifdef __cplusplus
}
#endif

#undef sleep_ms_impl

#endif /* TIME_SLEEP_H */
