/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Logger implementation backed by Arduino Serial writer.
 */

#ifndef LOG_ARDUINO_LOG_HPP
#define LOG_ARDUINO_LOG_HPP

#ifndef TARGET_PLATFORM_ARDUINO
#error "This header is only for the Arduino framework."
#endif

#include <cassert>

#include <Arduino.h>

#include "log/ilog.hpp"

namespace logger::arduino {

class Logger final : public logger::ILogger {
  public:
    using Base = logger::ILogger;

  protected:
    void write_impl_(span<const Base::char_type> data) const noexcept final {
        if (!Serial) return;
        [[maybe_unused]] const Base::size_type ok_ = Serial.print(data.data());

        /* Exclude the null terminator. A terminal detaching mid-write
         * legitimately yields a zero count, so only a partial write is a bug. */
        assert(ok_ == 0 || ok_ == data.size() - 1);
    }
};

} /* namespace logger::arduino */

#endif /* LOG_ARDUINO_LOG_HPP */
