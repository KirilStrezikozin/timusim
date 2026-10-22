/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-independent serial communication bus interface for register-based
 * devices.
 */

#ifndef BUS_IBUS_HPP
#define BUS_IBUS_HPP

#include <cassert>
#include <cstdint>
#include <cstdio>

#include <tl/expected.hpp>

#include "log/ilog.hpp"
#include "span.hpp"

namespace bus {

enum class BusError : std::uint8_t {
    UNKNOWN,
    TIMEOUT,
    NACK,
    NOT_LOCKED,
};

class IBus {
  public:
    using reg_type     = uint8_t;
    using element_type = uint8_t;
    using size_type    = std::size_t;

    IBus(const IBus &)            = delete;
    IBus(IBus &&)                 = delete;
    IBus &operator=(const IBus &) = delete;
    IBus &operator=(IBus &&)      = delete;

    IBus()                   = default;
    virtual ~IBus() noexcept = default;

    virtual void lock() noexcept   = 0;
    virtual void unlock() noexcept = 0;

    virtual tl::expected<int, BusError> read_register(reg_type reg, span<element_type> buffer) = 0;

    virtual tl::expected<int, BusError> write_register(
        reg_type reg, span<const element_type> buffer
    ) = 0;
};

/* Marker interface for I2C buses. */
class II2CBus : public IBus {};

/* Marker interface for SPI buses. */
class ISPIBus : public IBus {};

class BusLockGuard {
  public:
    /* Bus lock guards should never be copied or moved. */
    BusLockGuard(const BusLockGuard &)            = delete;
    BusLockGuard &operator=(const BusLockGuard &) = delete;

    BusLockGuard(BusLockGuard &&)            = delete;
    BusLockGuard &operator=(BusLockGuard &&) = delete;
    /* */

    explicit BusLockGuard(IBus &bus) noexcept : bus_(bus) { this->bus_.lock(); }

    ~BusLockGuard() noexcept { this->bus_.unlock(); }

  private:
    IBus &bus_;
};

} /* namespace bus */

template <> struct fmt::formatter<bus::BusError> : fmt::formatter<const char *> {
    template <typename FmtContext> auto format(const bus::BusError &error, FmtContext &ctx) const {
        const char *name = nullptr;
        switch (error) {
        case bus::BusError::UNKNOWN:
            name = "UNKNOWN";
            break;
        case bus::BusError::TIMEOUT:
            name = "TIMEOUT";
            break;
        case bus::BusError::NACK:
            name = "NACK";
            break;
        case bus::BusError::NOT_LOCKED:
            name = "NOT_LOCKED";
            break;
        }

        return fmt::formatter<const char *>::format(name, ctx);
    }
};

#endif /* BUS_IBUS_HPP */
