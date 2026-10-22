/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C bus implementation for Arduino framework.
 */

#ifndef BUS_ARDUINO_I2C_HPP
#define BUS_ARDUINO_I2C_HPP

#ifndef TARGET_PLATFORM_ARDUINO
#error "This header is only for the Arduino framework."
#endif

#include <Arduino.h>
#include <Wire.h>

#include "bus/ibus.hpp"
#include "log/ilog.hpp"

namespace bus::arduino {

class I2CBus : public II2CBus {
  public:
    using addr_type = uint8_t;

    I2CBus(TwoWire *wire, addr_type addr, logger::ILogger *logger) noexcept
        : wire_(wire), addr_(addr), logger_(logger) {}

    /* Bus is not copyable, but it is movable. */
    I2CBus(const I2CBus &)            = delete;
    I2CBus &operator=(const I2CBus &) = delete;
    /* */

    I2CBus(I2CBus &&other) noexcept
        /* Invalidate the other bus' hardware state. */
        : wire_(std::exchange(other.wire_, nullptr)), addr_(std::exchange(other.addr_, 0)),
          locked_(std::exchange(other.locked_, false)) {}

    I2CBus &operator=(I2CBus &&other) noexcept {
        if (this != &other) {
            /* Invalidate the other bus' hardware state. */
            this->wire_   = other.wire_;
            this->addr_   = other.addr_;
            this->locked_ = other.locked_;

            other.wire_   = nullptr;
            other.addr_   = 0;
            other.locked_ = false;
        }

        return *this;
    }

    ~I2CBus() noexcept override = default;

    void lock() noexcept override { this->locked_ = true; }
    void unlock() noexcept override { this->locked_ = false; }

    tl::expected<int, bus::BusError> write_register(reg_type reg, span<const element_type> buffer)
        override {
        if (!this->locked_) {
            this->logger_->error("{:#x}: attempted to write while bus is not locked", this->addr_);
            return tl::make_unexpected(BusError::NOT_LOCKED);
        }

        this->wire_->beginTransmission(this->addr_);
        this->wire_->write(reg);

        this->wire_->write(buffer.data(), buffer.size());

        uint8_t res = this->wire_->endTransmission(true); /* true = send STOP bit. */
        if (res != 0) { return this->resolve_error_(res); }

        return buffer.size();
    }

    tl::expected<int, bus::BusError> read_register(reg_type reg, span<element_type> buffer)
        override {
        if (!this->locked_) {
            this->logger_->error("{:#x}: attempted to read while bus is not locked", this->addr_);
            return tl::make_unexpected(BusError::NOT_LOCKED);
        }

        this->wire_->beginTransmission(this->addr_);
        this->wire_->write(reg);

        uint8_t res = this->wire_->endTransmission(false); /* false = restart instead of stop. */
        if (res != 0) { return this->resolve_error_(res); }

        uint8_t bytes_received = this->wire_->requestFrom(
            this->addr_, buffer.size(), true /* send STOP bit. */
        );

        if (bytes_received == 0 && buffer.size() > 0) {
            return tl::make_unexpected(BusError::TIMEOUT);
        }

        for (size_t i = 0; i < bytes_received; i++) {
            if (this->wire_->available()) { buffer[i] = this->wire_->read(); }
        }

        return bytes_received;
    }

  private:
    TwoWire *wire_;
    addr_type addr_;
    logger::ILogger *logger_;
    bool locked_ = false;

    [[nodiscard]] tl::expected<int, bus::BusError> resolve_error_(uint8_t error) {
        /* Arduino `endTransmission()` return codes:
         * 0: success.
         * 1: data too long to fit in transmit buffer.
         * 2: received NACK on transmit of address.
         * 3: received NACK on transmit of data.
         * 4: other error.
         * 5: timeout.
         */
        switch (error) {
        case 1:
            this->logger_->error("{:#x}: data too long to fit in transmit buffer", this->addr_);
            return tl::make_unexpected(BusError::UNKNOWN);
        case 2:
        case 3:
            this->logger_->error("{:#x}: address or data NACK", this->addr_);
            return tl::make_unexpected(BusError::NACK);
        case 5:
            this->logger_->error("{:#x}: timeout", this->addr_);
            return tl::make_unexpected(BusError::TIMEOUT);
        default:
            this->logger_->error("{:#x}: unknown hardware error {:d}", this->addr_, error);
            return tl::make_unexpected(BusError::UNKNOWN);
        }
    }
};

} /* namespace bus::arduino */

#endif /* BUS_ARDUINO_I2C_HPP */
