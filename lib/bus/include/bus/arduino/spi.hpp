/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI bus implementation for Arduino framework.
 */

#ifndef BUS_ARDUINO_SPI_HPP
#define BUS_ARDUINO_SPI_HPP

#ifndef TARGET_PLATFORM_ARDUINO
#error "This header is only for the Arduino framework."
#endif

#include <Arduino.h>
#include <SPI.h>

#include "bus/ibus.hpp"
#include "log/ilog.hpp"

namespace bus::arduino {

class SPIBus : public ISPIBus {
  public:
    SPIBus(
        SPIClass *spi, SPISettings spi_settings, uint8_t cs_pin, logger::ILogger *logger
    ) noexcept
        : spi_(spi), spi_settings_(spi_settings), cs_pin_(cs_pin), logger_(logger) {}

    /* Bus is not copyable, but it is movable. */
    SPIBus(const SPIBus &)            = delete;
    SPIBus &operator=(const SPIBus &) = delete;
    /* */

    SPIBus(SPIBus &&other) noexcept
        /* Invalidate the other bus' hardware state. */
        : spi_(std::exchange(other.spi_, nullptr)),
          spi_settings_(std::exchange(other.spi_settings_, SPISettings())),
          cs_pin_(std::exchange(other.cs_pin_, 0)), locked_(std::exchange(other.locked_, false)) {}

    SPIBus &operator=(SPIBus &&other) noexcept {
        if (this != &other) {
            /* Invalidate the other bus' hardware state. */
            this->spi_          = other.spi_;
            this->spi_settings_ = other.spi_settings_;
            this->cs_pin_       = other.cs_pin_;
            this->locked_       = other.locked_;

            other.spi_          = nullptr;
            other.spi_settings_ = SPISettings();
            other.cs_pin_       = 0;
            other.locked_       = false;
        }

        return *this;
    }

    ~SPIBus() noexcept override = default;

    void lock() noexcept override {
        asm volatile("nop \n nop \n nop");
        this->spi_->beginTransaction(this->spi_settings_);
        digitalWrite(this->cs_pin_, !SPIBus::spi_active_low_);
        this->locked_ = true;
        asm volatile("nop \n nop \n nop");
    }

    void unlock() noexcept override {
        asm volatile("nop \n nop \n nop");
        digitalWrite(this->cs_pin_, SPIBus::spi_active_low_);
        this->spi_->endTransaction();
        this->locked_ = false;
        asm volatile("nop \n nop \n nop");
    }

    tl::expected<int, bus::BusError> write_register(reg_type reg, span<const element_type> buffer)
        override {
        if (!this->locked_) {
            this->logger_->error("{:d}: attempted to write while bus is not locked", this->cs_pin_);
            return tl::make_unexpected(BusError::NOT_LOCKED);
        }

        this->spi_->transfer(reg);
        for (const auto &byte : buffer) {
            this->spi_->transfer(byte);
        }

        return buffer.size();
    }

    tl::expected<int, bus::BusError> read_register(reg_type reg, span<element_type> buffer)
        override {
        if (!this->locked_) {
            this->logger_->error("{:d}: attempted to read while bus is not locked", this->cs_pin_);
            return tl::make_unexpected(BusError::NOT_LOCKED);
        }

        this->spi_->transfer(reg);
        for (auto &byte : buffer) {
            byte = this->spi_->transfer(0x00);
        }

        return buffer.size();
    }

  private:
    SPIClass *spi_;
    SPISettings spi_settings_;
    uint8_t cs_pin_;
    bool locked_ = false;
    logger::ILogger *logger_;
    static constexpr bool spi_active_low_ = true;
};

} /* namespace bus::arduino */

#endif /* BUS_ARDUINO_SPI_HPP */
