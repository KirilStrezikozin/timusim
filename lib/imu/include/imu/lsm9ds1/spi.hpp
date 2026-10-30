/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * LSM9DS1 SPI Implementation
 */

#ifndef IMU_LSM9DS1_SPI_HPP
#define IMU_LSM9DS1_SPI_HPP

#include <cstdint>

#include <tl/expected.hpp>

#include "bus/ibus.hpp"
#include "imu/iimu.hpp"
#include "imu/lsm9ds1/base.hpp"
#include "imu/lsm9ds1/register_map.hpp"
#include "log/ilog.hpp"
#include "span.hpp"

namespace imu::lsm9ds1::imu {

class SPIIMU final : public base::BaseIMU_ {
  public:
    using Base           = base::BaseIMU_;
    using raw_value_type = int16_t;
    using bus_value_type = bus::IBus::element_type;

    SPIIMU(config::IMUConfig config, bus::ISPIBus *bus, logger::ILogger *logger) noexcept
        : BaseIMU_(config, bus, logger) {}

  protected:
    tl::expected<void, IMUError> write_registers_(
        bus::IBus::reg_type reg, span<const bus_value_type> data
    ) noexcept final {
        bus::BusLockGuard lock(*this->bus_); /* Lock the bus for this op. */

        auto res = this->bus_->write_register(
            (reg & LSM9DS1_IF_SPI_REG_ADDR_MASK) | LSM9DS1_IF_SPI_REG_SET_WRITE, data
        );

        if (!res) {
            this->logger_->error(
                "failed to write registers starting at {:#x}: {}", reg, res.error()
            );
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        return {};
    }

    tl::expected<void, IMUError> read_registers_(
        bus::IBus::reg_type reg, span<bus_value_type> data
    ) noexcept final {
        bus::BusLockGuard lock(*this->bus_); /* Lock the bus for this op. */

        auto res = this->bus_->read_register(
            (reg & LSM9DS1_IF_SPI_REG_ADDR_MASK) | LSM9DS1_IF_SPI_REG_SET_READ, data
        );

        if (!res) {
            this->logger_->error(
                "failed to read registers starting at {:#x}: {}", reg, res.error()
            );
            return tl::make_unexpected(IMUError::BUS_ERROR);
        }

        return {};
    }
};

} /* namespace imu::lsm9ds1::imu */

#endif /* IMU_LSM9DS1_SPI_HPP */
