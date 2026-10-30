/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * LSM6DSL Register Map
 */

#ifndef IMU_LSM6DSL_REG_H
#define IMU_LSM6DSL_REG_H

namespace imu::lsm6dsl::register_map {

/* Communication interface. */
#define LSM6DSL_IF_SPI_REG_SET_READ  0x80
#define LSM6DSL_IF_SPI_REG_SET_WRITE 0x00
#define LSM6DSL_IF_SPI_REG_ADDR_MASK 0x7F

#define LSM6DSL_IF_I2C_REG_SET_READ  0x00
#define LSM6DSL_IF_I2C_REG_SET_WRITE 0x00
#define LSM6DSL_IF_I2C_REG_ADDR_MASK 0xFF
/* */

} /* namespace imu::lsm6dsl::register_map */

#endif /* IMU_LSM6DSL_REG_H */
