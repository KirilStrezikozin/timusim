/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * LSM9DS1 Register Map
 */

#ifndef IMU_LSM9DS1_REG_H
#define IMU_LSM9DS1_REG_H

namespace imu::lsm9ds1::register_map {

/* Communication interface. */
#define LSM9DS1_IF_SPI_REG_SET_READ  0x80
#define LSM9DS1_IF_SPI_REG_SET_WRITE 0x00
#define LSM9DS1_IF_SPI_REG_ADDR_MASK 0x7F

#define LSM9DS1_IF_I2C_REG_SET_READ  0x00
#define LSM9DS1_IF_I2C_REG_SET_WRITE 0x00
#define LSM9DS1_IF_I2C_REG_ADDR_MASK 0xFF
/* */

} /* namespace imu::lsm9ds1::register_map */

#endif /* IMU_LSM9DS1_REG_H */
