/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Runs imu/conformance.hpp on the palm LSM6DSL and the finger LSM9DS1.
 * Protocol: boards still for 5 s in any orientation, then rotate them slowly in
 * all directions for 20 s. Results print every 2 s; reset the board to start over.
 */

#include <Arduino.h>
#include <Wire.h>

#include "bus/arduino/i2c.hpp"
#include "imu/conformance.hpp"
#include "imu/iimu.hpp"
#include "imu/lsm6dsl/i2c.hpp"
#include "imu/lsm9ds1/i2c.hpp"
#include "log/arduino/log.hpp"

#define PALM_PIN_SDA   PB11
#define PALM_PIN_SCL   PB10
#define FINGER_PIN_SDA PB9
#define FINGER_PIN_SCL PB8
#define I2C_SPEED      400000

namespace {

using imu::conformance::vector_type;

constexpr unsigned long REPORT_PERIOD_MS = 2000;

struct Sensor {
    const char *name;
    imu::IIMU *imu;
    imu::conformance::RestCheck rest;
    imu::conformance::HandednessCheck handed;
    unsigned long last_us = 0;
};

logger::arduino::Logger *g_logger = nullptr;
Sensor *g_sensors[2]              = {nullptr, nullptr};

imu::IIMU *make_palm_lsm6dsl() {
    imu::lsm6dsl::config::IMUConfig config;
    config.accel_odr = LSM6DSL_XL_ODR_416Hz;
    config.gyro_odr  = LSM6DSL_GY_ODR_416Hz;
    config.gyro_bw   = LSM6DSL_HP_DISABLE_LP1_STRONG;

    auto *wire = new TwoWire(PALM_PIN_SDA, PALM_PIN_SCL);
    wire->begin();
    wire->setClock(I2C_SPEED);
    auto *bus = new bus::arduino::I2CBus(
        wire,
        static_cast<bus::arduino::I2CBus::addr_type>(imu::lsm6dsl::config::IMUI2CAddress::LSM6DSL_SA0_LOW),
        g_logger
    );
    return new imu::lsm6dsl::imu::I2CIMU(config, bus, g_logger);
}

imu::IIMU *make_finger_lsm9ds1() {
    imu::lsm9ds1::config::IMUConfig config;
    config.imu_odr = LSM9DS1_IMU_476Hz;

    auto *wire = new TwoWire(FINGER_PIN_SDA, FINGER_PIN_SCL);
    wire->begin();
    wire->setClock(I2C_SPEED);
    auto *bus = new bus::arduino::I2CBus(
        wire,
        static_cast<bus::arduino::I2CBus::addr_type>(imu::lsm9ds1::config::IMUI2CAddress::LSM9DS1_SA0_HIGH),
        g_logger
    );
    return new imu::lsm9ds1::imu::I2CIMU(config, bus, g_logger);
}

void poll(Sensor &s) {
    auto ready = s.imu->get_is_data_ready();
    if (!ready || !ready.value()) { return; }

    vector_type accel;
    vector_type gyro;
    if (!s.imu->get_accel_data(accel) || !s.imu->get_gyro_data(gyro)) { return; }

    const unsigned long now_us = micros();
    const float dt_s           = s.last_us == 0 ? 0.0F : static_cast<float>(now_us - s.last_us) * 1e-6F;
    s.last_us                  = now_us;

    s.rest.update(accel, gyro);
    if (dt_s > 0.0F) { s.handed.update(accel, gyro - s.rest.gyro_bias_dps(), dt_s); }
}

void report(const Sensor &s) {
    const auto rest   = s.rest.result();
    const auto handed = s.handed.result();
    g_logger->info(
        "[conf] {} rest={} n={} mean=({:.3f},{:.3f},{:.3f}) |a|-1={:.4f}g bias=({:.2f},{:.2f},{:.2f})dps "
        "handed={} blocks={} accel_only={} corr={:.3f} gain={:.3f}",
        s.name,
        rest.pass ? "PASS" : "FAIL",
        rest.samples,
        rest.mean_accel_g.x(),
        rest.mean_accel_g.y(),
        rest.mean_accel_g.z(),
        rest.norm_error_g,
        rest.mean_gyro_dps.x(),
        rest.mean_gyro_dps.y(),
        rest.mean_gyro_dps.z(),
        handed.pass ? "PASS" : "FAIL",
        handed.blocks,
        handed.accel_only_blocks,
        handed.correlation,
        handed.gain
    );
}

} // namespace

void setup() {
    Serial.begin(115200);
    for (unsigned long start = millis(); !Serial && millis() - start < 2000;) {
        delay(10);
    }

    g_logger = new logger::arduino::Logger();
    g_logger->info("Starting up...");

    /* The boards start in any orientation, so the up-axis test is skipped; read the
     * sign off the printed mean. Face up, the LSM6DSL's +Z points up, and the LSM9DS1's
     * -Z since its driver negates Z. */
    g_sensors[0] = new Sensor{
        "palm_lsm6dsl",
        make_palm_lsm6dsl(),
        imu::conformance::RestCheck(vector_type()),
        imu::conformance::HandednessCheck(),
    };
    g_sensors[1] = new Sensor{
        "finger_lsm9ds1",
        make_finger_lsm9ds1(),
        imu::conformance::RestCheck(vector_type()),
        imu::conformance::HandednessCheck(),
    };

    for (Sensor *s : g_sensors) {
        if (auto res = s->imu->init(); !res) {
            while (true) {
                g_logger->error("{} init failed: {}", s->name, res.error());
                delay(1000);
            }
        }
    }

    g_logger->info("Hold both boards still for 5 s, then rotate them slowly in all directions for 20 s.");
}

void loop() {
    static unsigned long last_report_ms = 0;

    for (Sensor *s : g_sensors) {
        poll(*s);
    }

    if (millis() - last_report_ms >= REPORT_PERIOD_MS) {
        last_report_ms = millis();
        for (const Sensor *s : g_sensors) {
            report(*s);
        }
    }
}
