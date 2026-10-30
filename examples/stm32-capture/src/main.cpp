/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Streams every sample of the palm and finger IMUs, and of the palm LSM9DS1 when
 * PALM_LSM9 is set, from their FIFOs in the format of capture/protocol.hpp.
 * `timusim-capture` records it.
 */

#include <Arduino.h>
#include <Wire.h>

#include "bus/arduino/i2c.hpp"
#include "capture/protocol.hpp"
#include "imu/iimu.hpp"
#include "imu/lsm6dsl/i2c.hpp"
#include "imu/lsm6dsl/utils.hpp"
#include "imu/lsm9ds1/i2c.hpp"
#include "imu/lsm9ds1/utils.hpp"
#include "log/ilog.hpp"

#ifndef FW_HASH
#define FW_HASH "none"
#endif

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 921600
#endif

/* Finger LSM9DS1 settings; the V1 envs in platformio.ini override them. */
#ifndef CAPTURE_CONFIG
#define CAPTURE_CONFIG "default"
#endif
#ifndef FINGER_ODR
#define FINGER_ODR LSM9DS1_IMU_476Hz
#endif
#ifndef FINGER_ACCEL_FS
#define FINGER_ACCEL_FS LSM9DS1_4g
#endif
#ifndef FINGER_GYRO_FS
#define FINGER_GYRO_FS LSM9DS1_2000dps
#endif
#ifndef FINGER_GYRO_OUT_PATH
#define FINGER_GYRO_OUT_PATH LSM9DS1_LPF1_OUT
#endif
#ifndef FINGER_GYRO_BW
#define FINGER_GYRO_BW LSM9DS1_LP_ULTRA_LIGHT
#endif
#ifndef FINGER_ACCEL_BW
#define FINGER_ACCEL_BW LSM9DS1_LP_ODR_DIV_9
#endif
#ifndef FINGER_SA0
#define FINGER_SA0 LSM9DS1_SA0_LOW
#endif

/* Second LSM9DS1, on the finger's bus with SDO_AG high. */
#ifndef PALM_LSM9
#define PALM_LSM9 0
#endif
#ifndef PALM_LSM9_ODR
#define PALM_LSM9_ODR LSM9DS1_IMU_476Hz
#endif
#ifndef PALM_LSM9_ACCEL_FS
#define PALM_LSM9_ACCEL_FS LSM9DS1_4g
#endif
#ifndef PALM_LSM9_GYRO_FS
#define PALM_LSM9_GYRO_FS LSM9DS1_2000dps
#endif
#ifndef PALM_LSM9_GYRO_OUT_PATH
#define PALM_LSM9_GYRO_OUT_PATH LSM9DS1_LPF1_OUT
#endif
#ifndef PALM_LSM9_GYRO_BW
#define PALM_LSM9_GYRO_BW LSM9DS1_LP_ULTRA_LIGHT
#endif
#ifndef PALM_LSM9_ACCEL_BW
#define PALM_LSM9_ACCEL_BW LSM9DS1_LP_ODR_DIV_9
#endif

/* Palm LSM6DSL settings; the palm sweep envs override them. */
#ifndef PALM_ODR
#define PALM_ODR 416
#endif
#ifndef PALM_ACCEL_FS
#define PALM_ACCEL_FS LSM6DSL_4g
#endif
#ifndef PALM_GYRO_FS
#define PALM_GYRO_FS LSM6DSL_2000dps
#endif
#ifndef PALM_GYRO_BW
#define PALM_GYRO_BW LSM6DSL_HP_DISABLE_LP1_STRONG
#endif
#ifndef PALM_ACCEL_LP2
#define PALM_ACCEL_LP2 LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9
#endif

#define PALM_PIN_SDA   PB11
#define PALM_PIN_SCL   PB10
#define FINGER_PIN_SDA PB9
#define FINGER_PIN_SCL PB8
#define I2C_SPEED      400000

namespace {

constexpr unsigned long HEADER_PERIOD_MS = 5000;
constexpr unsigned long TEMP_PERIOD_MS   = 1000;
constexpr std::size_t SENSOR_COUNT      = PALM_LSM9 ? 3 : 2;

struct Sensor {
    const char *name;
    const char *chip;
    float odr_hz;
    float accel_fs_g;
    float gyro_fs_dps;
    /* Filter settings as register enum values, for the capture header. */
    int accel_bw;
    int gyro_bw;
    int gyro_out_path;
    imu::IIMU *imu;
    imu::IIMU::RawScale scale;
    std::uint32_t next_index;
};

capture::PacketWriter g_writer;

void send(capture::PacketWriter::frame_type frame) {
    Serial.write(frame.data(), frame.size());
}

class PacketLogger final : public logger::ILogger {
  public:
    using Base = logger::ILogger;

  protected:
    void write_impl_(span<const Base::char_type> data) const noexcept final {
        /* The logger's buffer ends with a null terminator. */
        const std::size_t len = data.size() > 0 ? data.size() - 1 : 0;
        capture::write_text(this->writer_, capture::PacketType::TEXT, {data.data(), len});
        send(this->writer_.finish());
    }

  private:
    mutable capture::PacketWriter writer_;
};

PacketLogger *g_logger = nullptr;
Sensor g_sensors[SENSOR_COUNT];

TwoWire *make_wire(std::uint32_t sda, std::uint32_t scl) {
    auto *wire = new TwoWire(sda, scl);
    wire->begin();
    wire->setClock(I2C_SPEED);
    return wire;
}

static_assert(
    imu::lsm6dsl::utils::accel_odr(PALM_ODR) != LSM6DSL_XL_ODR_ND,
    "PALM_ODR must be 104, 208, 416 or 833"
);

Sensor make_palm_lsm6dsl() {
    imu::lsm6dsl::config::IMUConfig config;
    config.accel_odr    = imu::lsm6dsl::utils::accel_odr(PALM_ODR);
    config.gyro_odr     = imu::lsm6dsl::utils::gyro_odr(PALM_ODR);
    config.accel_range  = PALM_ACCEL_FS;
    config.gyro_range   = PALM_GYRO_FS;
    config.gyro_bw      = PALM_GYRO_BW;
    config.accel_lp2_bw = PALM_ACCEL_LP2;

    auto *bus = new bus::arduino::I2CBus(
        make_wire(PALM_PIN_SDA, PALM_PIN_SCL),
        static_cast<bus::arduino::I2CBus::addr_type>(
            imu::lsm6dsl::config::IMUI2CAddress::LSM6DSL_SA0_LOW
        ),
        g_logger
    );
    return Sensor{
        .name          = "palm",
        .chip          = "LSM6DSL",
        .odr_hz        = static_cast<float>(PALM_ODR),
        .accel_fs_g    = static_cast<float>(imu::lsm6dsl::utils::accel_fs_g(config.accel_range)),
        .gyro_fs_dps   = static_cast<float>(imu::lsm6dsl::utils::gyro_fs_dps(config.gyro_range)),
        .accel_bw      = static_cast<int>(config.accel_lp2_bw),
        .gyro_bw       = static_cast<int>(config.gyro_bw),
        .gyro_out_path = -1,
        .imu           = new imu::lsm6dsl::imu::I2CIMU(config, bus, g_logger),
        .scale         = {},
        .next_index    = 0,
    };
}

static_assert(
    imu::lsm9ds1::utils::odr_hz(FINGER_ODR) > 0.0, "FINGER_ODR must have both accel and gyro on"
);

Sensor make_lsm9ds1(
    const char *name,
    TwoWire *wire,
    imu::lsm9ds1::config::IMUI2CAddress address,
    const imu::lsm9ds1::config::IMUConfig &config
) {
    auto *bus = new bus::arduino::I2CBus(
        wire, static_cast<bus::arduino::I2CBus::addr_type>(address), g_logger
    );
    return Sensor{
        .name          = name,
        .chip          = "LSM9DS1",
        .odr_hz        = static_cast<float>(imu::lsm9ds1::utils::odr_hz(config.imu_odr)),
        .accel_fs_g    = static_cast<float>(imu::lsm9ds1::utils::accel_fs_g(config.accel_range)),
        .gyro_fs_dps   = static_cast<float>(imu::lsm9ds1::utils::gyro_fs_dps(config.gyro_range)),
        .accel_bw      = static_cast<int>(config.accel_bw),
        .gyro_bw       = static_cast<int>(config.gyro_bw),
        .gyro_out_path = static_cast<int>(config.gyro_out_path),
        .imu           = new imu::lsm9ds1::imu::I2CIMU(config, bus, g_logger),
        .scale         = {},
        .next_index    = 0,
    };
}

imu::lsm9ds1::config::IMUConfig finger_config() {
    imu::lsm9ds1::config::IMUConfig config;
    config.imu_odr       = FINGER_ODR;
    config.accel_range   = FINGER_ACCEL_FS;
    config.gyro_range    = FINGER_GYRO_FS;
    config.gyro_out_path = FINGER_GYRO_OUT_PATH;
    config.gyro_bw       = FINGER_GYRO_BW;
    config.accel_bw      = FINGER_ACCEL_BW;
    return config;
}

#if PALM_LSM9
static_assert(
    imu::lsm9ds1::utils::odr_hz(PALM_LSM9_ODR) > 0.0,
    "PALM_LSM9_ODR must have both accel and gyro on"
);

imu::lsm9ds1::config::IMUConfig palm_lsm9_config() {
    imu::lsm9ds1::config::IMUConfig config;
    config.imu_odr       = PALM_LSM9_ODR;
    config.accel_range   = PALM_LSM9_ACCEL_FS;
    config.gyro_range    = PALM_LSM9_GYRO_FS;
    config.gyro_out_path = PALM_LSM9_GYRO_OUT_PATH;
    config.gyro_bw       = PALM_LSM9_GYRO_BW;
    config.accel_bw      = PALM_LSM9_ACCEL_BW;
    return config;
}
#endif

void send_header() {
    fmt::basic_memory_buffer<char, 256> buf;
    fmt::format_to(
        std::back_inserter(buf),
        "fw={}\nconfig={}\nuid={:08x}{:08x}{:08x}\nsensors={}\nt_us={}\n",
        FW_HASH,
        CAPTURE_CONFIG,
        HAL_GetUIDw2(),
        HAL_GetUIDw1(),
        HAL_GetUIDw0(),
        SENSOR_COUNT,
        micros()
    );
    capture::write_text(g_writer, capture::PacketType::HEADER, {buf.data(), buf.size()});
    send(g_writer.finish());

    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        const Sensor &s = g_sensors[i];
        buf.clear();
        fmt::format_to(
            std::back_inserter(buf),
            "s{0}.name={1}\ns{0}.chip={2}\ns{0}.odr_hz={3}\ns{0}.accel_fs_g={4}\ns{0}.gyro_fs_dps={"
            "5}\n"
            "s{0}.accel_lsb_g={6:.9g}\ns{0}.gyro_lsb_dps={7:.9g}\ns{0}.accel_bw={8}\ns{0}.gyro_bw={"
            "9}\n"
            "s{0}.gyro_out_path={10}\ns{0}.next_index={11}\n",
            i,
            s.name,
            s.chip,
            s.odr_hz,
            s.accel_fs_g,
            s.gyro_fs_dps,
            s.scale.accel_g_per_lsb,
            s.scale.gyro_dps_per_lsb,
            s.accel_bw,
            s.gyro_bw,
            s.gyro_out_path,
            s.next_index
        );
        capture::write_text(g_writer, capture::PacketType::HEADER, {buf.data(), buf.size()});
        send(g_writer.finish());
    }
}

[[noreturn]] void halt(const char *what, imu::IMUError error) {
    while (true) {
        g_logger->error("{}: {}", what, error);
        delay(1000);
    }
}

void poll(std::uint8_t id, Sensor &s) {
    imu::IIMU::RawSample batch[capture::MAX_BATCH];
    bool overrun = false;
    auto n       = s.imu->read_fifo({batch, capture::MAX_BATCH}, overrun);
    if (!n) {
        g_logger->error("{} read_fifo: {}", s.name, n.error());
        return;
    }
    if (n.value() == 0 && !overrun) { return; }

    capture::write_samples(g_writer, id, s.next_index, micros(), overrun, {batch, n.value()});
    send(g_writer.finish());
    s.next_index += n.value();
}

}  // namespace

/* libc's stdio, pulled in by fmt's error path, needs this. Text on the wire must be
 * framed, so stdio output is dropped. */
extern "C" int _write([[maybe_unused]] int fd, [[maybe_unused]] char *buf, int len) {
    return len;
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    for (unsigned long start = millis(); !Serial && millis() - start < 2000;) {
        delay(10);
    }

    g_logger = new PacketLogger();
    g_logger->info("Starting up...");

    using imu::lsm9ds1::config::IMUI2CAddress;
    g_sensors[0]        = make_palm_lsm6dsl();
    TwoWire *finger_bus = make_wire(FINGER_PIN_SDA, FINGER_PIN_SCL);
    g_sensors[1] = make_lsm9ds1("finger", finger_bus, IMUI2CAddress::FINGER_SA0, finger_config());
#if PALM_LSM9
    g_sensors[2] = make_lsm9ds1(
        "palm_lsm9", finger_bus, IMUI2CAddress::LSM9DS1_SA0_HIGH, palm_lsm9_config()
    );
#endif

    for (Sensor &s : g_sensors) {
        if (auto res = s.imu->init(); !res) { halt(s.name, res.error()); }
        auto scale = s.imu->get_raw_scale();
        if (!scale) { halt(s.name, scale.error()); }
        s.scale = scale.value();
    }
    for (Sensor &s : g_sensors) {
        if (auto res = s.imu->enable_fifo(); !res) { halt(s.name, res.error()); }
    }

    send_header();
}

void loop() {
    static unsigned long last_header_ms = millis();
    static unsigned long last_temp_ms   = millis();

    for (std::uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        poll(i, g_sensors[i]);
    }

    const unsigned long now_ms = millis();
    if (now_ms - last_temp_ms >= TEMP_PERIOD_MS) {
        last_temp_ms = now_ms;
        for (std::uint8_t i = 0; i < SENSOR_COUNT; ++i) {
            if (auto t = g_sensors[i].imu->get_temperature_c(); t) {
                capture::write_temp(g_writer, i, micros(), t.value());
                send(g_writer.finish());
            }
        }
    }
    if (now_ms - last_header_ms >= HEADER_PERIOD_MS) {
        last_header_ms = now_ms;
        send_header();
    }
}
