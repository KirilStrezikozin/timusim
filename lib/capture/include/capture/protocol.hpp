/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Capture stream format, shared by the capture firmware and the simulator.
 * src/timusim/capture.py reads it.
 *
 * Frame: COBS(payload, CRC-16/CCITT-FALSE of payload as little-endian u16) + 0x00.
 * payload[0] is the packet type; all integers are little-endian.
 *
 * TEXT    = 1: UTF-8 log text.
 * HEADER  = 2: UTF-8 "key=value" lines describing the capture.
 * SAMPLES = 3: u8 sensor, u32 first sample index, u32 t_us (when read), u8 flags
 *              (bit 0: chip FIFO overrun before this batch), u8 n,
 *              n x (i16 ax ay az gx gy gz) in output codes, IIMU frame.
 * TEMP    = 4: u8 sensor, u32 t_us, f32 die temperature in deg C.
 */

#ifndef CAPTURE_PROTOCOL_HPP
#define CAPTURE_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "imu/iimu.hpp"
#include "span.hpp"

namespace capture {

enum class PacketType : std::uint8_t {
    TEXT    = 1,
    HEADER  = 2,
    SAMPLES = 3,
    TEMP    = 4,
};

inline constexpr std::size_t MAX_BATCH = 32;
inline constexpr std::size_t MAX_PAYLOAD =
    1 + 1 + 4 + 4 + 1 + 1 + MAX_BATCH * sizeof(imu::IIMU::RawSample);

[[nodiscard]] constexpr std::uint16_t crc16(const std::uint8_t *data, std::size_t len) noexcept {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i] << 8);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0 ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021U)
                                       : static_cast<std::uint16_t>(crc << 1);
        }
    }
    return crc;
}

/* Worst case output length of cobs_encode() for len input bytes, without the 0x00 delimiter. */
[[nodiscard]] constexpr std::size_t cobs_max_size(std::size_t len) noexcept { return len + len / 254 + 1; }

/* Returns the encoded length; dst must hold cobs_max_size(len) bytes. */
inline std::size_t cobs_encode(const std::uint8_t *src, std::size_t len, std::uint8_t *dst) noexcept {
    std::size_t out      = 1;
    std::size_t code_pos = 0;
    std::uint8_t code    = 1;
    for (std::size_t i = 0; i < len; ++i) {
        if (src[i] != 0) {
            dst[out++] = src[i];
            ++code;
        }
        if (src[i] == 0 || code == 0xFF) {
            dst[code_pos] = code;
            code          = 1;
            code_pos      = out++;
        }
    }
    dst[code_pos] = code;
    return out;
}

/* Builds one payload and frames it. Sink is called with the finished frame. */
class PacketWriter {
  public:
    using frame_type = span<const std::uint8_t>;

    void begin(PacketType type) noexcept {
        this->len_        = 0;
        this->overflowed_ = false;
        this->put_u8(static_cast<std::uint8_t>(type));
    }

    void put_u8(std::uint8_t v) noexcept { this->put_(&v, 1); }

    void put_u32(std::uint32_t v) noexcept {
        const std::uint8_t b[4] = {
            static_cast<std::uint8_t>(v), static_cast<std::uint8_t>(v >> 8),
            static_cast<std::uint8_t>(v >> 16), static_cast<std::uint8_t>(v >> 24)
        };
        this->put_(b, sizeof(b));
    }

    void put_i16(std::int16_t v) noexcept {
        const auto u           = static_cast<std::uint16_t>(v);
        const std::uint8_t b[2] = {static_cast<std::uint8_t>(u), static_cast<std::uint8_t>(u >> 8)};
        this->put_(b, sizeof(b));
    }

    void put_f32(float v) noexcept {
        std::uint32_t u = 0;
        std::memcpy(&u, &v, sizeof(u));
        this->put_u32(u);
    }

    void put_bytes(const void *data, std::size_t len) noexcept {
        this->put_(static_cast<const std::uint8_t *>(data), len);
    }

    /* Appends the CRC, COBS-encodes and terminates the frame. Empty if the payload overflowed. */
    [[nodiscard]] frame_type finish() noexcept {
        if (this->overflowed_) { return {this->frame_, 0}; }
        const std::uint16_t crc = crc16(this->payload_, this->len_);
        this->payload_[this->len_++] = static_cast<std::uint8_t>(crc);
        this->payload_[this->len_++] = static_cast<std::uint8_t>(crc >> 8);
        std::size_t n      = cobs_encode(this->payload_, this->len_, this->frame_);
        this->frame_[n++]  = 0;
        return {this->frame_, n};
    }

  private:
    static constexpr std::size_t PAYLOAD_CAPACITY = MAX_PAYLOAD + 2;

    std::uint8_t payload_[PAYLOAD_CAPACITY] = {};
    std::uint8_t frame_[cobs_max_size(PAYLOAD_CAPACITY) + 1] = {};
    std::size_t len_  = 0;
    bool overflowed_  = false;

    void put_(const std::uint8_t *data, std::size_t len) noexcept {
        if (this->len_ + len > MAX_PAYLOAD) {
            this->overflowed_ = true;
            return;
        }
        std::memcpy(this->payload_ + this->len_, data, len);
        this->len_ += len;
    }
};

inline void write_samples(
    PacketWriter &w, std::uint8_t sensor, std::uint32_t first_index, std::uint32_t t_us, bool overrun,
    span<const imu::IIMU::RawSample> samples
) noexcept {
    w.begin(PacketType::SAMPLES);
    w.put_u8(sensor);
    w.put_u32(first_index);
    w.put_u32(t_us);
    w.put_u8(overrun ? 1 : 0);
    w.put_u8(static_cast<std::uint8_t>(samples.size()));
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto &s = samples.data()[i];
        for (int k = 0; k < 3; ++k) { w.put_i16(s.accel[k]); }
        for (int k = 0; k < 3; ++k) { w.put_i16(s.gyro[k]); }
    }
}

inline void write_temp(PacketWriter &w, std::uint8_t sensor, std::uint32_t t_us, float celsius) noexcept {
    w.begin(PacketType::TEMP);
    w.put_u8(sensor);
    w.put_u32(t_us);
    w.put_f32(celsius);
}

inline void write_text(PacketWriter &w, PacketType type, span<const char> text) noexcept {
    w.begin(type);
    w.put_bytes(text.data(), text.size());
}

} /* namespace capture */

#endif /* CAPTURE_PROTOCOL_HPP */
