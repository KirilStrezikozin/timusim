/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cstdio>

#include "capture/protocol.hpp"
#include "imu/lsm6dsl/i2c.hpp"
#include "imu/lsm9ds1/i2c.hpp"

namespace {

int g_failures = 0;

void check(bool ok, const char *what) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) { ++g_failures; }
}

/* Inverse of cobs_encode, for the round trip. */
std::size_t cobs_decode(const std::uint8_t *src, std::size_t len, std::uint8_t *dst) {
    std::size_t out = 0;
    for (std::size_t i = 0; i < len;) {
        const std::uint8_t code = src[i++];
        for (std::uint8_t k = 1; k < code; ++k) { dst[out++] = src[i++]; }
        if (code != 0xFF && i < len) { dst[out++] = 0; }
    }
    return out;
}

} // namespace

int main() {
    const char check_input[] = "123456789";
    check(capture::crc16(reinterpret_cast<const std::uint8_t *>(check_input), 9) == 0x29B1,
          "CRC-16/CCITT-FALSE check value");

    imu::IIMU::RawSample samples[capture::MAX_BATCH] = {};
    for (std::size_t i = 0; i < capture::MAX_BATCH; ++i) {
        for (int k = 0; k < 3; ++k) {
            samples[i].accel[k] = static_cast<std::int16_t>(i * 7 - k * 300);
            samples[i].gyro[k]  = static_cast<std::int16_t>(k == 0 ? 0 : -32768 + static_cast<int>(i));
        }
    }

    capture::PacketWriter writer;
    capture::write_samples(writer, 1, 123456, 0xDEADBEEF, true, {samples, capture::MAX_BATCH});
    const auto frame = writer.finish();
    check(frame.size() > 0, "full batch fits in one frame");
    check(frame.data()[frame.size() - 1] == 0, "frame ends with the delimiter");
    bool inner_zero = false;
    for (std::size_t i = 0; i + 1 < frame.size(); ++i) { inner_zero = inner_zero || frame.data()[i] == 0; }
    check(!inner_zero, "no zero byte inside the frame");

    std::uint8_t payload[capture::MAX_PAYLOAD + 2];
    const std::size_t n = cobs_decode(frame.data(), frame.size() - 1, payload);
    const std::uint16_t crc = static_cast<std::uint16_t>(payload[n - 2] | (payload[n - 1] << 8));
    check(crc == capture::crc16(payload, n - 2), "CRC matches after decoding");
    check(payload[0] == static_cast<std::uint8_t>(capture::PacketType::SAMPLES) && payload[1] == 1
              && payload[10] == 1 && payload[11] == capture::MAX_BATCH,
          "header fields decode");

    std::printf("%s (%d failed checks)\n", g_failures == 0 ? "ALL CHECKS PASSED" : "CHECKS FAILED",
                g_failures);
    return g_failures == 0 ? 0 : 1;
}
