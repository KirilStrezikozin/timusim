import struct

import numpy as np

from timusim import capture


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_pos, code = 0, 1
    for byte in data:
        if byte:
            out.append(byte)
            code += 1
        if not byte or code == 0xFF:
            out[code_pos] = code
            code_pos, code = len(out), 1
            out.append(0)
    out[code_pos] = code
    return bytes(out)


def frame(payload: bytes) -> bytes:
    return cobs_encode(payload + capture.crc16(payload).to_bytes(2, "little")) + b"\x00"


def samples(sensor: int, first: int, t_us: int, codes: np.ndarray, overrun: bool = False) -> bytes:
    body = struct.pack("<BBIIBB", capture.PacketType.SAMPLES, sensor, first, t_us, int(overrun), len(codes))
    return frame(body + codes.astype("<i2").tobytes())


def header(text: str) -> bytes:
    return frame(bytes([capture.PacketType.HEADER]) + text.encode())


def test_crc16_check_value() -> None:
    assert capture.crc16(b"123456789") == 0x29B1


def test_cobs_round_trip_with_zero_runs() -> None:
    data = bytes([0, 1, 0, 0, 2]) + bytes(range(1, 255)) * 2 + bytes([0])
    assert capture.cobs_decode(cobs_encode(data)) == data


def test_stream_with_gap_and_bad_frame() -> None:
    codes = np.arange(4 * 6, dtype=np.int16).reshape(4, 6) - 10
    raw = (
        header("s0.name=palm\ns0.chip=LSM9DS1\ns0.odr_hz=476\ns0.accel_lsb_g=0.000122\ns0.gyro_lsb_dps=0.07\n")
        + samples(0, 0, 1000, codes)
        + b"\x05\x01\x02\x00"
        + samples(0, 4, 9400, codes)
        + samples(0, 10, 22000, codes, overrun=True)
    )
    cap = capture.parse_capture(raw)
    s = cap.sensors[0]

    assert cap.bad_frames == 1
    assert s.name == "palm" and s.chip == "LSM9DS1"
    assert len(s.index) == 12
    assert s.gaps.missing_samples == 2 and s.gaps.missing_ranges == [(8, 10)]
    assert s.gaps.overruns == 1
    np.testing.assert_allclose(s.accel_g[0], codes[0, :3] * 0.000122)
    np.testing.assert_allclose(s.gyro_dps[0], codes[0, 3:] * 0.07)


def test_restart_keeps_last_session() -> None:
    codes = np.zeros((2, 6), dtype=np.int16)
    raw = samples(1, 100, 0, codes) + samples(1, 0, 5000, codes) + samples(1, 2, 9000, codes)
    cap = capture.parse_capture(raw)
    assert cap.restarts == 1
    assert list(cap.sensors[1].index) == [0, 1, 2, 3]


def test_longest_run_skips_gaps_and_mask() -> None:
    import numpy as np

    s = capture.SensorStream(
        id=0,
        header={},
        index=np.array([0, 1, 2, 5, 6, 7, 8, 9, 20, 21]),
        t_s=np.arange(10) * 0.1,
        accel_g=np.zeros((10, 3)),
        gyro_dps=np.zeros((10, 3)),
        accel_codes=np.zeros((10, 3), np.int16),
        gyro_codes=np.zeros((10, 3), np.int16),
        odr_hz=10.0,
        gaps=capture.Gaps(),
    )
    assert s.longest_run() == slice(3, 8)
    mask = np.ones(10, bool)
    mask[5] = False
    assert s.longest_run(mask) == slice(0, 3)
