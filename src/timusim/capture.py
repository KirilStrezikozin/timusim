"""Reader for the capture stream defined in lib/capture/include/capture/protocol.hpp."""

import array
import binascii
import dataclasses
import enum
import pathlib
import struct
import typing

import numpy as np

SAMPLE_DTYPE = np.dtype("<i2")
SAMPLE_WORDS = 6


class PacketType(enum.IntEnum):
    TEXT = 1
    HEADER = 2
    SAMPLES = 3
    TEMP = 4


def crc16(data: bytes) -> int:
    """CRC16-CCITT-FALSE: crc_hqx is the same polynomial, unreflected, from 0xFFFF."""
    return binascii.crc_hqx(data, 0xFFFF)


def cobs_decode(frame: bytes) -> bytes | None:
    out = bytearray()
    i = 0
    while i < len(frame):
        code = frame[i]
        if code == 0 or i + code > len(frame):
            return None
        out += frame[i + 1 : i + code]
        i += code
        if code != 0xFF and i < len(frame):
            out.append(0)
    return bytes(out)


def iter_payloads(raw: bytes) -> typing.Iterator[bytes | None]:
    """Yields each frame's payload, or None for a frame that fails COBS or CRC."""
    start = 0
    while start < len(raw):
        end = raw.find(b"\x00", start)
        if end < 0:
            end = len(raw)
        frame, start = raw[start:end], end + 1
        if not frame:
            continue
        decoded = cobs_decode(frame)
        if decoded is None or len(decoded) < 3:
            yield None
            continue
        payload, crc = decoded[:-2], int.from_bytes(decoded[-2:], "little")
        yield payload if crc16(payload) == crc else None


class Batches:
    """One sensor's FIFO batches, kept as flat arrays so hour-long captures fit in memory."""

    def __init__(self) -> None:
        self.first_index = array.array("q")
        self.t_us = array.array("q")
        self.overrun = array.array("b")
        self.count = array.array("q")
        self.data = bytearray()

    def append(self, first_index: int, t_us: int, overrun: bool, data: bytes) -> None:
        self.first_index.append(first_index)
        self.t_us.append(t_us)
        self.overrun.append(overrun)
        self.count.append(len(data) // (SAMPLE_WORDS * SAMPLE_DTYPE.itemsize))
        self.data += data


@dataclasses.dataclass
class Gaps:
    overruns: int = 0
    missing_samples: int = 0
    missing_ranges: list[tuple[int, int]] = dataclasses.field(default_factory=list)

    @property
    def clean(self) -> bool:
        return self.overruns == 0 and self.missing_samples == 0


@dataclasses.dataclass
class SensorStream:
    id: int
    header: dict[str, str]
    index: np.ndarray
    t_s: np.ndarray
    accel_g: np.ndarray
    gyro_dps: np.ndarray
    accel_codes: np.ndarray
    gyro_codes: np.ndarray
    odr_hz: float
    gaps: Gaps
    # Board time of the first sample; t_s starts at 0, so t_s + t0_s lines sensors up.
    t0_s: float = 0.0

    @property
    def name(self) -> str:
        return self.header.get("name", f"s{self.id}")

    @property
    def chip(self) -> str:
        return self.header.get("chip", "")

    @property
    def duration_s(self) -> float:
        return float(self.t_s[-1] - self.t_s[0]) if len(self.t_s) > 1 else 0.0

    def longest_run(self, mask: np.ndarray | None = None) -> slice:
        """The longest stretch of consecutive sample indices, within mask if given."""
        ok = np.ones(len(self.index), bool) if mask is None else mask
        breaks = np.concatenate([[True], (np.diff(self.index) != 1) | ~ok[1:] | ~ok[:-1], [True]])
        starts = np.flatnonzero(breaks[:-1])
        ends = np.append(starts[1:], len(self.index))
        lengths = np.where(ok[starts], ends - starts, 0)
        k = int(np.argmax(lengths))
        return slice(int(starts[k]), int(ends[k]))


@dataclasses.dataclass
class Capture:
    header: dict[str, str]
    sensors: dict[int, SensorStream]
    temps: dict[int, tuple[np.ndarray, np.ndarray]]
    texts: list[str]
    bad_frames: int
    restarts: int

    def stream(self, name: str) -> SensorStream:
        """The stream of the sensor with this name (palm, finger, palm_lsm9)."""
        found = next((s for s in self.sensors.values() if s.name == name), None)
        if found is None:
            raise KeyError(f"no sensor {name!r} in the capture")
        return found


def _parse_header(text: str, header: dict[str, str]) -> None:
    for line in text.splitlines():
        key, sep, value = line.partition("=")
        if sep:
            header[key.strip()] = value.strip()


def _unwrap_us(t_us: np.ndarray) -> np.ndarray:
    wraps = np.concatenate([[0], np.cumsum(np.diff(t_us) < -(1 << 31))])
    return t_us.astype(np.int64) + wraps * (1 << 32)


def _build_stream(sensor: int, batches: Batches, header: dict[str, str]) -> SensorStream:
    prefix = f"s{sensor}."
    meta = {k[len(prefix) :]: v for k, v in header.items() if k.startswith(prefix)}
    accel_lsb = float(meta.get("accel_lsb_g", "nan"))
    gyro_lsb = float(meta.get("gyro_lsb_dps", "nan"))

    first = np.frombuffer(batches.first_index, np.int64)
    count = np.frombuffer(batches.count, np.int64)
    gaps = Gaps(overruns=int(np.frombuffer(batches.overrun, np.int8).sum()))
    skipped = first[1:] - (first[:-1] + count[:-1])
    for k in np.flatnonzero(skipped > 0):
        gaps.missing_samples += int(skipped[k])
        gaps.missing_ranges.append((int(first[k] + count[k]), int(first[k + 1])))

    codes = np.frombuffer(batches.data, SAMPLE_DTYPE).reshape(-1, SAMPLE_WORDS)
    nonempty = count > 0
    index = np.repeat(first[nonempty] - np.cumsum(count[nonempty]) + count[nonempty], count[nonempty])
    index += np.arange(len(index))

    # Each batch is read just after its last sample; a line through those points gives
    # the chip's actual ODR and a timestamp for every sample.
    odr_hz = float(meta.get("odr_hz", "nan"))
    t_s = np.empty(0)
    t0_s = 0.0
    if nonempty.sum() >= 2:
        last_index = (first + count - 1)[nonempty].astype(np.float64)
        read_s = _unwrap_us(np.frombuffer(batches.t_us, np.int64)[nonempty]) * 1e-6
        slope, intercept = np.polyfit(last_index, read_s, 1)
        odr_hz = 1.0 / slope
        t_s = index * slope + intercept
        t0_s = float(t_s[0])
        t_s -= t0_s

    return SensorStream(
        id=sensor,
        header=meta,
        index=index,
        t_s=t_s,
        accel_g=codes[:, :3].astype(np.float64) * accel_lsb,
        gyro_dps=codes[:, 3:].astype(np.float64) * gyro_lsb,
        accel_codes=codes[:, :3],
        gyro_codes=codes[:, 3:],
        odr_hz=odr_hz,
        gaps=gaps,
        t0_s=t0_s,
    )


def parse_capture(raw: bytes) -> Capture:
    """Parses a raw stream. Only the part after the last board restart is kept."""
    header: dict[str, str] = {}
    batches: dict[int, Batches] = {}
    temps: dict[int, list[tuple[int, float]]] = {}
    texts: list[str] = []
    bad_frames = 0
    restarts = 0

    for payload in iter_payloads(raw):
        if payload is None:
            bad_frames += 1
            continue
        kind, body = payload[0], payload[1:]
        if kind == PacketType.TEXT:
            text = body.decode("utf-8", errors="replace")
            texts.append(text)
            if "Starting up" in text:
                restarts += int(bool(batches))
                batches.clear()
                temps.clear()
        elif kind == PacketType.HEADER:
            _parse_header(body.decode("utf-8", errors="replace"), header)
        elif kind == PacketType.SAMPLES and len(body) >= 11:
            sensor, first_index, t_us, flags, n = struct.unpack_from("<BIIBB", body)
            data = body[11 : 11 + n * SAMPLE_WORDS * 2]
            if len(data) != n * SAMPLE_WORDS * 2:
                bad_frames += 1
                continue
            stream = batches.setdefault(sensor, Batches())
            if stream.first_index and first_index < stream.first_index[-1]:
                restarts += 1
                batches.clear()
                temps.clear()
                stream = batches.setdefault(sensor, Batches())
            stream.append(first_index, t_us, bool(flags & 1), data)
        elif kind == PacketType.TEMP and len(body) >= 9:
            sensor, t_us, celsius = struct.unpack_from("<BIf", body)
            temps.setdefault(sensor, []).append((t_us, celsius))

    sensors = {s: _build_stream(s, b, header) for s, b in sorted(batches.items())}
    temp_arrays = {
        s: (_unwrap_us(np.array([t for t, _ in v], dtype=np.int64)) * 1e-6, np.array([c for _, c in v]))
        for s, v in temps.items()
    }
    return Capture(
        header=header,
        sensors=sensors,
        temps=temp_arrays,
        texts=texts,
        bad_frames=bad_frames,
        restarts=restarts,
    )


def read_capture(path: pathlib.Path | str) -> Capture:
    return parse_capture(pathlib.Path(path).read_bytes())
