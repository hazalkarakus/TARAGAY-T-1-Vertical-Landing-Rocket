#!/usr/bin/env python3
"""
TARAGAY-T1 P54 flight.bin V14 decoder

V14:
- 384-byte state frame @ 200 Hz
- 5 x raw IMU samples embedded per frame (~1 kHz evidence)
- CRC16-CCITT over bytes 0..381

Usage:
    python decode_flight_v14.py flight.bin

Outputs next to the BIN:
    flight_v14_state.csv
    flight_v14_fast_imu.csv
"""

from __future__ import annotations

import csv
import struct
import sys
from pathlib import Path

MAGIC = 0x54475931
FORMAT_VERSION = 14
FRAME_SIZE = 384
CRC_OFFSET = 382

ACCEL_SCALE_G = 0.000244
GYRO_SCALE_DPS = 0.035

FAST_IMU_OFFSET = 288
FAST_IMU_COUNT = 5
FAST_IMU_STRIDE = 16

EXT_MARKER_OFFSET = 286
EXT_MARKER = 0x1419

META_OFFSET = 368


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def u32(buf: bytes, off: int) -> int:
    return struct.unpack_from("<I", buf, off)[0]


def i32(buf: bytes, off: int) -> int:
    return struct.unpack_from("<i", buf, off)[0]


def u16(buf: bytes, off: int) -> int:
    return struct.unpack_from("<H", buf, off)[0]


def i16(buf: bytes, off: int) -> int:
    return struct.unpack_from("<h", buf, off)[0]


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: python decode_flight_v14.py flight.bin")
        return 2

    bin_path = Path(sys.argv[1])
    data = bin_path.read_bytes()

    state_path = bin_path.with_name("flight_v14_state.csv")
    imu_path = bin_path.with_name("flight_v14_fast_imu.csv")

    valid_frames = 0
    bad_crc = 0
    bad_header = 0

    state_rows = []
    imu_rows = []

    expected_sequence = None
    sequence_gaps = 0
    session_boundary_detected = False
    session_boundary_offset = None
    previous_timestamp_us = None
    previous_sensor_count = None
    previous_baro_count = None
    previous_lidar_count = None

    def u32_went_backwards(current: int, previous: int) -> bool:
        # Modular forward motion, including a genuine 32-bit wrap, produces a
        # delta < 2^31. A preallocated old-session tail produces a small
        # numerical rollback and therefore a huge modular delta.
        return ((current - previous) & 0xFFFFFFFF) > 0x80000000

    for off in range(0, len(data) - FRAME_SIZE + 1, FRAME_SIZE):
        frame = data[off:off + FRAME_SIZE]

        magic = u32(frame, 0)
        version = u16(frame, 36)
        frame_size = u16(frame, 38)
        marker = u16(frame, EXT_MARKER_OFFSET)

        if (
            magic != MAGIC
            or version != FORMAT_VERSION
            or frame_size != FRAME_SIZE
            or marker != EXT_MARKER
        ):
            bad_header += 1
            continue

        stored_crc = u16(frame, CRC_OFFSET)
        calculated_crc = crc16_ccitt(frame[:CRC_OFFSET])

        if stored_crc != calculated_crc:
            bad_crc += 1
            continue

        seq = u32(frame, 4)
        timestamp_us = u32(frame, 8)
        sensor_count = u32(frame, 24)
        baro_count = u32(frame, 28)
        lidar_count = u32(frame, 32)

        # P40: flight.bin is preallocated. If power is removed before the final
        # truncate, CRC-valid frames from an older session may remain after the
        # current session. Stop at the first unambiguous rollback instead of
        # silently concatenating two boots into one CSV. Timestamp rollback is
        # sufficient; otherwise require at least two independent source
        # counters to move backwards to avoid false boundaries on one driver.
        if previous_timestamp_us is not None:
            counter_rollbacks = sum((
                u32_went_backwards(sensor_count, previous_sensor_count),
                u32_went_backwards(baro_count, previous_baro_count),
                u32_went_backwards(lidar_count, previous_lidar_count),
            ))
            if (u32_went_backwards(timestamp_us, previous_timestamp_us) or
                    counter_rollbacks >= 2):
                session_boundary_detected = True
                session_boundary_offset = off
                break

        previous_timestamp_us = timestamp_us
        previous_sensor_count = sensor_count
        previous_baro_count = baro_count
        previous_lidar_count = lidar_count

        if expected_sequence is not None and seq != expected_sequence:
            if seq > expected_sequence:
                sequence_gaps += seq - expected_sequence
            else:
                sequence_gaps += 1
        expected_sequence = (seq + 1) & 0xFFFFFFFF

        fast_count = frame[META_OFFSET + 0]
        fast_valid_mask = frame[META_OFFSET + 1]
        imu_age_us = u16(frame, META_OFFSET + 2)
        baro_age_us = u16(frame, META_OFFSET + 4)
        lidar_age_us = u16(frame, META_OFFSET + 6)
        qual_state = frame[META_OFFSET + 8]
        qual_good_windows = frame[META_OFFSET + 9]
        qual_flags = (
            u16(frame, META_OFFSET + 10)
            | (u16(frame, META_OFFSET + 12) << 16)
        )

        state_rows.append(
            {
                "sequence": seq,
                "timestamp_us": timestamp_us,
                "timestamp_ms": u32(frame, 12),
                "pressure_pa": i32(frame, 16),
                "baro_altitude_cm": i32(frame, 20),
                "sensor_update_count": sensor_count,
                "baro_update_count": baro_count,
                "lidar_update_count": lidar_count,
                "accel_x_raw_latest": i16(frame, 40),
                "accel_y_raw_latest": i16(frame, 42),
                "accel_z_raw_latest": i16(frame, 44),
                "gyro_x_raw_latest": i16(frame, 46),
                "gyro_y_raw_latest": i16(frame, 48),
                "gyro_z_raw_latest": i16(frame, 50),
                "temperature_c": i16(frame, 52) / 100.0,
                "lidar_distance_m": u16(frame, 54) / 100.0,
                "cpu_load_percent": u16(frame, 56) / 100.0,
                "health_flags": frame[58],
                "fault_code": frame[59],
                "fast_imu_sample_count": fast_count,
                "fast_imu_valid_mask": fast_valid_mask,
                "imu_sample_age_us": imu_age_us,
                "baro_sample_age_us": baro_age_us,
                "lidar_sample_age_us": lidar_age_us,
                "sensor_qual_state": qual_state,
                "sensor_qual_good_windows": qual_good_windows,
                "sensor_qual_flags": qual_flags,
                "crc16": stored_crc,
            }
        )

        # Embedded 1 kHz raw IMU history, chronological oldest -> newest.
        for index in range(min(fast_count, FAST_IMU_COUNT)):
            if (fast_valid_mask & (1 << index)) == 0:
                continue

            p = FAST_IMU_OFFSET + index * FAST_IMU_STRIDE
            imu_ts = u32(frame, p + 0)
            ax = i16(frame, p + 4)
            ay = i16(frame, p + 6)
            az = i16(frame, p + 8)
            gx = i16(frame, p + 10)
            gy = i16(frame, p + 12)
            gz = i16(frame, p + 14)

            imu_rows.append(
                {
                    "parent_sequence": seq,
                    "slot": index,
                    "timestamp_us": imu_ts,
                    "accel_x_raw": ax,
                    "accel_y_raw": ay,
                    "accel_z_raw": az,
                    "gyro_x_raw": gx,
                    "gyro_y_raw": gy,
                    "gyro_z_raw": gz,
                    "accel_x_g": ax * ACCEL_SCALE_G,
                    "accel_y_g": ay * ACCEL_SCALE_G,
                    "accel_z_g": az * ACCEL_SCALE_G,
                    "gyro_x_dps": gx * GYRO_SCALE_DPS,
                    "gyro_y_dps": gy * GYRO_SCALE_DPS,
                    "gyro_z_dps": gz * GYRO_SCALE_DPS,
                }
            )

        valid_frames += 1

    if state_rows:
        with state_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=state_rows[0].keys())
            writer.writeheader()
            writer.writerows(state_rows)

    # Rolling 5-sample windows overlap by design if timer phase differs.
    # Deduplicate the fast stream by timestamp + raw tuple before CSV output.
    deduped_imu_rows = []
    seen = set()
    for row in imu_rows:
        key = (
            row["timestamp_us"],
            row["accel_x_raw"], row["accel_y_raw"], row["accel_z_raw"],
            row["gyro_x_raw"], row["gyro_y_raw"], row["gyro_z_raw"],
        )
        if key in seen:
            continue
        seen.add(key)
        deduped_imu_rows.append(row)

    deduped_imu_rows.sort(key=lambda r: r["timestamp_us"])

    if deduped_imu_rows:
        with imu_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=deduped_imu_rows[0].keys())
            writer.writeheader()
            writer.writerows(deduped_imu_rows)

    print(f"Valid frames     : {valid_frames}")
    print(f"Bad CRC          : {bad_crc}")
    print(f"Bad/empty header : {bad_header}")
    print(f"Sequence gaps    : {sequence_gaps}")
    if session_boundary_detected:
        print(f"Session boundary : byte {session_boundary_offset} (old preallocated tail ignored)")
    else:
        print("Session boundary : none")
    print(f"Fast IMU samples : {len(deduped_imu_rows)}")

    if state_rows:
        duration_s = (
            (state_rows[-1]["timestamp_us"] - state_rows[0]["timestamp_us"])
            & 0xFFFFFFFF
        ) * 1e-6
        if duration_s > 0:
            print(f"State rate       : {(len(state_rows)-1)/duration_s:.2f} Hz")

    if len(deduped_imu_rows) > 1:
        duration_s = (
            (deduped_imu_rows[-1]["timestamp_us"]
             - deduped_imu_rows[0]["timestamp_us"])
            & 0xFFFFFFFF
        ) * 1e-6
        if duration_s > 0:
            print(
                f"Fast IMU rate    : "
                f"{(len(deduped_imu_rows)-1)/duration_s:.2f} Hz"
            )

    if state_rows:
        print(f"Wrote            : {state_path}")
    if deduped_imu_rows:
        print(f"Wrote            : {imu_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
