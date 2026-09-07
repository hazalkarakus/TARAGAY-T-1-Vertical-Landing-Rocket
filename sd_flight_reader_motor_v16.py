
# -*- coding: utf-8 -*-
"""
TARAGAY-T1 SD kart flight.bin okuyucu
-------------------------------------
Bu dosya TARAGAY-T1'in mevcut SD logger formatlarini okur:

- V14 normal 384-byte frame (marker 0x1419)
- V15 / R3R10 eski compact flight replay (marker 0xF10A)
- V16 motor-ayrintili compact flight replay (marker 0xF10B)
- CRC16-CCITT kontrolu yapar
- V14 icin state + fast IMU CSV olusturur
- V15/V16 replay icin ucus/control CSV olusturur
- Tum motor/igne-vana verilerini tek flight_motor.csv dosyasinda birlestirir
- Ozet TXT olusturur

Harici kutuphane gerekmez. Python 3 yeterlidir.

KULLANIM
========
1) SD karttaki flight.bin dosyasini belirt:
   python sd_flight_reader.py E:\\flight.bin

veya flight.bin script ile ayni klasordeyse:
   python sd_flight_reader.py

Ciktilar:
   sd_flight_output/
       flight_v14_state.csv
       flight_v14_fast_imu.csv
       flight_v15_replay.csv
       flight_v16_replay.csv
       flight_motor.csv
       flight_summary.txt
"""

from __future__ import annotations

import binascii
import csv
import struct
import sys
from pathlib import Path

FRAME_SIZE = 384
MAGIC = 0x54475931          # "TGY1"
V14_VERSION = 14
V14_MARKER = 0x1419
V15_VERSION = 15
V15_MARKER = 0xF10A
V16_VERSION = 16
V16_MARKER = 0xF10B

DEFAULT_ADC_PER_TURN = 195.0
DEFAULT_MAX_TURNS = 2.0
CRC_OFFSET = 382

FAST_IMU_OFFSET = 288
FAST_IMU_COUNT = 5
FAST_IMU_STRIDE = 16
META_OFFSET = 368

ACCEL_SCALE_G = 0.000244
GYRO_SCALE_DPS = 0.035


def u8(b: bytes, o: int) -> int:
    return b[o]


def u16(b: bytes, o: int) -> int:
    return struct.unpack_from("<H", b, o)[0]


def i16(b: bytes, o: int) -> int:
    return struct.unpack_from("<h", b, o)[0]


def u32(b: bytes, o: int) -> int:
    return struct.unpack_from("<I", b, o)[0]


def i32(b: bytes, o: int) -> int:
    return struct.unpack_from("<i", b, o)[0]


def crc_ok(frame: bytes) -> bool:
    return binascii.crc_hqx(frame[:CRC_OFFSET], 0xFFFF) == u16(frame, CRC_OFFSET)



def motor_derived(needle_adc: int, zero_adc: int, target_adc: int,
                  error_adc: int, rpwm: int, lpwm: int, flags: int,
                  adc_per_turn: float = DEFAULT_ADC_PER_TURN,
                  max_turns: float = DEFAULT_MAX_TURNS) -> dict:
    zero_valid = bool(flags & 0x02)
    turns = 0.0
    target_turns = 0.0
    moved_adc = 0
    if zero_valid and zero_adc >= needle_adc and adc_per_turn > 0:
        moved_adc = zero_adc - needle_adc
        turns = min(max_turns, moved_adc / adc_per_turn)
    if zero_valid and zero_adc >= target_adc and adc_per_turn > 0:
        target_turns = min(max_turns, (zero_adc - target_adc) / adc_per_turn)

    if lpwm > 0 and rpwm == 0:
        direction = "OPEN"
    elif rpwm > 0 and lpwm == 0:
        direction = "CLOSE"
    elif rpwm == 0 and lpwm == 0:
        direction = "STOP"
    else:
        direction = "INVALID_BOTH"

    return {
        "needle_position_turns": round(turns, 5),
        "needle_target_turns": round(target_turns, 5),
        "needle_moved_adc": moved_adc,
        "needle_position_pct": round((turns / max_turns) * 100.0, 3) if max_turns > 0 else 0.0,
        "needle_error_turns": round(error_adc / adc_per_turn, 5) if adc_per_turn > 0 else 0.0,
        "needle_active_pwm": max(rpwm, lpwm),
        "needle_direction": direction,
        "needle_homing_active": int(bool(flags & 0x08)),
        "needle_homing_complete": int(bool(flags & 0x10)),
        "needle_motor_active": int(bool(flags & 0x20)),
        "needle_opening": int(bool(flags & 0x40)),
        "needle_closing": int(bool(flags & 0x80)),
    }


def v16_motor_ext(f: bytes) -> dict:
    o = 148
    return {
        "needle_position_turns_direct": u16(f, o + 0) / 10000.0,
        "needle_max_turns": u16(f, o + 2) / 10000.0,
        "needle_max_open_adc": u16(f, o + 4),
        "needle_adc_per_turn": u16(f, o + 6),
        "needle_max_travel_adc": u16(f, o + 8),
        "needle_homing_start_adc": u16(f, o + 10),
        "needle_hw_adc_raw12": u16(f, o + 12),
        "needle_hw_adc_mv": u16(f, o + 14),
        "needle_stall_ms": u32(f, o + 16),
        "needle_homing_elapsed_ms": u32(f, o + 20),
        "needle_control_tick_count": u32(f, o + 24),
        "needle_adc_invalid_count": u32(f, o + 28),
        "p83_adc1_raw": u16(f, o + 32),
        "p83_adc2_raw": u16(f, o + 34),
        "p83_pair_diff": u16(f, o + 36),
        "p83_pair_candidate": u16(f, o + 38),
        "p83_median7": u16(f, o + 40),
        "p83_filtered_adc": u16(f, o + 42),
        "p83_feedback_valid": u8(f, o + 44),
        "p83_confidence_pct": u8(f, o + 45),
        "p83_mode": u8(f, o + 46),
        "p83_acq_progress_pct": u8(f, o + 47),
        "p83_pair_reject_count": u32(f, o + 48),
        "p83_rate_reject_count": u32(f, o + 52),
        "p83_quarantine_count": u32(f, o + 56),
        "p83_reacquire_count": u32(f, o + 60),
        "p83_adc_timeout_count": u32(f, o + 64),
        "p110_start_adc": u16(f, o + 68),
        "p110_target_adc": u16(f, o + 70),
        "p110_current_adc": u16(f, o + 72),
        "p110_error_adc": i16(f, o + 74),
        "p110_speed_adc_s": u16(f, o + 76),
        "p110_stop_distance_adc": u16(f, o + 78),
        "p110_brake_entry_adc": u16(f, o + 80),
        "p110_final_adc": u16(f, o + 82),
        "p110_final_error_adc": i16(f, o + 84),
        "p110_total_powered_ms": u16(f, o + 86),
        "p110_state": u8(f, o + 88),
        "p110_result": u8(f, o + 89),
        "p110_direction": u8(f, o + 90),
        "p110_active_pwm": u8(f, o + 91),
        "p110_abort_reason": u8(f, o + 92),
        "p111_state": u8(f, o + 93),
        "p111_result": u8(f, o + 94),
    }

def find_flight_bin() -> Path | None:
    candidates: list[Path] = []

    # Once script ile ayni klasor / mevcut klasor.
    for p in (
        Path.cwd() / "flight.bin",
        Path(__file__).resolve().parent / "flight.bin",
    ):
        if p.exists() and p.is_file() and p not in candidates:
            candidates.append(p)

    # Windows'ta SD kart koklerini tara.
    if sys.platform.startswith("win"):
        for letter in "DEFGHIJKLMNOPQRSTUVWXYZ":
            root = Path(f"{letter}:\\")
            for name in ("flight.bin", "FLIGHT.BIN"):
                p = root / name
                try:
                    if p.exists() and p.is_file() and p not in candidates:
                        candidates.append(p)
                except OSError:
                    pass

    if len(candidates) == 1:
        return candidates[0]

    if len(candidates) > 1:
        print("Birden fazla flight.bin bulundu:")
        for p in candidates:
            print("  ", p)
        print("\nDosya yolunu elle ver:")
        print(r'  python sd_flight_reader.py E:\flight.bin')
        return None

    return None


def v14_state_row(f: bytes) -> dict:
    needle_flags = u8(f, 268)
    fast_count = u8(f, META_OFFSET + 0)
    fast_valid_mask = u8(f, META_OFFSET + 1)
    qual_flags = u16(f, META_OFFSET + 10) | (u16(f, META_OFFSET + 12) << 16)

    row = {
        "frame_type": "V14_NORMAL",
        "sequence": u32(f, 4),
        "time_us": u32(f, 8),
        "time_ms": u32(f, 12),
        "time_s": u32(f, 8) / 1e6,

        "pressure_pa": i32(f, 16),
        "baro_altitude_m": i32(f, 20) / 100.0,
        "sensor_update_count": u32(f, 24),
        "baro_update_count": u32(f, 28),
        "lidar_update_count": u32(f, 32),

        "accel_x_raw": i16(f, 40),
        "accel_y_raw": i16(f, 42),
        "accel_z_raw": i16(f, 44),
        "gyro_x_raw": i16(f, 46),
        "gyro_y_raw": i16(f, 48),
        "gyro_z_raw": i16(f, 50),
        "temperature_c": i16(f, 52) / 100.0,
        "lidar_raw_m": u16(f, 54) / 100.0,
        "cpu_pct": u16(f, 56) / 100.0,
        "health_flags": u8(f, 58),
        "fault_code": u8(f, 59),

        "accel_x_filtered_raw_eq": i16(f, 60),
        "accel_y_filtered_raw_eq": i16(f, 62),
        "accel_z_filtered_raw_eq": i16(f, 64),
        "gyro_x_filtered_raw_eq": i16(f, 66),
        "gyro_y_filtered_raw_eq": i16(f, 68),
        "gyro_z_filtered_raw_eq": i16(f, 70),
        "accel_filtered_norm_mg": u16(f, 72),
        "filter_flags": u8(f, 74),
        "sensor_filter_flags": u8(f, 75),
        "filter_reset_count": u16(f, 76),
        "baro_median_delta_pa": i16(f, 78),
        "filtered_pressure_pa": i32(f, 80),
        "filtered_altitude_m": i32(f, 84) / 1000.0,
        "vertical_speed_mps": i16(f, 88) / 100.0,
        "lidar_median_m": u16(f, 90) / 1000.0,
        "lidar_filtered_m": u16(f, 92) / 1000.0,

        "eskf_z_m": i32(f, 96) / 1000.0,
        "eskf_vz_mps": i32(f, 100) / 1000.0,
        "eskf_x_m": i16(f, 112) / 100.0,
        "eskf_y_m": i16(f, 114) / 100.0,
        "eskf_vx_mps": i16(f, 116) / 100.0,
        "eskf_vy_mps": i16(f, 118) / 100.0,

        "q_w": i16(f, 126) / 32767.0,
        "q_x": i16(f, 128) / 32767.0,
        "q_y": i16(f, 130) / 32767.0,
        "q_z": i16(f, 132) / 32767.0,
        "roll_deg": i16(f, 134) / 100.0,
        "pitch_deg": i16(f, 136) / 100.0,
        "yaw_deg": i16(f, 138) / 100.0,
        "world_accel_x_mps2": i16(f, 140) / 100.0,
        "world_accel_y_mps2": i16(f, 142) / 100.0,
        "world_accel_z_mps2": i16(f, 144) / 100.0,
        "eskf_flags": u8(f, 110),
        "eskf_attitude_flags": u8(f, 146),

        "imu_sample_valid": u8(f, 228),
        "imu_recovery_state": u8(f, 229),
        "rcs_imu_inhibit": u8(f, 230),
        "imu_diag_flags": u8(f, 231),
        "imu_stale_count": u32(f, 232),
        "imu_pattern_error_count": u32(f, 236),
        "imu_dma_timeout_count": u32(f, 240),
        "imu_recovery_count": u32(f, 244),
        "imu_register_error_count": u32(f, 248),

        "needle_requested": u16(f, 254) / 10000.0,
        "needle_limited": u16(f, 256) / 10000.0,
        "needle_adc": u16(f, 258),
        "needle_zero_adc": u16(f, 260),
        "needle_target_adc": u16(f, 262),
        "needle_error_adc": i16(f, 264),
        "needle_rpwm": u8(f, 266),
        "needle_lpwm": u8(f, 267),
        "needle_enabled": int(bool(needle_flags & 0x01)),
        "needle_zero_valid": int(bool(needle_flags & 0x02)),
        "needle_lock": int(bool(needle_flags & 0x04)),
        "needle_fault": u8(f, 269),

        "nrf_link_active": u8(f, 270),
        "nrf_command": u8(f, 271),
        "nrf_valid_packet_count": u32(f, 272),
        "nrf_last_sequence": u8(f, 276),
        "servo_target_open": u8(f, 277),
        "servo_pulse_us": u16(f, 278),
        "nrf_last_packet_age_ms": u16(f, 280),
        "nrf_irq_count_low": u16(f, 282),
        "nrf_rx_count_low": u16(f, 284),

        "fast_imu_sample_count": fast_count,
        "fast_imu_valid_mask": fast_valid_mask,
        "imu_age_us": u16(f, 370),
        "baro_age_us": u16(f, 372),
        "lidar_age_us": u16(f, 374),
        "sensor_qual_state": u8(f, 376),
        "sensor_qual_good_windows": u8(f, 377),
        "sensor_qual_flags": qual_flags,
        "needle_stall_ms": u16(f, 94),
        "needle_homing_elapsed_ms": u16(f, 252),
        "crc16": u16(f, 382),
    }
    row.update(motor_derived(
        row["needle_adc"], row["needle_zero_adc"], row["needle_target_adc"],
        row["needle_error_adc"], row["needle_rpwm"], row["needle_lpwm"], needle_flags
    ))
    return row


def v15_replay_row(f: bytes) -> dict:
    # R3R10 V15 replay layout; mevcut firmware decoder'i ile ayni offsetler.
    flags_low = u16(f, 378)
    flags_high = u16(f, 380)

    return {
        "frame_type": "V15_FLIGHT_REPLAY",
        "sequence": u32(f, 4),
        "time_us": u32(f, 8),
        "time_ms": u32(f, 12),
        "time_s": u32(f, 12) / 1000.0,
        "pressure_pa": i32(f, 16),
        "baro_altitude_m": i32(f, 20) / 100.0,
        "lidar_m": u16(f, 92) / 1000.0,

        "eskf_x_m": i16(f, 112) / 100.0,
        "eskf_y_m": i16(f, 114) / 100.0,
        "eskf_z_m": i32(f, 96) / 1000.0,
        "eskf_vx_mps": i16(f, 116) / 100.0,
        "eskf_vy_mps": i16(f, 118) / 100.0,
        "eskf_vz_mps": i32(f, 100) / 1000.0,

        "roll_deg": i16(f, 134) / 100.0,
        "pitch_deg": i16(f, 136) / 100.0,
        "yaw_deg": i16(f, 138) / 100.0,
        "pitch_rate_dps": i16(f, 76) / 100.0,
        "yaw_rate_dps": i16(f, 78) / 100.0,

        "q_w": i16(f, 126) / 32767.0,
        "q_x": i16(f, 128) / 32767.0,
        "q_y": i16(f, 130) / 32767.0,
        "q_z": i16(f, 132) / 32767.0,

        "cpu_pct": u16(f, 56) / 100.0,
        "health_flags": u8(f, 58),
        "fault_code": u8(f, 59),

        "valve_cmd": u16(f, 252) / 10000.0,
        "needle_requested": u16(f, 254) / 10000.0,
        "needle_limited": u16(f, 256) / 10000.0,
        "needle_adc": u16(f, 258),
        "needle_target_adc": u16(f, 262),
        "needle_error_adc": i16(f, 264),
        "needle_rpwm": u8(f, 266),
        "needle_lpwm": u8(f, 267),
        "needle_state_flags": u8(f, 268),
        "needle_fault": u8(f, 269),

        "imu_age_us": u16(f, 370),
        "baro_age_us": u16(f, 372),
        "lidar_age_us": u16(f, 374),

        "mission_state": u8(f, 376),
        "rcs_requested_mask": u8(f, 377),
        "rcs_applied_mask": flags_low & 0xFF,
        "flight_input_valid": (flags_low >> 8) & 0xFF,
        "flight_rcs_fault": flags_high & 0xFF,
        "mount_cal_valid": (flags_high >> 8) & 0xFF,
        "eskf_flags": u8(f, 110),
        "eskf_attitude_flags": u8(f, 146),
        "crc16": u16(f, 382),
    }


def v16_replay_row(f: bytes) -> dict:
    flags_low = u16(f, 378)
    flags_high = u16(f, 380)
    needle_flags = u8(f, 268)
    ext = v16_motor_ext(f)

    row = {
        "frame_type": "V16_FLIGHT_MOTOR_REPLAY",
        "sequence": u32(f, 4),
        "time_us": u32(f, 8),
        "time_ms": u32(f, 12),
        "time_s": u32(f, 12) / 1000.0,
        "pressure_pa": i32(f, 16),
        "baro_altitude_m": i32(f, 20) / 100.0,
        "lidar_m": u16(f, 92) / 1000.0,
        "eskf_x_m": i16(f, 112) / 100.0,
        "eskf_y_m": i16(f, 114) / 100.0,
        "eskf_z_m": i32(f, 96) / 1000.0,
        "eskf_vx_mps": i16(f, 116) / 100.0,
        "eskf_vy_mps": i16(f, 118) / 100.0,
        "eskf_vz_mps": i32(f, 100) / 1000.0,
        "roll_deg": i16(f, 134) / 100.0,
        "pitch_deg": i16(f, 136) / 100.0,
        "yaw_deg": i16(f, 138) / 100.0,
        "pitch_rate_dps": i16(f, 76) / 100.0,
        "yaw_rate_dps": i16(f, 78) / 100.0,
        "q_w": i16(f, 126) / 32767.0,
        "q_x": i16(f, 128) / 32767.0,
        "q_y": i16(f, 130) / 32767.0,
        "q_z": i16(f, 132) / 32767.0,
        "cpu_pct": u16(f, 56) / 100.0,
        "health_flags": u8(f, 58),
        "fault_code": u8(f, 59),
        "valve_cmd": u16(f, 252) / 10000.0,
        "needle_requested": u16(f, 254) / 10000.0,
        "needle_limited": u16(f, 256) / 10000.0,
        "needle_adc": u16(f, 258),
        "needle_zero_adc": u16(f, 260),
        "needle_target_adc": u16(f, 262),
        "needle_error_adc": i16(f, 264),
        "needle_rpwm": u8(f, 266),
        "needle_lpwm": u8(f, 267),
        "needle_state_flags": needle_flags,
        "needle_enabled": int(bool(needle_flags & 0x01)),
        "needle_zero_valid": int(bool(needle_flags & 0x02)),
        "needle_lock": int(bool(needle_flags & 0x04)),
        "needle_fault": u8(f, 269),
        "imu_age_us": u16(f, 370),
        "baro_age_us": u16(f, 372),
        "lidar_age_us": u16(f, 374),
        "mission_state": u8(f, 376),
        "rcs_requested_mask": u8(f, 377),
        "rcs_applied_mask": flags_low & 0xFF,
        "flight_input_valid": (flags_low >> 8) & 0xFF,
        "flight_rcs_fault": flags_high & 0xFF,
        "mount_cal_valid": (flags_high >> 8) & 0xFF,
        "eskf_flags": u8(f, 110),
        "eskf_attitude_flags": u8(f, 146),
        "crc16": u16(f, 382),
    }
    row.update(ext)
    adc_per_turn = float(ext["needle_adc_per_turn"] or DEFAULT_ADC_PER_TURN)
    max_turns = float(ext["needle_max_turns"] or DEFAULT_MAX_TURNS)
    row.update(motor_derived(
        row["needle_adc"], row["needle_zero_adc"], row["needle_target_adc"],
        row["needle_error_adc"], row["needle_rpwm"], row["needle_lpwm"],
        needle_flags, adc_per_turn, max_turns
    ))
    return row


def extract_fast_imu(f: bytes, parent_sequence: int) -> list[dict]:
    rows: list[dict] = []
    count = min(u8(f, META_OFFSET), FAST_IMU_COUNT)
    valid_mask = u8(f, META_OFFSET + 1)

    for index in range(count):
        if (valid_mask & (1 << index)) == 0:
            continue

        p = FAST_IMU_OFFSET + index * FAST_IMU_STRIDE
        ts = u32(f, p)
        ax = i16(f, p + 4)
        ay = i16(f, p + 6)
        az = i16(f, p + 8)
        gx = i16(f, p + 10)
        gy = i16(f, p + 12)
        gz = i16(f, p + 14)

        rows.append({
            "parent_sequence": parent_sequence,
            "slot": index,
            "time_us": ts,
            "time_s": ts / 1e6,
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
        })
    return rows


def motor_row(row: dict) -> dict:
    keep = [
        "frame_type", "sequence", "time_us", "time_ms", "time_s",
        "valve_cmd", "needle_requested", "needle_limited",
        "needle_adc", "needle_zero_adc", "needle_target_adc", "needle_error_adc",
        "needle_position_turns", "needle_position_turns_direct", "needle_target_turns",
        "needle_moved_adc", "needle_position_pct", "needle_error_turns",
        "motor_motion_reference_adc", "motor_motion_delta_adc",
        "motor_motion_turns_est", "motor_feedback_adc",
        "needle_rpwm", "needle_lpwm", "needle_active_pwm", "needle_direction",
        "needle_enabled", "needle_zero_valid", "needle_lock",
        "needle_homing_active", "needle_homing_complete", "needle_motor_active",
        "needle_opening", "needle_closing", "needle_fault",
        "needle_max_turns", "needle_max_open_adc", "needle_adc_per_turn",
        "needle_max_travel_adc", "needle_homing_start_adc",
        "needle_stall_ms", "needle_homing_elapsed_ms",
        "needle_control_tick_count", "needle_adc_invalid_count",
        "needle_hw_adc_raw12", "needle_hw_adc_mv",
        "p83_adc1_raw", "p83_adc2_raw", "p83_pair_diff", "p83_pair_candidate",
        "p83_median7", "p83_filtered_adc", "p83_feedback_valid",
        "p83_confidence_pct", "p83_mode", "p83_acq_progress_pct",
        "p83_pair_reject_count", "p83_rate_reject_count", "p83_quarantine_count",
        "p83_reacquire_count", "p83_adc_timeout_count",
        "p110_start_adc", "p110_target_adc", "p110_current_adc", "p110_error_adc",
        "p110_speed_adc_s", "p110_stop_distance_adc", "p110_brake_entry_adc",
        "p110_final_adc", "p110_final_error_adc", "p110_total_powered_ms",
        "p110_state", "p110_result", "p110_direction", "p110_active_pwm",
        "p110_abort_reason", "p111_state", "p111_result",
        "mission_state", "flight_input_valid", "health_flags", "fault_code",
    ]
    return {k: row.get(k, "") for k in keep}


def annotate_motor_motion(rows: list[dict]) -> None:
    """Add a relative movement estimate that does not require zero_valid.

    V16 already stores the controller's direct turn estimate plus P83/P110
    diagnostics.  This helper additionally tracks ADC movement from the start
    of each powered/homing move, so a motor motion is still visible even if a
    zero reference has not been accepted yet.
    """
    reference_adc = None
    previous_active = False

    for row in rows:
        p83 = row.get("p83_filtered_adc", "")
        p110_current = row.get("p110_current_adc", "")
        needle_adc = row.get("needle_adc", "")

        feedback_adc = None
        for candidate in (p83, p110_current, needle_adc):
            try:
                value = int(candidate)
            except (TypeError, ValueError):
                continue
            if value > 0:
                feedback_adc = value
                break

        try:
            p110_state = int(row.get("p110_state", 0) or 0)
        except (TypeError, ValueError):
            p110_state = 0
        try:
            rpwm = int(row.get("needle_rpwm", 0) or 0)
            lpwm = int(row.get("needle_lpwm", 0) or 0)
            motor_flag = int(row.get("needle_motor_active", 0) or 0)
            homing = int(row.get("needle_homing_active", 0) or 0)
        except (TypeError, ValueError):
            rpwm = lpwm = motor_flag = homing = 0

        active = bool(rpwm or lpwm or motor_flag or homing or (1 <= p110_state <= 4))

        if active and not previous_active:
            try:
                p110_start = int(row.get("p110_start_adc", 0) or 0)
            except (TypeError, ValueError):
                p110_start = 0
            reference_adc = p110_start if p110_start > 0 else feedback_adc

        if reference_adc is None and feedback_adc is not None:
            reference_adc = feedback_adc

        try:
            adc_per_turn = float(row.get("needle_adc_per_turn", 0) or 0)
        except (TypeError, ValueError):
            adc_per_turn = 0.0
        if adc_per_turn <= 0.0:
            adc_per_turn = DEFAULT_ADC_PER_TURN

        delta_adc = 0
        turns_est = 0.0
        if (reference_adc is not None) and (feedback_adc is not None):
            delta_adc = feedback_adc - reference_adc
            turns_est = abs(delta_adc) / adc_per_turn

        row["motor_motion_reference_adc"] = "" if reference_adc is None else reference_adc
        row["motor_motion_delta_adc"] = delta_adc
        row["motor_motion_turns_est"] = round(turns_est, 5)
        row["motor_feedback_adc"] = "" if feedback_adc is None else feedback_adc

        previous_active = active


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8-sig") as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def duration_seconds(rows: list[dict]) -> float:
    if len(rows) < 2:
        return 0.0
    a = int(rows[0]["time_ms"])
    b = int(rows[-1]["time_ms"])
    return ((b - a) & 0xFFFFFFFF) / 1000.0


def main() -> int:
    if len(sys.argv) >= 2:
        bin_path = Path(sys.argv[1]).expanduser()
    else:
        bin_path = find_flight_bin()
        if bin_path is None:
            print("flight.bin bulunamadi.")
            print(r"Kullanim: python sd_flight_reader.py E:\flight.bin")
            return 2

    if not bin_path.exists():
        print(f"Dosya bulunamadi: {bin_path}")
        return 2

    print(f"\nBIN : {bin_path.resolve()}")
    print(f"Boyut: {bin_path.stat().st_size:,} byte")
    print("Okunuyor...\n")

    data = bin_path.read_bytes()

    output_dir = Path.cwd() / "sd_flight_output"
    output_dir.mkdir(parents=True, exist_ok=True)

    v14_rows: list[dict] = []
    v15_rows: list[dict] = []
    v16_rows: list[dict] = []
    imu_rows: list[dict] = []

    bad_crc = 0
    bad_magic = 0
    unknown_valid = 0
    zero_tail = 0

    consecutive_bad_magic = 0

    for off in range(0, len(data) - FRAME_SIZE + 1, FRAME_SIZE):
        f = data[off:off + FRAME_SIZE]

        if u32(f, 0) != MAGIC:
            bad_magic += 1
            consecutive_bad_magic += 1

            # Preallocated dosyada logger, son gercek veriden sonraki ilk
            # yazilmamis frame bolgesini sifir guard ile gecersiz kilar.
            # Bu noktadan SONRA kartta onceki testlerden kalmis, CRC'si hala
            # gecerli eski V14 frame'ler bulunabilir. Onlari yeni oturuma
            # eklememek icin ilk tam-sifir guard frame'inde okumayi bitir.
            if f == bytes(FRAME_SIZE):
                zero_tail += 1
                if v14_rows or v15_rows or v16_rows:
                    break

            # Hic gecerli frame bulunmadan uzun bos alan gelirse gereksiz
            # taramayi kes.
            if consecutive_bad_magic >= 64:
                break
            continue

        consecutive_bad_magic = 0

        if not crc_ok(f):
            bad_crc += 1
            continue

        version = u16(f, 36)
        frame_size = u16(f, 38)
        marker = u16(f, 286)

        if frame_size != FRAME_SIZE:
            unknown_valid += 1
            continue

        if version == V14_VERSION and marker == V14_MARKER:
            row = v14_state_row(f)
            v14_rows.append(row)
            imu_rows.extend(extract_fast_imu(f, row["sequence"]))

        elif version == V15_VERSION and marker == V15_MARKER:
            v15_rows.append(v15_replay_row(f))

        elif version == V16_VERSION and marker == V16_MARKER:
            v16_rows.append(v16_replay_row(f))

        else:
            unknown_valid += 1

    # Fast IMU tekrarlarini temizle.
    dedup_imu: list[dict] = []
    seen = set()
    for row in imu_rows:
        key = (
            row["time_us"],
            row["accel_x_raw"], row["accel_y_raw"], row["accel_z_raw"],
            row["gyro_x_raw"], row["gyro_y_raw"], row["gyro_z_raw"],
        )
        if key in seen:
            continue
        seen.add(key)
        dedup_imu.append(row)

    dedup_imu.sort(key=lambda r: int(r["time_us"]))

    v14_path = output_dir / "flight_v14_state.csv"
    imu_path = output_dir / "flight_v14_fast_imu.csv"
    v15_path = output_dir / "flight_v15_replay.csv"
    v16_path = output_dir / "flight_v16_replay.csv"
    motor_path = output_dir / "flight_motor.csv"
    summary_path = output_dir / "flight_summary.txt"

    motor_rows = [motor_row(r) for r in (v14_rows + v15_rows + v16_rows)]
    motor_rows.sort(key=lambda r: (float(r.get("time_s") or 0.0), int(r.get("sequence") or 0)))
    annotate_motor_motion(motor_rows)

    write_csv(v14_path, v14_rows)
    write_csv(imu_path, dedup_imu)
    write_csv(v15_path, v15_rows)
    write_csv(v16_path, v16_rows)
    write_csv(motor_path, motor_rows)

    summary_lines = [
        "TARAGAY-T1 SD FLIGHT.BIN OZETI",
        "=" * 40,
        f"Kaynak dosya          : {bin_path.resolve()}",
        f"Dosya boyutu          : {len(data):,} byte",
        f"V14 normal frame      : {len(v14_rows)}",
        f"V15 flight replay     : {len(v15_rows)}",
        f"V16 motor replay      : {len(v16_rows)}",
        f"Motor CSV satiri      : {len(motor_rows)}",
        f"Fast IMU sample       : {len(dedup_imu)}",
        f"CRC hatali frame      : {bad_crc}",
        f"Magic/header disi     : {bad_magic}",
        f"Bilinmeyen valid frame: {unknown_valid}",
        f"Bos/prealloc tail     : {zero_tail}",
    ]

    if v14_rows:
        d = duration_seconds(v14_rows)
        summary_lines.append(f"V14 sure              : {d:.3f} s")
        if d > 0:
            summary_lines.append(f"V14 ortalama rate     : {(len(v14_rows)-1)/d:.2f} Hz")

        ages = [int(r["nrf_last_packet_age_ms"]) for r in v14_rows]
        links = [int(r["nrf_link_active"]) for r in v14_rows]
        summary_lines.append(f"NRF link=0 satiri     : {sum(v == 0 for v in links)}")
        summary_lines.append(f"NRF max packet age    : {max(ages)} ms")
        summary_lines.append(
            f"NRF son valid paket   : {int(v14_rows[-1]['nrf_valid_packet_count'])}"
        )

    if v15_rows:
        d = duration_seconds(v15_rows)
        summary_lines.append(f"V15 flight sure       : {d:.3f} s")
        if d > 0:
            summary_lines.append(f"V15 ortalama rate     : {(len(v15_rows)-1)/d:.2f} Hz")
        summary_lines.append(
            f"Needle fault satiri   : {sum(int(r['needle_fault']) != 0 for r in v15_rows)}"
        )
        summary_lines.append(
            f"RCS fault satiri      : {sum(int(r['flight_rcs_fault']) != 0 for r in v15_rows)}"
        )

    if v16_rows:
        d = duration_seconds(v16_rows)
        summary_lines.append(f"V16 flight sure       : {d:.3f} s")
        if d > 0:
            summary_lines.append(f"V16 ortalama rate     : {(len(v16_rows)-1)/d:.2f} Hz")
        summary_lines.append(
            f"Motor fault satiri    : {sum(int(r['needle_fault']) != 0 for r in v16_rows)}"
        )
        summary_lines.append(
            f"ADC invalid son sayac : {int(v16_rows[-1]['needle_adc_invalid_count'])}"
        )

    if motor_rows:
        feedback_values = []
        active_rows = []
        movement_turns = []
        for r in motor_rows:
            try:
                fv = int(r.get("motor_feedback_adc", ""))
                feedback_values.append(fv)
            except (TypeError, ValueError):
                pass
            try:
                is_active = (
                    int(r.get("needle_active_pwm", 0) or 0) > 0 or
                    int(r.get("needle_motor_active", 0) or 0) != 0 or
                    1 <= int(r.get("p110_state", 0) or 0) <= 4
                )
            except (TypeError, ValueError):
                is_active = False
            if is_active:
                active_rows.append(r)
            try:
                movement_turns.append(float(r.get("motor_motion_turns_est", 0) or 0))
            except (TypeError, ValueError):
                pass

        if feedback_values:
            summary_lines.append(
                f"Motor feedback ADC     : {min(feedback_values)}..{max(feedback_values)}"
            )
        summary_lines.append(f"Motor aktif kayit     : {len(active_rows)}")
        if movement_turns:
            summary_lines.append(
                f"Motor rel. max hareket: {max(movement_turns):.3f} tur"
            )

    summary_lines += [
        "",
        "CIKTILAR",
        "=" * 40,
        str(v14_path.resolve()) if v14_rows else "V14 CSV: veri yok",
        str(imu_path.resolve()) if dedup_imu else "Fast IMU CSV: veri yok",
        str(v15_path.resolve()) if v15_rows else "V15 replay CSV: veri yok",
        str(v16_path.resolve()) if v16_rows else "V16 replay CSV: veri yok",
        str(motor_path.resolve()) if motor_rows else "Motor CSV: veri yok",
        str(summary_path.resolve()),
        "",
        "Not: V15/V16 compact flight replay formatinda ayrintili NRF age/count alanlari",
        "saklanmiyor. NRF ayrintisi V14 normal frame veya UART telemetrisinden okunur.",
    ]

    summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")

    print("\n".join(summary_lines))

    if not v14_rows and not v15_rows and not v16_rows:
        print("\nUYARI: Gecerli TARAGAY flight frame bulunamadi.")
        print("Yanlis flight.bin, eksik flush veya farkli logger formati olabilir.")
        return 1

    print("\nTAMAM. CSV dosyalari 'sd_flight_output' klasorune yazildi.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())