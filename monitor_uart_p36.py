#!/usr/bin/env python3
"""Validate and monitor TGY V55 USART2 diagnostic frames.

Examples:
  python monitor_uart_v55.py --port COM5
  python monitor_uart_v55.py --port /dev/ttyUSB0
  python monitor_uart_v55.py uart_capture.txt
  type uart_capture.txt | python monitor_uart_v55.py -
"""

from __future__ import annotations

import argparse
import binascii
import json
import sys
from pathlib import Path
from typing import Iterable


FIELDS = (
    "frame", "seq", "time_ms", "preflight_state", "flight_active",
    "pe9_raw_open", "pe9_debounced_open", "preflight_ready",
    "preflight_fault", "flight_time_ms", "connector_seen",
    "actuator_authorized", "imu_valid", "imu_age_ms",
    "imu_recovery_state", "imu_recovery_step", "imu_recovery_count",
    "imu_recovery_attempts", "imu_recovery_failures",
    "imu_recovery_last_us", "imu_recovery_max_us", "imu_stale_count",
    "imu_pattern_errors", "imu_invalid_samples", "imu_redundant_rejects",
    "baro_valid", "lidar_valid", "eskf_init", "eskf_ok", "eskf_public_age_ms",
    "eskf_public_count", "baro_fresh", "lidar_fresh",
    "vertical_source_mask", "vertical_degraded", "eskf_z_mm",
    "eskf_vz_mmps", "baro_alt_mm", "lidar_mm", "baro_pressure_pa",
    "baro_innov_mm", "lidar_innov_mm", "baro_updates", "lidar_updates",
    "baro_rejects", "lidar_rejects", "lidar_state", "lidar_age_ms",
    "lidar_reads", "lidar_errors", "lidar_timeouts",
    "lidar_wait_timeouts", "lidar_recoveries", "roll_cdeg", "pitch_cdeg",
    "roll_rate_mdps", "pitch_rate_mdps", "rcs_state", "rcs_fault",
    "rcs_req_mask", "rcs_applied_mask", "rcs_roll_pd_us",
    "rcs_pitch_pd_us", "rcs_roll_pulse_rem_ms",
    "rcs_pitch_pulse_rem_ms", "rcs_roll_cooldown_rem_ms",
    "rcs_pitch_cooldown_rem_ms", "rcs_autodamp_mask", "rcs_dry_run",
    "rcs_event_count", "rcs_flight_authorized", "rcs_safety_inhibited",
    "rcs_inhibit_count", "main_cmd_x10000", "main_target_force_cN",
    "main_est_thrust_cN", "main_burn_active", "main_output_valid",
    "main_physical_enabled", "needle_req_x10000",
    "needle_limited_x10000", "needle_adc", "needle_target_adc",
    "needle_enabled", "needle_lock", "needle_fault", "needle_zero_adc",
    "needle_error_adc", "needle_rpwm", "needle_lpwm", "needle_zero_valid",
    "needle_homing_active", "needle_homing_complete", "needle_stall_ms",
    "nrf_connected",
    "nrf_link", "nrf_cmd", "nrf_flags", "nrf_age_ms", "nrf_rx_count",
    "nrf_errors", "nrf_invalid", "nrf_irq", "nrf_status", "nrf_config",
    "nrf_channel", "nrf_rf_setup", "nrf_fifo", "stop_latched",
    "sd_initialized", "sd_mount_ok", "sd_file_open", "sd_ready",
    "sd_logging", "sd_last_result", "sd_disk_status", "sd_mount_retries",
    "sd_errors", "sd_write_errors", "sd_dropped", "sd_ring_overruns",
    "sd_async_timeouts", "sd_frames", "sd_source_publishes", "sd_hal_init",
    "sd_wide", "sd_host_attempts", "sd_host_resets", "sd_hal_error",
    "sd_last_hal_error", "sd_recovered",
    "cpu_x100", "cpu_bg_x100", "sd_update_us", "sd_update_max_us",
    "sd_drain_us", "sd_drain_max_us", "sd_drain_yields", "cov_us",
    "cov_max_us", "cov_count", "sched_slow_defers", "bg_fresh_us",
    "bg_remote_us", "bg_control_us", "bg_uart_us", "bg_uart_max_us",
    "sd_defers", "control_defers", "uart_defers",
    "cpu_imu_x100", "cpu_baro_x100", "cpu_lidar_x100",
    "cpu_nrf_x100", "cpu_eskf_x100", "task_imu_us", "task_baro_us",
    "task_lidar_us", "task_nrf_us", "task_eskf_us", "task_imu_max_us",
    "task_baro_max_us", "task_lidar_max_us", "task_nrf_max_us",
    "task_eskf_max_us", "miss_imu", "miss_baro", "miss_lidar",
    "miss_nrf", "miss_eskf", "system_ok", "system_fault",
    "system_imu_fresh", "system_lidar_fresh", "system_eskf_fresh",
    "scheduler_realigns", "uart_dma_errors", "uart_busy_skips",
)


def decode_frame(line: str) -> dict[str, str]:
    line = line.strip()
    if not line.startswith("$TGY55,"):
        raise ValueError("not a $TGY55 frame")
    if "*" not in line:
        raise ValueError("CRC separator is missing")

    body, crc_text = line.rsplit("*", 1)
    if len(crc_text) != 4:
        raise ValueError("CRC must contain four hex digits")

    received_crc = int(crc_text, 16)
    calculated_crc = binascii.crc_hqx(body.encode("ascii"), 0xFFFF)
    if received_crc != calculated_crc:
        raise ValueError(
            f"CRC mismatch: received={received_crc:04X} "
            f"calculated={calculated_crc:04X}"
        )

    values = body.split(",")
    if len(values) != len(FIELDS):
        raise ValueError(
            f"field count mismatch: received={len(values)} "
            f"expected={len(FIELDS)}"
        )

    return dict(zip(FIELDS, values))


def serial_lines(port: str, baud: int) -> Iterable[str]:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Serial monitoring requires pyserial: pip install pyserial"
        ) from exc

    with serial.Serial(port, baudrate=baud, timeout=1.0) as connection:
        while True:
            raw = connection.readline()
            if raw:
                yield raw.decode("ascii", errors="replace")


def input_lines(path: str | None) -> Iterable[str]:
    if path in (None, "-"):
        yield from sys.stdin
        return

    with Path(path).open("r", encoding="ascii", errors="replace") as source:
        yield from source


def compact_status(frame: dict[str, str]) -> str:
    return (
        f"seq={frame['seq']} t={frame['time_ms']}ms "
        f"state={frame['preflight_state']} pfault={frame['preflight_fault']} "
        f"flight={frame['flight_active']} PE9open={frame['pe9_debounced_open']} "
        f"seen={frame['connector_seen']} auth={frame['actuator_authorized']} "
        f"src={frame['vertical_source_mask']} "
        f"fresh(B/L)={frame['baro_fresh']}/{frame['lidar_fresh']} "
        f"age(I/E/L)={frame['imu_age_ms']}/{frame['eskf_public_age_ms']}/"
        f"{frame['lidar_age_ms']}ms "
        f"health(I/L/E)={frame['system_imu_fresh']}/"
        f"{frame['system_lidar_fresh']}/{frame['system_eskf_fresh']} "
        f"z={int(frame['eskf_z_mm']) / 1000.0:+.3f}m "
        f"vz={int(frame['eskf_vz_mmps']) / 1000.0:+.3f}m/s "
        f"RCSreq/app={frame['rcs_req_mask']}/{frame['rcs_applied_mask']} "
        f"events={frame['rcs_event_count']} "
        f"IMUrec={frame['imu_recovery_state']}/{frame['imu_recovery_step']}/"
        f"{frame['imu_recovery_count']} "
        f"Lrecover={frame['lidar_recoveries']} "
        f"realign={frame['scheduler_realigns']} "
        f"main={int(frame['main_cmd_x10000']) / 10000.0:.4f} "
        f"needleADC={frame['needle_adc']} PWM={frame['needle_lpwm']}/{frame['needle_rpwm']} "
        f"needleFault={frame['needle_fault']} "
        f"NRF={frame['nrf_link']} STOP={frame['stop_latched']} "
        f"SD={frame['sd_ready']}/{frame['sd_logging']} "
        f"CPU={int(frame['cpu_x100']) / 100.0:.1f}% "
        f"BG={int(frame['cpu_bg_x100']) / 100.0:.1f}% "
        f"cov={frame['cov_us']}us/{frame['cov_count']} "
        f"SDupd={frame['sd_update_us']}us "
        f"CPU(I/B/L/N/E)={int(frame['cpu_imu_x100'])/100.0:.1f}/"
        f"{int(frame['cpu_baro_x100'])/100.0:.1f}/"
        f"{int(frame['cpu_lidar_x100'])/100.0:.1f}/"
        f"{int(frame['cpu_nrf_x100'])/100.0:.1f}/"
        f"{int(frame['cpu_eskf_x100'])/100.0:.1f}% "
        f"UARTerr={frame['uart_dma_errors']} skip={frame['uart_busy_skips']}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", nargs="?", help="capture file or '-' for stdin")
    parser.add_argument("--port", help="serial port, for example COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--json", action="store_true", help="print every valid frame as JSON")
    args = parser.parse_args()

    lines = serial_lines(args.port, args.baud) if args.port else input_lines(args.capture)
    valid = 0
    rejected = 0

    try:
        for line in lines:
            if not line.startswith("$TGY55,"):
                if line.startswith("#"):
                    print(line.rstrip())
                continue
            try:
                frame = decode_frame(line)
            except (ValueError, UnicodeError) as exc:
                rejected += 1
                print(f"REJECT[{rejected}]: {exc}", file=sys.stderr)
                continue

            valid += 1
            print(json.dumps(frame, sort_keys=True) if args.json else compact_status(frame))
    except KeyboardInterrupt:
        pass

    print(f"valid={valid} rejected={rejected}", file=sys.stderr)
    return 0 if rejected == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
