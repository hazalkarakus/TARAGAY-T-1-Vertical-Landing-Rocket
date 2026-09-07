#!/usr/bin/env python3
"""Monitor TARAGAY-T1 P39 $TGY55 UART telemetry (212 fields)."""

from __future__ import annotations

import argparse
import binascii
import json
import sys
from pathlib import Path
from typing import Iterable

FIELDS = (
    'frame', 'seq', 'time_ms', 'preflight_state', 'flight_active', 'pe9_raw_open',
    'pe9_debounced_open', 'preflight_ready', 'preflight_fault', 'flight_time_ms',
    'connector_seen', 'actuator_authorized', 'imu_valid', 'imu_age_ms', 'imu_recovery_state',
    'imu_recovery_step', 'imu_recovery_count', 'imu_recovery_attempts',
    'imu_recovery_failures', 'imu_recovery_last_us', 'imu_recovery_max_us', 'imu_stale_count',
    'imu_pattern_errors', 'imu_invalid_samples', 'imu_redundant_rejects', 'baro_valid',
    'lidar_valid', 'eskf_init', 'eskf_ok', 'eskf_inhibit', 'eskf_reacquire', 'eskf_div_reason',
    'eskf_div_count', 'eskf_reacq_count', 'eskf_div_candidate', 'eskf_reacq_stable',
    'vertical_consistency_mm', 'vertical_reacq_target_mm', 'eskf_public_age_ms',
    'eskf_public_count', 'baro_fresh', 'lidar_fresh', 'vertical_source_mask',
    'vertical_degraded', 'eskf_z_mm', 'eskf_vz_mmps', 'baro_alt_mm', 'lidar_mm',
    'baro_pressure_pa', 'baro_innov_mm', 'lidar_innov_mm', 'baro_updates', 'lidar_updates',
    'baro_rejects', 'lidar_rejects', 'lidar_state', 'lidar_age_ms', 'lidar_reads',
    'lidar_errors', 'lidar_timeouts', 'lidar_wait_timeouts', 'lidar_recoveries',
    'lidar_rec_active', 'lidar_rec_step', 'lidar_rec_attempts', 'lidar_rec_success',
    'lidar_rec_failures', 'lidar_rec_last_us', 'lidar_rec_max_us', 'lidar_rec_step_max_us',
    'roll_cdeg', 'pitch_cdeg', 'roll_rate_mdps', 'pitch_rate_mdps', 'rcs_state', 'rcs_fault',
    'rcs_req_mask', 'rcs_applied_mask', 'rcs_roll_pd_us', 'rcs_pitch_pd_us',
    'rcs_roll_pulse_rem_ms', 'rcs_pitch_pulse_rem_ms', 'rcs_roll_cooldown_rem_ms',
    'rcs_pitch_cooldown_rem_ms', 'rcs_autodamp_mask', 'rcs_dry_run', 'rcs_event_count',
    'rcs_flight_authorized', 'rcs_safety_inhibited', 'rcs_inhibit_count', 'main_cmd_x10000',
    'main_target_force_cN', 'main_est_thrust_cN', 'main_burn_active', 'main_output_valid',
    'main_physical_enabled', 'needle_req_x10000', 'needle_limited_x10000', 'needle_adc',
    'needle_target_adc', 'needle_enabled', 'needle_lock', 'needle_fault', 'needle_zero_adc',
    'needle_error_adc', 'needle_rpwm', 'needle_lpwm', 'needle_zero_valid',
    'needle_homing_active', 'needle_homing_complete', 'needle_stall_ms', 'nrf_connected',
    'nrf_link', 'nrf_cmd', 'nrf_flags', 'nrf_age_ms', 'nrf_rx_count', 'nrf_errors',
    'nrf_invalid', 'nrf_irq', 'nrf_status', 'nrf_config', 'nrf_channel', 'nrf_rf_setup',
    'nrf_fifo', 'stop_latched', 'sd_initialized', 'sd_mount_ok', 'sd_file_open', 'sd_ready',
    'sd_logging', 'sd_last_result', 'sd_disk_status', 'sd_mount_retries', 'sd_errors',
    'sd_write_errors', 'sd_dropped', 'sd_ring_overruns', 'sd_async_timeouts', 'sd_frames',
    'sd_source_publishes', 'sd_hal_init', 'sd_wide', 'sd_host_attempts', 'sd_host_resets',
    'sd_hal_error', 'sd_last_hal_error', 'sd_recovered', 'sd_soft_recoveries',
    'sd_runtime_reinits', 'sd_runtime_reinit_success', 'sd_runtime_reinit_failures',
    'sd_runtime_last_error', 'sd_runtime_rec_count', 'sd_runtime_rec_success',
    'sd_runtime_rec_failures', 'sd_runtime_rec_flight_aborts', 'sd_runtime_rec_last_us',
    'sd_runtime_rec_max_us', 'cpu_x100', 'cpu_bg_x100', 'sd_update_us', 'sd_update_max_us',
    'sd_drain_us', 'sd_drain_max_us', 'sd_drain_yields', 'cov_us', 'cov_max_us', 'cov_count',
    'sched_slow_defers', 'bg_fresh_us', 'bg_remote_us', 'bg_control_us', 'bg_uart_us',
    'bg_uart_max_us', 'sd_defers', 'control_defers', 'uart_defers', 'cpu_imu_x100',
    'cpu_baro_x100', 'cpu_lidar_x100', 'cpu_nrf_x100', 'cpu_eskf_x100', 'task_imu_us',
    'task_baro_us', 'task_lidar_us', 'task_nrf_us', 'task_eskf_us', 'task_imu_max_us',
    'task_baro_max_us', 'task_lidar_max_us', 'task_nrf_max_us', 'task_eskf_max_us', 'miss_imu',
    'miss_baro', 'miss_lidar', 'miss_nrf', 'miss_eskf', 'fast_fault',
    'baro_raw_spike_rejects', 'baro_raw_step_confirms',
    'eskf_lidar_soft_reacq_active', 'eskf_lidar_soft_reacq_count',
    'eskf_lidar_soft_reacq_success', 'system_ok', 'system_fault',
    'system_imu_fresh', 'system_lidar_fresh', 'system_eskf_fresh', 'scheduler_realigns',
    'uart_dma_errors', 'uart_busy_skips',
)

assert len(FIELDS) == 212

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
        raise ValueError(f"CRC mismatch: received={received_crc:04X} calculated={calculated_crc:04X}")
    values = body.split(",")
    if len(values) != len(FIELDS):
        raise ValueError(f"field count mismatch: received={len(values)} expected={len(FIELDS)}")
    return dict(zip(FIELDS, values))

def serial_lines(port: str, baud: int) -> Iterable[str]:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("Serial monitoring requires pyserial: pip install pyserial") from exc
    with serial.Serial(port, baudrate=baud, timeout=1.0) as connection:
        print(f"# P39 UART monitor | port={port} baud={baud} fields={len(FIELDS)}")
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

def compact_status(f: dict[str, str]) -> str:
    return (
        f"seq={f['seq']} t={f['time_ms']}ms "
        f"flight={f['flight_active']} auth={f['actuator_authorized']} "
        f"ESKF={f['eskf_ok']} inh/reacq={f['eskf_inhibit']}/{f['eskf_reacquire']} "
        f"div={f['eskf_div_count']} reason={f['eskf_div_reason']} "
        f"fast={f['fast_fault']} Lsoft={f['eskf_lidar_soft_reacq_active']}/"
        f"{f['eskf_lidar_soft_reacq_count']}/{f['eskf_lidar_soft_reacq_success']} "
        f"z={int(f['eskf_z_mm'])/1000.0:+.3f}m vz={int(f['eskf_vz_mmps'])/1000.0:+.3f}m/s "
        f"IMUrec={f['imu_recovery_count']} fail={f['imu_recovery_failures']} "
        f"Lrec={f['lidar_rec_active']}/{f['lidar_rec_step']} "
        f"{f['lidar_rec_success']}/{f['lidar_rec_failures']} "
        f"SD={f['sd_ready']}/{f['sd_logging']} frames={f['sd_frames']} "
        f"drop={f['sd_dropped']} rtRec={f['sd_runtime_rec_success']}/{f['sd_runtime_rec_failures']} "
        f"CPU={int(f['cpu_x100'])/100.0:.1f}% "
        f"miss={f['miss_imu']}/{f['miss_baro']}/{f['miss_lidar']}/{f['miss_nrf']}/{f['miss_eskf']} "
        f"realign={f['scheduler_realigns']} fault={f['system_fault']}"
    )

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", nargs="?", help="capture file or '-' for stdin")
    parser.add_argument("--port", help="serial port, for example COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    lines = serial_lines(args.port, args.baud) if args.port else input_lines(args.capture)
    valid = rejected = 0
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
