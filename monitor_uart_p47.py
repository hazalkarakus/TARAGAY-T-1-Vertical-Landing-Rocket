#!/usr/bin/env python3
"""Monitor TARAGAY-T1 P47 $TGY58 UART telemetry (253 fields)."""

from __future__ import annotations

import argparse
import binascii
import json
import sys
import time
import traceback
from pathlib import Path
from typing import Iterable

FIELDS = (
    'frame',
    'seq',
    'time_ms',
    'preflight_state',
    'flight_active',
    'pe9_raw_open',
    'pe9_debounced_open',
    'preflight_ready',
    'preflight_fault',
    'flight_time_ms',
    'connector_seen',
    'actuator_authorized',
    'imu_valid',
    'imu_age_ms',
    'imu_recovery_state',
    'imu_recovery_step',
    'imu_recovery_count',
    'imu_recovery_attempts',
    'imu_recovery_failures',
    'imu_recovery_last_us',
    'imu_recovery_max_us',
    'imu_stale_count',
    'imu_pattern_errors',
    'imu_pattern_retries',
    'imu_pattern_retry_success',
    'imu_pattern_recovery_escalations',
    'imu_invalid_samples',
    'imu_redundant_rejects',
    'baro_valid',
    'lidar_valid',
    'eskf_init',
    'eskf_ok',
    'eskf_inhibit',
    'eskf_reacquire',
    'eskf_div_reason',
    'eskf_div_count',
    'eskf_reacq_count',
    'eskf_div_candidate',
    'eskf_reacq_stable',
    'vertical_consistency_mm',
    'vertical_reacq_target_mm',
    'eskf_public_age_ms',
    'eskf_public_count',
    'eskf_reset_reason',
    'eskf_reset_count',
    'eskf_num_errors',
    'eskf_public_rejects',
    'eskf_z_jump_rejects',
    'eskf_vz_jump_rejects',
    'eskf_cov_ok',
    'eskf_cov_checks',
    'eskf_cov_faults',
    'eskf_cov_reinits',
    'eskf_cov_diag_min_u1e6',
    'eskf_cov_diag_max_m1e3',
    'eskf_cov_sym_max_u1e6',
    'eskf_cov_fault_stage',
    'eskf_cov_fault_state',
    'eskf_cov_fault_other',
    'eskf_cov_fault_raw_n1e9',
    'eskf_cov_fault_aux_n1e9',
    'eskf_cov_fault_time_ms',
    'eskf_cov_rollback_last_good',
    'eskf_cov_rollbacks',
    'eskf_cov_roundoff_clamps',
    'eskf_cov_roundoff_stage',
    'eskf_cov_roundoff_state',
    'eskf_cov_roundoff_raw_n1e9',
    'eskf_cov_roundoff_time_ms',
    'baro_fresh',
    'lidar_fresh',
    'vertical_source_mask',
    'vertical_degraded',
    'eskf_z_mm',
    'eskf_vz_mmps',
    'baro_alt_mm',
    'lidar_mm',
    'baro_pressure_pa',
    'baro_innov_mm',
    'lidar_innov_mm',
    'baro_updates',
    'lidar_updates',
    'baro_rejects',
    'lidar_rejects',
    'lidar_state',
    'lidar_age_ms',
    'lidar_reads',
    'lidar_errors',
    'lidar_timeouts',
    'lidar_wait_timeouts',
    'lidar_recoveries',
    'lidar_rec_active',
    'lidar_rec_step',
    'lidar_rec_attempts',
    'lidar_rec_success',
    'lidar_rec_failures',
    'lidar_rec_last_us',
    'lidar_rec_max_us',
    'lidar_rec_step_max_us',
    'roll_cdeg',
    'pitch_cdeg',
    'roll_rate_mdps',
    'pitch_rate_mdps',
    'rcs_state',
    'rcs_fault',
    'rcs_req_mask',
    'rcs_applied_mask',
    'rcs_roll_pd_us',
    'rcs_pitch_pd_us',
    'rcs_roll_pulse_rem_ms',
    'rcs_pitch_pulse_rem_ms',
    'rcs_roll_cooldown_rem_ms',
    'rcs_pitch_cooldown_rem_ms',
    'rcs_autodamp_mask',
    'rcs_dry_run',
    'rcs_event_count',
    'rcs_flight_authorized',
    'rcs_safety_inhibited',
    'rcs_inhibit_count',
    'main_cmd_x10000',
    'main_target_force_cN',
    'main_est_thrust_cN',
    'main_burn_active',
    'main_output_valid',
    'main_physical_enabled',
    'needle_req_x10000',
    'needle_limited_x10000',
    'needle_adc',
    'needle_target_adc',
    'needle_enabled',
    'needle_lock',
    'needle_fault',
    'needle_zero_adc',
    'needle_error_adc',
    'needle_rpwm',
    'needle_lpwm',
    'needle_zero_valid',
    'needle_homing_active',
    'needle_homing_complete',
    'needle_stall_ms',
    'nrf_connected',
    'nrf_link',
    'nrf_cmd',
    'nrf_flags',
    'nrf_age_ms',
    'nrf_rx_count',
    'nrf_errors',
    'nrf_invalid',
    'nrf_irq',
    'nrf_status',
    'nrf_config',
    'nrf_channel',
    'nrf_rf_setup',
    'nrf_fifo',
    'stop_latched',
    'sd_initialized',
    'sd_mount_ok',
    'sd_file_open',
    'sd_ready',
    'sd_logging',
    'sd_last_result',
    'sd_disk_status',
    'sd_mount_retries',
    'sd_errors',
    'sd_write_errors',
    'sd_dropped',
    'sd_ring_overruns',
    'sd_async_timeouts',
    'sd_ring_count',
    'sd_ring_high_water',
    'sd_backpressure_level',
    'sd_backpressure_entries',
    'sd_backpressure_critical_entries',
    'sd_guard_pending',
    'sd_guard_deferred',
    'sd_card_busy_polls',
    'sd_write_max_us',
    'sd_guard_max_us',
    'sd_frames',
    'sd_source_publishes',
    'sd_hal_init',
    'sd_wide',
    'sd_host_attempts',
    'sd_host_resets',
    'sd_hal_error',
    'sd_last_hal_error',
    'sd_recovered',
    'sd_soft_recoveries',
    'sd_runtime_reinits',
    'sd_runtime_reinit_success',
    'sd_runtime_reinit_failures',
    'sd_runtime_last_error',
    'sd_runtime_rec_count',
    'sd_runtime_rec_success',
    'sd_runtime_rec_failures',
    'sd_runtime_rec_flight_aborts',
    'sd_runtime_rec_last_us',
    'sd_runtime_rec_max_us',
    'cpu_x100',
    'cpu_bg_x100',
    'sd_update_us',
    'sd_update_max_us',
    'sd_drain_us',
    'sd_drain_max_us',
    'sd_drain_yields',
    'cov_us',
    'cov_max_us',
    'cov_count',
    'grav_joseph_updates',
    'grav_joseph_faults',
    'sched_slow_defers',
    'bg_fresh_us',
    'bg_remote_us',
    'bg_control_us',
    'bg_uart_us',
    'bg_uart_max_us',
    'sd_defers',
    'control_defers',
    'uart_defers',
    'cpu_imu_x100',
    'cpu_baro_x100',
    'cpu_lidar_x100',
    'cpu_nrf_x100',
    'cpu_eskf_x100',
    'task_imu_us',
    'task_baro_us',
    'task_lidar_us',
    'task_nrf_us',
    'task_eskf_us',
    'task_imu_max_us',
    'task_baro_max_us',
    'task_lidar_max_us',
    'task_nrf_max_us',
    'task_eskf_max_us',
    'miss_imu',
    'miss_baro',
    'miss_lidar',
    'miss_nrf',
    'miss_eskf',
    'fast_fault',
    'baro_raw_spike_rejects',
    'baro_raw_step_confirms',
    'eskf_lidar_soft_reacq_active',
    'eskf_lidar_soft_reacq_count',
    'eskf_lidar_soft_reacq_success',
    'system_ok',
    'system_fault',
    'system_imu_fresh',
    'system_lidar_fresh',
    'system_eskf_fresh',
    'scheduler_realigns',
    'uart_dma_errors',
    'uart_busy_skips',
)

assert len(FIELDS) == 253

RESET_REASONS = {
    "0": "NONE",
    "1": "MANUAL",
    "2": "STATE_NUM",
    "3": "COV_NONFINITE",
    "4": "COV_DIAG",
    "5": "COV_ASYM",
    "6": "PUBLIC_Z",
    "7": "PUBLIC_VZ",
    "8": "PUBLIC_Z_VZ",
}

COV_STAGES = {
    "0": "NONE", "1": "PROP", "2": "GRAV", "3": "ZUPT",
    "4": "GYRO_BIAS", "5": "ACC_BIAS", "6": "BARO", "7": "LIDAR",
    "8": "INTEGRITY", "9": "GAP",
}

COV_STATES = {
    "0": "PX", "1": "PY", "2": "PZ", "3": "VX", "4": "VY", "5": "VZ",
    "6": "THX", "7": "THY", "8": "THZ", "9": "BAX", "10": "BAY",
    "11": "BAZ", "12": "BGX", "13": "BGY", "14": "BGZ", "255": "NA",
}

def decode_frame(line: str) -> dict[str, str]:
    line = line.strip()
    if not line.startswith("$TGY58,"):
        raise ValueError("not a $TGY58 frame")
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

def _load_serial():
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
        return serial, list_ports
    except ImportError:
        print("\nHATA: pyserial kurulu degil.")
        print("Komut Istemi / PowerShell acip sunu calistir:")
        print("    py -m pip install pyserial")
        raise

def choose_port(requested: str | None) -> str:
    serial, list_ports = _load_serial()

    if requested:
        return requested

    ports = list(list_ports.comports())
    if len(ports) == 1:
        print(f"Tek seri port bulundu: {ports[0].device} - {ports[0].description}")
        return ports[0].device

    if ports:
        print("\nBulunan seri portlar:")
        for i, p in enumerate(ports, start=1):
            print(f"  {i}) {p.device:8s}  {p.description}")

        # P45 testinde COM8 kullanildigi icin mevcutsa varsayilan olarak one al.
        default_index = None
        for i, p in enumerate(ports):
            if p.device.upper() == "COM8":
                default_index = i
                break

        if default_index is not None:
            prompt = f"Port sec [Enter = COM8, 1-{len(ports)}]: "
        else:
            prompt = f"Port sec [1-{len(ports)}]: "

        while True:
            answer = input(prompt).strip()
            if answer == "" and default_index is not None:
                return ports[default_index].device
            if answer.isdigit():
                idx = int(answer) - 1
                if 0 <= idx < len(ports):
                    return ports[idx].device
            # COM adini elle de kabul et.
            if answer.upper().startswith("COM"):
                return answer.upper()
            print("Gecersiz secim.")

    print("\nOtomatik seri port bulunamadi.")
    while True:
        answer = input("STM32 UART portunu yaz (orn. COM8): ").strip().upper()
        if answer:
            return answer

def serial_lines(port: str, baud: int) -> Iterable[str]:
    serial, _ = _load_serial()
    try:
        with serial.Serial(port, baudrate=baud, timeout=1.0) as connection:
            # Acilista kart resetlenebiliyorsa kisa sure bekle.
            time.sleep(0.25)
            try:
                connection.reset_input_buffer()
            except Exception:
                pass
            print(f"# P47 UART monitor | port={port} baud={baud} fields={len(FIELDS)}")
            print("# Durdurmak icin Ctrl+C")
            while True:
                raw = connection.readline()
                if raw:
                    yield raw.decode("ascii", errors="replace")
    except serial.SerialException as exc:
        raise RuntimeError(f"{port} acilamadi: {exc}") from exc

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
        f"patRetry={f['imu_pattern_retry_success']}/{f['imu_pattern_retries']} "
        f"patEsc={f['imu_pattern_recovery_escalations']} "
        f"Ereset={RESET_REASONS.get(f['eskf_reset_reason'], 'UNKNOWN')}"
        f"({f['eskf_reset_reason']})/{f['eskf_reset_count']} "
        f"Enum={f['eskf_num_errors']} pubRej={f['eskf_public_rejects']} "
        f"Cov={f['eskf_cov_ok']}/{f['eskf_cov_faults']}/{f['eskf_cov_reinits']} "
        f"Jgrav={f['grav_joseph_updates']}/{f['grav_joseph_faults']} "
        f"stage={COV_STAGES.get(f['eskf_cov_fault_stage'], '?')} "
        f"state={COV_STATES.get(f['eskf_cov_fault_state'], '?')} "
        f"raw={int(f['eskf_cov_fault_raw_n1e9'])/1e9:+.3g} "
        f"rb={f['eskf_cov_rollbacks']} rdo={f['eskf_cov_roundoff_clamps']} "
        f"rdoLast={COV_STAGES.get(f['eskf_cov_roundoff_stage'], '?')}/"
        f"{COV_STATES.get(f['eskf_cov_roundoff_state'], '?')}/"
        f"{int(f['eskf_cov_roundoff_raw_n1e9'])/1e9:+.3g} "
        f"diag={int(f['eskf_cov_diag_min_u1e6'])/1e6:.3g}/"
        f"{int(f['eskf_cov_diag_max_m1e3'])/1e3:.3g} "
        f"sym={int(f['eskf_cov_sym_max_u1e6'])/1e6:.3g} "
        f"Lrec={f['lidar_rec_active']}/{f['lidar_rec_step']} "
        f"{f['lidar_rec_success']}/{f['lidar_rec_failures']} "
        f"SD={f['sd_ready']}/{f['sd_logging']} frames={f['sd_frames']} "
        f"drop={f['sd_dropped']} ring={f['sd_ring_count']}/{f['sd_ring_high_water']} "
        f"bp={f['sd_backpressure_level']} guard={f['sd_guard_pending']}/{f['sd_guard_deferred']} "
        f"busy={f['sd_card_busy_polls']} wMax={f['sd_write_max_us']} gMax={f['sd_guard_max_us']} "
        f"rtRec={f['sd_runtime_rec_success']}/{f['sd_runtime_rec_failures']} "
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
    parser.add_argument("--log", default="uart_p47.txt",
                        help="decoded UART log file (default: uart_p47.txt)")
    args = parser.parse_args()

    # Capture-file mode is kept for offline decoding.
    if args.capture is not None and not args.port:
        lines = input_lines(args.capture)
        log_file = None
    else:
        try:
            selected_port = choose_port(args.port)
        except ImportError:
            return 2
        lines = serial_lines(selected_port, args.baud)
        log_file = Path(args.log)

    valid = rejected = 0
    log_handle = None
    p46_seen = 0
    unknown_seen = 0

    try:
        if log_file is not None:
            log_handle = log_file.open("w", encoding="utf-8", buffering=1)
            header = (
                f"# P47 UART monitor | port={selected_port} "
                f"baud={args.baud} fields={len(FIELDS)}\n"
            )
            log_handle.write(header)
            log_handle.write("# " + ",".join(FIELDS) + "\n")
            print(f"# Kayit dosyasi: {log_file.resolve()}")

        for line in lines:
            clean = line.rstrip("\r\n")

            if not clean.startswith("$TGY58,"):
                # Important: P45 firmware sends $TGY56. Do not silently ignore it.
                if clean.startswith("$TGY57,"):
                    p46_seen += 1
                    if p46_seen == 1 or (p46_seen % 10) == 0:
                        print(
                            f"UYARI: COM porttan $TGY57 geliyor (adet={p46_seen}). "
                            "Bu P46/V57 firmware demektir; P47 flash edilmemis olabilir.",
                            flush=True,
                        )
                    if log_handle is not None:
                        log_handle.write("# P46_FRAME_DETECTED " + clean + "\n")
                    continue

                # Firmware banner/comment lines are useful in the log too.
                if clean.startswith("#"):
                    print(clean, flush=True)
                    if log_handle is not None:
                        log_handle.write(clean + "\n")
                    continue

                # Show a few unexpected raw lines instead of appearing frozen.
                if clean:
                    unknown_seen += 1
                    if unknown_seen <= 5:
                        print(f"RAW[{unknown_seen}]: {clean[:180]}", flush=True)
                continue

            try:
                frame = decode_frame(clean)
            except (ValueError, UnicodeError) as exc:
                rejected += 1
                print(f"REJECT[{rejected}]: {exc}", file=sys.stderr)
                continue

            valid += 1

            # Screen: compact status by default, full JSON with --json.
            print(json.dumps(frame, sort_keys=True) if args.json else compact_status(frame), flush=True)

            # File: always full decoded JSON; this is easiest to analyse later.
            if log_handle is not None:
                log_handle.write(json.dumps(frame, sort_keys=True) + "\n")

    except KeyboardInterrupt:
        print("\nMonitor durduruldu.")
    finally:
        if log_handle is not None:
            log_handle.close()

    print(f"valid={valid} rejected={rejected}", file=sys.stderr)
    return 0 if rejected == 0 else 1


def run_safely() -> int:
    try:
        return main()
    except Exception as exc:
        print("\n================ UART MONITOR HATASI ================")
        print(exc)
        print("=====================================================")
        traceback.print_exc()
        return 2

if __name__ == "__main__":
    code = run_safely()
    # On Windows/double-click, keep the console visible after an error.
    if code != 0:
        try:
            input("\nKapatmak icin Enter'a basin...")
        except EOFError:
            pass
    raise SystemExit(code)
