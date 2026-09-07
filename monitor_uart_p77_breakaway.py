# P71_MONITOR_FIXED_V2
#!/usr/bin/env python3
"""Monitor TARAGAY-T1 P77 needle-valve breakaway-threshold characterization bench firmware.

P77 keeps the $TGY68 diagnostic stream, adds raw-pulse characterization fields, keeps NRF disabled and uses UART only for diagnostics.
"""

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
    'imu_fast_cfg_checks',
    'imu_fast_cfg_attempts',
    'imu_fast_cfg_success',
    'imu_fast_cfg_failures',
    'imu_fast_cfg_last_us',
    'imu_fast_cfg_max_us',
    'imu_stale_fast_checks',
    'imu_stale_fast_attempts',
    'imu_stale_fast_success',
    'imu_stale_fast_failures',
    'imu_stale_retry_success',
    'imu_stale_recovery_escalations',
    'imu_stale_fast_last_us',
    'imu_stale_fast_max_us',
    'imu_stale_warmup_events',
    'imu_stale_warmup_polls',
    'imu_stale_warmup_success',
    'imu_stale_warmup_timeouts',
    'imu_stale_warmup_first_ready_us',
    'imu_stale_warmup_max_ready_us',
    'imu_stale_warmup_last_status',
    'imu_stale_warmup_active',
    'imu_stale_diag_valid',
    'imu_stale_diag_event_count',
    'imu_stale_diag_time_ms',
    'imu_stale_diag_age_us',
    'imu_stale_diag_reg_valid',
    'imu_stale_diag_whoami',
    'imu_stale_diag_ctrl1_xl',
    'imu_stale_diag_ctrl2_g',
    'imu_stale_diag_ctrl3_c',
    'imu_stale_diag_ctrl4_c',
    'imu_invalid_samples',
    'imu_redundant_rejects',
    'imu_pat_valid',
    'imu_pat_event_count',
    'imu_pat_time_ms',
    'imu_pat_mag',
    'imu_pat_sign_mask',
    'imu_pat_b0_gx',
    'imu_pat_b0_gy',
    'imu_pat_b0_gz',
    'imu_pat_b0_ax',
    'imu_pat_b0_ay',
    'imu_pat_b0_az',
    'imu_pat_b1_gx',
    'imu_pat_b1_gy',
    'imu_pat_b1_gz',
    'imu_pat_b1_ax',
    'imu_pat_b1_ay',
    'imu_pat_b1_az',
    'imu_pat_b2_gx',
    'imu_pat_b2_gy',
    'imu_pat_b2_gz',
    'imu_pat_b2_ax',
    'imu_pat_b2_ay',
    'imu_pat_b2_az',
    'imu_pat_reg_valid',
    'imu_pat_whoami',
    'imu_pat_ctrl1_xl',
    'imu_pat_ctrl2_g',
    'imu_pat_ctrl3_c',
    'imu_pat_ctrl4_c',
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
    'baro_ground_pa',
    'baro_temp_cdeg',
    'baro_ref_track_allowed',
    'baro_ref_track_active',
    'baro_ref_frozen',
    'baro_ref_updates',
    'baro_ref_error_mpa',
    'baro_ref_step_mpa',
    'baro_ref_last_ms',
    'baro_ref_freezes',
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
    'sd_fifo_order_faults',
    'sd_fifo_last_started_order',
    'sd_fifo_next_ready_order',
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
    'zupt_joseph_updates',
    'zupt_joseph_faults',
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
    'nrf_tlm_schedule',
    'nrf_tlm_tx_start',
    'nrf_tlm_tx_success',
    'nrf_tlm_tx_fail',
    'nrf_tlm_pending_replace',
    'nrf_tlm_last_tx_us',
    'nrf_tlm_max_tx_us',
    'p77_stage',
    'p77_result',
    'p77_button_count',
    'p77_phase',
    'p77_direction',
    'p77_pwm',
    'p77_pulse_ms',
    'p77_ref_adc',
    'p77_start_adc',
    'p77_stop_adc',
    'p77_adc_100ms',
    'p77_adc_250ms',
    'p77_adc_500ms',
    'p77_delta_500_adc',
    'p77_max_excursion_adc',
)

assert len(FIELDS) == 351

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
    if not line.startswith("$TGY68,"):
        raise ValueError("not a $TGY68 frame")
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
            print(f"# P77 UART breakaway-threshold characterization | port={port} baud={baud} fields={len(FIELDS)}")
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

def vent_pair_label(mask_value: str) -> str:
    """Return a readable label for the P71 vent/RCS applied mask.

    This is monitor-only formatting; it does not affect firmware behavior.
    """
    try:
        mask = int(str(mask_value), 0) & 0x0F
    except (TypeError, ValueError):
        return "?"

    labels = {
        0x00: "OFF",
        0x01: "X+",
        0x02: "X-",
        0x03: "X+|X-",
        0x04: "Y+",
        0x08: "Y-",
        0x0C: "Y+|Y-",
        0x0F: "ALL(UNEXPECTED)",
    }
    return labels.get(mask, f"0x{mask:02X}")


def compact_status(f: dict[str, str]) -> str:
    stage_names = {
        0: "CAPTURE_REF",
        1: "OPEN65x10",
        2: "OPEN65x15",
        3: "OPEN65x20",
        4: "OPEN75x10",
        5: "OPEN75x15",
        6: "OPEN75x20",
        7: "DONE",
        8: "ABORT",
    }
    result_names = {
        0: "NOT_RUN",
        1: "RUNNING",
        2: "NO_MOTION",
        3: "THRESHOLD_FOUND",
        4: "NO_THRESHOLD",
        5: "BUTTON_ABORT",
        6: "SAFETY_ABORT",
        7: "WRONG_DIR",
        8: "TRAVEL_GUARD",
        9: "ZERO_UNSAFE",
    }
    phase_names = {0: "IDLE", 1: "PULSE", 2: "SETTLE"}
    dir_names = {0: "-", 1: "OPEN"}

    def iv(name: str, default: int = 0) -> int:
        try:
            return int(f.get(name, str(default)), 0)
        except (TypeError, ValueError):
            return default

    stage = iv("p77_stage")
    result = iv("p77_result")
    phase = iv("p77_phase")
    direction = iv("p77_direction")
    adc = iv("needle_adc")
    ref = iv("p77_ref_adc")
    start_adc = iv("p77_start_adc")
    stop_adc = iv("p77_stop_adc")
    a100 = iv("p77_adc_100ms")
    a250 = iv("p77_adc_250ms")
    a500 = iv("p77_adc_500ms")
    delta = iv("p77_delta_500_adc")
    max_exc = iv("p77_max_excursion_adc")
    pwm = iv("p77_pwm")
    pulse_ms = iv("p77_pulse_ms")

    if f.get("system_ok", "0") != "1":
        hint = "WAIT: system_ok=1 bekle"
    elif stage == 0:
        hint = "USER #1: kapali konum referansini yakala"
    elif phase in (1, 2):
        hint = "DOKUNMA: pulse/settle aktif; USER basisi ABORT eder"
    elif stage == 1:
        hint = "USER: OPEN PWM65 / 10ms"
    elif stage == 2:
        hint = "USER: OPEN PWM65 / 15ms"
    elif stage == 3:
        hint = "USER: OPEN PWM65 / 20ms"
    elif stage == 4:
        hint = "USER: OPEN PWM75 / 10ms"
    elif stage == 5:
        hint = "USER: OPEN PWM75 / 15ms"
    elif stage == 6:
        hint = "USER: OPEN PWM75 / 20ms"
    elif stage == 7 and result == 3:
        hint = "STOP: ESİK BULUNDU. Daha fazla butona basma; logu gönder."
    elif stage == 7:
        hint = "DONE: hareket bulunmadi; logu gönder."
    else:
        hint = "ABORT: driver kapali; logu gönder."

    return (
        f"t={iv('time_ms')/1000.0:7.2f}s | SYS={f.get('system_ok')}/{f.get('system_fault')} "
        f"PE9={f.get('pe9_debounced_open')} flight={f.get('flight_active')} | "
        f"P77 {stage_names.get(stage, stage)}/{phase_names.get(phase, phase)} "
        f"res={result_names.get(result, result)} btn={iv('p77_button_count')} | "
        f"dir={dir_names.get(direction, direction)} PWM={pwm} t={pulse_ms}ms | "
        f"ADC now/ref/start/stop/100/250/500={adc}/{ref}/{start_adc}/{stop_adc}/{a100}/{a250}/{a500} "
        f"d500={delta:+d} maxExc={max_exc} | "
        f"bridge={f.get('needle_rpwm')}/{f.get('needle_lpwm')} SD={f.get('sd_ready')}/{f.get('sd_logging')} | {hint}"
    )

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", nargs="?", help="capture file or '-' for stdin")
    parser.add_argument("--port", help="serial port, for example COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--log", default="uart_p77_breakaway.txt",
                        help="decoded UART log file (default: uart_p77_breakaway.txt)")
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
    previous_seen = 0
    older_seen = 0
    unknown_seen = 0

    try:
        if log_file is not None:
            log_handle = log_file.open("w", encoding="utf-8", buffering=1)
            header = (
                f"# P77 UART breakaway-threshold characterization | port={selected_port} "
                f"baud={args.baud} fields={len(FIELDS)}\n"
            )
            log_handle.write(header)
            log_handle.write("# " + ",".join(FIELDS) + "\n")
            print(f"# Kayit dosyasi: {log_file.resolve()}")

        for line in lines:
            clean = line.rstrip("\r\n")

            if not clean.startswith("$TGY68,"):
                # Detect recent pre-P55 firmware instead of appearing frozen.
                if clean.startswith("$TGY67,") or clean.startswith("$TGY66,") or clean.startswith("$TGY65,") or clean.startswith("$TGY64,") or clean.startswith("$TGY63,") or clean.startswith("$TGY62,"):
                    previous_seen += 1
                    if previous_seen == 1 or (previous_seen % 10) == 0:
                        print(
                            f"UYARI: COM porttan eski TGY frame geliyor (adet={previous_seen}, prefix={clean[:6]}). "
                            "Eski firmware olabilir; P77 $TGY68 frame kullanmalidir.",
                            flush=True,
                        )
                    if log_handle is not None:
                        log_handle.write("# PREVIOUS_FRAME_DETECTED " + clean + "\n")
                    continue

                if clean.startswith("$TGY61,") or clean.startswith("$TGY60,") or clean.startswith("$TGY59,") or clean.startswith("$TGY58,") or clean.startswith("$TGY57,") or clean.startswith("$TGY56,"):
                    older_seen += 1
                    if older_seen == 1:
                        print(
                            f"UYARI: daha eski UART frame goruldu: {clean[:6]}. "
                            "P77 flash durumunu kontrol et.",
                            flush=True,
                        )
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
