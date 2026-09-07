#!/usr/bin/env python3
# P112R2_STATIC_BURST_MONITOR
"""Monitor TARAGAY-T1 P112R2 static median-of-3 PC1 feedback qualifier. Motor locked OFF."""

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
    'p83_diag_flags',
    'p83_samples_total',
    'p83_adc1_raw',
    'p83_adc2_raw',
    'p83_pair_diff',
    'p83_pair_candidate',
    'p83_median7',
    'p83_filtered_adc',
    'p83_feedback_valid',
    'p83_confidence_pct',
    'p83_pair_reject_count',
    'p83_rate_reject_count',
    'p83_quarantine_count',
    'p83_reacquire_count',
    'p83_mode',
    'p83_acq_progress_pct',
    'p83_adc_timeout_count',
    'p83_win_raw_pp',
    'p83_win_filtered_pp',
    'p83_vref_win_pp_raw12',
    'p87_sweep_samples',
    'p87_adc1_min',
    'p87_adc1_max',
    'p87_adc2_min',
    'p87_adc2_max',
    'p87_candidate_min',
    'p87_candidate_max',
    'p87_adc1_max_step',
    'p87_adc1_step_from',
    'p87_adc1_step_to',
    'p87_adc2_max_step',
    'p87_adc2_step_from',
    'p87_adc2_step_to',
    'p87_candidate_max_step',
    'p87_candidate_step_from',
    'p87_candidate_step_to',
    'p87_filtered_max_step',
    'p87_filtered_step_from',
    'p87_filtered_step_to',
    'p87_raw_gap_event_count',
    'p87_filtered_gap_event_count',
    'p87_pair_diff_max',
    'p99_abort_reason',
    'p99_phase',
    'p99_open_chunks',
    'p99_open_drive_ms',
    'p99_return_chunks',
    'p99_return_drive_ms',
    'p105_cycle',
    'p105_completed',
    'p105_pass_mask',
    'p105_last_abort',
    'p105_last_start_adc',
    'p105_last_target_adc',
    'p105_last_open_end_adc',
    'p105_last_open_error_adc',
    'p105_last_home_adc',
    'p105_last_open_chunks',
    'p105_last_open_drive_ms',
    'p105_last_return_chunks',
    'p105_last_return_drive_ms',
    'p106_state',
    'p106_stage',
    'p106_test_pwm',
    'p106_start_adc',
    'p106_stage_start_adc',
    'p106_end_adc',
    'p106_delta_adc',
    'p106_breakaway_found',
    'p106_breakaway_pwm',
    'p106_result',
    'p107_state',
    'p107_result',
    'p107_start_adc',
    'p107_target_adc',
    'p107_end_adc',
    'p107_error_adc',
    'p107_boost_pwm',
    'p107_sustain_pwm',
    'p107_boost_ms',
    'p107_sustain_ms',
    'p107_max_open_drop_adc',
    'p107_brake_cause',
    'p109_state',
    'p109_result',
    'p109_start_adc',
    'p109_target_adc',
    'p109_prebrake_adc',
    'p109_brake_entry_adc',
    'p109_adc_100ms',
    'p109_adc_250ms',
    'p109_adc_500ms',
    'p109_coast_100_adc',
    'p109_coast_250_adc',
    'p109_coast_500_adc',
    'p109_final_error_adc',
    'p109_boost_ms',
    'p109_sustain_ms',
    'p109_brake_cause',
    'p110_state',
    'p110_result',
    'p110_direction',
    'p110_start_adc',
    'p110_target_adc',
    'p110_current_adc',
    'p110_error_adc',
    'p110_active_pwm',
    'p110_learned_breakaway_pwm',
    'p110_sustain_pwm',
    'p110_speed_adc_s',
    'p110_stop_distance_adc',
    'p110_brake_entry_adc',
    'p110_coast_max_adc',
    'p110_final_adc',
    'p110_final_error_adc',
    'p110_search_ms',
    'p110_drive_ms',
    'p110_brake_ms',
    'p110_total_powered_ms',
    'p110_correction_count',
    'p110_breakaway_found',
    'p110_abort_reason',
    'p111_state',
    'p111_result',
    'p111_cycle',
    'p111_completed',
    'p111_pass_mask',
    'p111_phase',
    'p111_moves_completed',
    'p111_baseline_adc',
    'p111_open_target_adc',
    'p111_learned_open_pwm',
    'p111_learned_close_pwm',
    'p111_learned_open_coast',
    'p111_learned_close_coast',
    'p111_last_open_error_adc',
    'p111_last_close_error_adc',
    'p111_last_open_powered_ms',
    'p111_last_close_powered_ms',
    'p111_last_open_breakaway_pwm',
    'p111_last_close_breakaway_pwm',
    'p111_last_open_stop_adc',
    'p111_last_close_stop_adc',
    'p111_last_open_coast_adc',
    'p111_last_close_coast_adc',
    'p111_abort_reason',
    'p112_isr_control_ticks',
    'p112_isr_adc_samples',
    'p112_hard_off_count',
    'p112_uart_suppressed_count',
    'p112_sd_suppressed_count',
    'p112_isr_last_us',
    'p112_isr_max_us',
    'p112_timing_critical',
    'p112_decel_active',
    'p112_decel_pwm',
    'p112_initial_coast_adc',
)

assert len(FIELDS) == 493

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
            print(f"# P112R2 STATIC BURST feedback qualifier | port={port} baud={baud} fields={len(FIELDS)}")
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


P99_ABORT_NAMES = {
    0: "NONE",
    1: "FEEDBACK_INVALID",
    2: "START_RANGE",
    3: "ALREADY_USED",
    4: "WRONG_DIRECTION",
    5: "DRIVE_TIMEOUT",
    6: "SEQUENCE_TIMEOUT",
    7: "OPEN_OVERSHOOT",
    8: "CHUNK_LIMIT",
    9: "HOME_RANGE",
}

P99_PHASE_NAMES = {
    0: "PREHOME",
    1: "PREHOME_DWELL",
    2: "OPEN",
    3: "TURNAROUND",
    4: "RETURN",
    5: "DONE",
    6: "INTERCYCLE",
}

def compact_status(f: dict[str, str]) -> str:
    def iv(name: str, default: int = 0) -> int:
        try:
            return int(f.get(name, default))
        except (TypeError, ValueError):
            return default

    valid = iv("p83_feedback_valid")
    mode = iv("p83_mode")
    mode_name = {0:"BOOT",1:"NORMAL",2:"QUAR",3:"TRAJ"}.get(mode, f"M{mode}")
    a1, a2 = iv("p83_adc1_raw"), iv("p83_adc2_raw")
    cand, filt = iv("p83_pair_candidate"), iv("p83_filtered_adc")
    diff, conf = iv("p83_pair_diff"), iv("p83_confidence_pct")
    rawpp, filtp = iv("p83_win_raw_pp"), iv("p83_win_filtered_pp")
    pairrej, raterej, quar = iv("p83_pair_reject_count"), iv("p83_rate_reject_count"), iv("p83_quarantine_count")
    vrefpp = iv("p83_vref_win_pp_raw12")
    isr, isrmax = iv("p112_isr_last_us"), iv("p112_isr_max_us")
    pwm_l, pwm_r = iv("needle_lpwm"), iv("needle_rpwm")
    if valid == 1 and mode == 1 and rawpp <= 24 and filtp <= 8 and raterej == 0:
        hint = "STATIC HEALTH GOOD"
    elif mode == 2 or raterej > 0:
        hint = "STATIC HEALTH FAIL: external PC1/source likely unstable"
    else:
        hint = "acquiring; keep bench still"
    return (
        f"t={iv('time_ms')/1000.0:7.2f}s | P112R2 MOTOR_LOCK L/R={pwm_l}/{pwm_r} | "
        f"P83 V={valid} {mode_name} conf={conf}% A1/A2={a1}/{a2} diff={diff} cand/filt={cand}/{filt} "
        f"winPP raw/filt={rawpp}/{filtp} VREFpp={vrefpp} rej={pairrej}/{raterej}/{quar} | "
        f"ISR={isr}/{isrmax}us | {hint}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", nargs="?", help="capture file or '-' for stdin")
    parser.add_argument("--port", help="serial port, for example COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--log", default="uart_p112r2_static_burst.txt",
                        help="decoded UART log file (default: uart_p112r2_static_burst.txt)")
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
                f"# P112R2 STATIC BURST feedback qualifier | port={selected_port} "
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
                            "Eski firmware olabilir; P112R2 $TGY68 frame kullanmalidir.",
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
                            "P112R2 flash durumunu kontrol et.",
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
