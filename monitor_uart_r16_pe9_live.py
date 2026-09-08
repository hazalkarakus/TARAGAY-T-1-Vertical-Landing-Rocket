#!/usr/bin/env python3
"""TARAGAY-T1 R16 / PE9 -- live $TGY73 UART telemetry monitor.

Alan listesi (FIELDS), firmware kaynagindan (App/Services/UARTTelemetry/
uart_telemetry.c -> UARTTelemetry_BuildTimingLine icindeki T70_U32/T70_I32
cagrilari) satir satir sayilarak, guncel R16/PE9 haline gore yeniden
turetilmistir: 340 gercek telemetri alani + hattin basindaki "$TGY73"
etiketinin kendisi icin 1 yer tutucu ('frame') = 341 virgul-ayrimli parca.
Isimler eski monitor_uart_p112r12r8r35r3r10r4r1_..._LIVE.py script'inden
daha aciklayici secildi (orn. 'debounced_open', 'actuator_authorized'),
ama alttaki veri ayni protokol/CRC'ye dayanir.

Kullanim:
    python monitor_uart_r16_pe9_live.py --port COM5
    python monitor_uart_r16_pe9_live.py --port COM5 --dump-all
    python monitor_uart_r16_pe9_live.py --port COM5 --duration 0   (suresiz)

$TGY73 hattinin gramer: "$TGY73,<340 virgullu alan>*<CRC16-CCITT hex>"
"""
from __future__ import annotations
import argparse, binascii, time
from pathlib import Path

PREFIX = "$TGY73,"

FIELDS = (
    'frame', 'sequence', 'time_ms', 'preflight_ready', 'flight_active', 'debounced_open', 'actuator_authorized', 'imu_valid', 'imu_age_ms', 'baro_valid', 'baro_fresh', 'pressure_pa', 'temperature_c_x100', 'distance_valid', 'lidar_fresh', 'lidar_age_ms', 'lidar_state', 'error_count', 'timeout_count', 'bus_recovery_count', 'lidar_recovery_active', 'lidar_recovery_step', 'lidar_recovery_attempts', 'lidar_recovery_success', 'lidar_recovery_failures', 'lidar_recovery_last_us', 'lidar_recovery_max_us', 'eskf_healthy', 'eskf_public_age_ms', 'covariance_integrity_ok', 'covariance_integrity_check_count', 'last_correction_exec_us', 'max_correction_exec_us', 'last_predict_exec_us', 'max_predict_exec_us', 'full_eskf_last_gravity_cycles', 'full_eskf_max_gravity_cycles', 'full_eskf_last_stationary_cycles', 'full_eskf_max_stationary_cycles', 'full_eskf_last_baro_cycles', 'full_eskf_max_baro_cycles', 'full_eskf_last_lidar_cycles', 'full_eskf_max_lidar_cycles', 'full_eskf_last_public_output_cycles', 'full_eskf_max_public_output_cycles', 'public_output_count', 'gravity_update_count', 'gravity_reject_count', 'gravity_joseph_update_count', 'gravity_joseph_fault_count', 'cpu_load_x100', 'cpu_external_load_x100', 'cpu_task_load_x100_imu', 'cpu_task_load_x100_baro', 'cpu_task_load_x100_lidar', 'cpu_task_load_x100_eskf', 'covariance_last_us', 'covariance_max_us', 'task_last_exec_us_imu', 'task_last_exec_us_baro', 'task_last_exec_us_lidar', 'task_last_exec_us_eskf', 'task_max_exec_us_imu', 'task_max_exec_us_baro', 'task_max_exec_us_lidar', 'task_max_exec_us_eskf', 'task_deadline_miss_imu', 'task_deadline_miss_baro', 'task_deadline_miss_lidar', 'task_deadline_miss_eskf', 'system_ok', 'system_fault', 'imu_fresh', 'lidar_fresh_2', 'eskf_public_fresh', 'scheduler_realign_count', 'scheduler_task_last_lateness_us_imu', 'scheduler_task_max_lateness_us_imu', 'scheduler_task_last_lateness_us_baro', 'scheduler_task_max_lateness_us_baro', 'scheduler_task_last_lateness_us_eskf', 'scheduler_task_max_lateness_us_eskf', 'scheduler_task_defer_count_baro', 'scheduler_task_defer_streak_baro', 'scheduler_task_max_defer_streak_baro', 'scheduler_task_last_defer_slack_us_baro', 'scheduler_task_defer_count_eskf', 'scheduler_task_defer_streak_eskf', 'scheduler_task_max_defer_streak_eskf', 'scheduler_task_last_defer_slack_us_eskf', 'p34_bg_fresh_last_us', 'p34_bg_fresh_max_us', 'p34_bg_remote_last_us', 'p34_bg_remote_max_us', 'p34_bg_control_last_us', 'p34_bg_control_max_us', 'v87_sd_update_last_us', 'v87_sd_update_max_us', 'bg_uart_last_us', 'bg_uart_max_us', 'uart_defer_count', 'sd_ready', 'nrf_connected', 'nrf_link', 'nrf_flags', 'nrf_packet_age_ms', 'nrf_rx_count', 'remote_rx_valid_packet_count', 'nrf_error_count', 'nrf_invalid_packet_count', 'nrf_irq_count', 'nrf_status_reg', 'nrf_config_reg', 'nrf_channel_reg', 'nrf_rf_setup_reg', 'nrf_fifo_status_reg', 'cpu_task_load_x100_nrf', 'task_last_exec_us_nrf', 'task_max_exec_us_nrf', 'task_deadline_miss_nrf', 'nrf_tlm_schedule_count', 'nrf_tlm_tx_start_count', 'nrf_tlm_tx_success_count', 'nrf_tlm_tx_fail_count', 'nrf_tlm_pending_replace_count', 'nrf_tlm_last_tx_duration_us', 'nrf_tlm_max_tx_duration_us', 'stop_latched', 'estop_safe_close_active', 'estop_safe_close_complete', 'estop_safe_close_failed', 'estop_safe_close_fail_reason', 'estop_safe_close_start_count', 'estop_safe_close_elapsed_ms', 'p110_active_pwm', 'p111_moves_completed', 'p111_abort_reason', 'needle_fault', 'valve_applied_mask', 'p112_hard_off_count', 'p112_isr_max_us', 'raw_adc', 'zero_adc', 'target_adc', 'error_adc', 'zero_valid', 'position_locked', 'needle_auto_state', 'move_in_progress', 'p110_direction', 'lpwm', 'rpwm', 'requested_command_x10000', 'limited_cmd_x10000', 'estop_safe_close_override_used', 'synthetic_input_active', 'mission_state', 'step_count', 'logic_elapsed_ms', 'input_z_cg_m_x1000', 'input_vz_mps_x1000', 'input_x_m_x1000', 'input_y_m_x1000', 'input_pitch_deg_x100', 'input_yaw_deg_x100', 'z_reference_m_x1000', 'hover_best_s_x1000', 'target_force_n_x100', 'valve_cmd_x10000', 'target_pitch_deg_x100', 'target_yaw_deg_x100', 'thrust_shortage', 'rcs_fault', 'rcs_requested_mask', 'rcs_applied_mask', 'rcs_pitch_mode', 'rcs_yaw_mode', 'predicted_pitch_deg_x100', 'predicted_yaw_deg_x100', 'time_to_ground_s_x1000', 'real_input_active', 'input_valid', 'input_reject_reason', 'horizontal_position_valid', 'vertical_position_valid', 'eskf_origin_zeroed', 'eskf_output_inhibited', 'input_eskf_age_ms', 'input_imu_age_ms', 'input_vx_mps_x1000', 'input_vy_mps_x1000', 'input_pitch_rate_dps_x100', 'input_yaw_rate_dps_x100', 'invalid_input_count', 'horizontal_target_gated', 'rcs_v1_event_count', 'rcs_v3_event_count', 'rcs_v5_event_count', 'rcs_v7_event_count', 'mount_cal_phase', 'mount_cal_valid', 'mount_cal_fault', 'mount_cal_upright_samples', 'mount_cal_tilt_samples', 'mount_cal_tilt_deg_x100', 'mount_r00_x10000', 'mount_r01_x10000', 'mount_r02_x10000', 'mount_r10_x10000', 'mount_r11_x10000', 'mount_r12_x10000', 'mount_r20_x10000', 'mount_r21_x10000', 'mount_r22_x10000', 'mount_cal_y_tilt_samples', 'mount_cal_y_tilt_deg_x100', 'mount_cal_xy_angle_deg_x100', 'mount_cal_axis_agreement_x10000', 'mount_cal_ortho_error_x10000', 'mount_cal_det_x10000', 'mount_cal_neg_x_samples', 'mount_cal_neg_x_tilt_deg_x100', 'mount_cal_neg_y_samples', 'mount_cal_neg_y_tilt_deg_x100', 'mount_cal_x_opposition_x10000', 'mount_cal_y_opposition_x10000', 'mount_cal_z_agreement_x10000', 'needle_auto_fault', 'stall_ms', 'command_reject_count', 'p110_state', 'p110_result', 'p110_abort_reason', 'p110_total_powered_ms', 'p110_speed_adc_s', 'p110_stop_distance_adc', 'p110_correction_count', 'pulse_complete_count', 'last_pulse_ms', 'max_pulse_ms', 'p83_diag_flags', 'p83_adc1_raw', 'p83_adc2_raw', 'p83_pair_diff', 'p83_pair_candidate', 'p83_median7', 'p83_filtered_adc', 'p83_feedback_valid', 'p83_confidence_pct', 'p83_pair_reject_count', 'p83_rate_reject_count', 'p83_quarantine_count', 'p83_reacquire_count', 'p83_mode', 'p83_acq_progress_pct', 'p83_adc_timeout_count', 'p83_win_raw_pp', 'p83_win_filtered_pp', 'p83_vref_win_pp_raw12', 'sd_initialized', 'sd_mount_ok', 'sd_file_open', 'sd_logging', 'sd_last_result', 'sd_disk_status', 'sd_mount_retry_count', 'sd_error_count', 'sd_write_error_count', 'sd_dropped_frame_count', 'sd_ring_overrun_count', 'sd_async_timeout_count', 'sd_ring_count', 'sd_ring_high_watermark', 'sd_backpressure_level', 'sd_guard_pending', 'sd_guard_deferred', 'sd_card_busy_polls', 'sd_runtime_reinits', 'sd_runtime_reinit_success', 'sd_runtime_reinit_failures', 'sd_runtime_last_error', 'sd_runtime_recovery_count', 'sd_runtime_recovery_success', 'sd_runtime_recovery_failures', 'sd_runtime_recovery_flight_aborts', 'sd_runtime_recovery_last_us', 'sd_runtime_recovery_max_us', 'sd_logger_frame_count', 'sd_logger_ring_push_count', 'sd_logger_ring_pop_count', 'sd_logger_async_data_start_count', 'sd_logger_async_data_complete_count', 'v87_sd_update_count', 'p34_sd_defer_count', 'p112_sd_suppressed_count', 'sd_logger_drain_last_us', 'sd_logger_drain_max_us', 'sd_logger_drain_budget_yield_count', 'sd_logger_max_write_duration_us', 'sd_logger_dma_retry_card_busy_count', 'sd_bsp_hal_state', 'sd_dma_start_error_count', 'sd_r7_write_start_fail_count', 'sd_r7_first_fail_time_ms', 'sd_r7_first_fail_hal_status', 'sd_r7_first_fail_hsd_state', 'sd_r7_first_fail_hsd_error', 'sd_r7_first_fail_hsd_context', 'sd_r7_first_fail_dma_state', 'sd_r7_first_fail_dma_error', 'sd_r7_first_fail_sdio_sta', 'sd_r7_first_fail_sdio_dctrl', 'sd_r7_first_fail_sdio_dcount', 'sd_r7_last_fail_time_ms', 'sd_r7_last_fail_hal_status', 'sd_r7_last_fail_hsd_state', 'sd_r7_last_fail_hsd_error', 'sd_r7_last_fail_hsd_context', 'sd_r7_last_fail_dma_state', 'sd_r7_last_fail_dma_error', 'sd_r7_last_fail_sdio_sta', 'sd_r7_last_fail_sdio_dctrl', 'sd_r7_last_fail_sdio_dcount', 'violation_count', 'last_requested_ms', 'sd_logger_motor_write_hold_active', 'sd_logger_r9_hold_remaining_ms', 'sd_logger_r9_rcs_hold_event_count', 'sd_bsp_sdio_clkcr', 'sd_logger_r10_ram_mode', 'sd_logger_r10_ram_frame_count', 'sd_logger_r10_ram_capacity', 'sd_logger_r10_ram_overflow_count', 'sd_logger_r10_capture_frozen', 'sd_logger_r10_flush_active', 'sd_logger_r10_flush_index', 'sd_logger_r10_flush_complete', 'sd_logger_r10_tail_dma_at_entry', 'sd_logger_r10_physical_sd_fault_count', 'sd_logger_r10_postflight_reason', 'sd_bsp_sdio_clkcr_2',
)
N = len(FIELDS)
assert N == 341, (N, 341)  # 340 real telemetry values + 1 for the literal "$TGY73" tag token


def n(v):
    try:
        return int(v, 0)
    except Exception:
        try:
            return float(v)
        except Exception:
            return v


def iv(f, k):
    try:
        return int(f.get(k, 0))
    except Exception:
        return 0


def dec(line):
    """Parse one '$TGY73,...*CRC' line -> (dict, raw_text). Raises ValueError on any mismatch."""
    s = line.strip()
    if not s.startswith(PREFIX) or "*" not in s:
        raise ValueError("not TGY73")
    body, crc = s.rsplit("*", 1)
    rx = int(crc, 16)
    calc = binascii.crc_hqx(body.encode("ascii"), 0xFFFF)
    if rx != calc:
        raise ValueError(f"CRC {rx:04X}!={calc:04X}")
    a = body.split(",")
    if len(a) != N:
        raise ValueError(f"fields {len(a)} != {N}")
    return {k: n(v) for k, v in zip(FIELDS, a)}, s


def live_line(f):
    """One compact status line for the terminal, the fields you look at first."""
    return (
        f"t={iv(f,'time_ms'):7d}ms  "
        f"ready={iv(f,'preflight_ready')} flight={iv(f,'flight_active')} auth={iv(f,'actuator_authorized')} "
        f"state={iv(f,'mission_state'):2d}  "
        f"needle={iv(f,'raw_adc'):4d}->{iv(f,'target_adc'):4d} (zero={iv(f,'zero_adc')}) "
        f"rcs_req=0x{iv(f,'rcs_requested_mask') & 0xFF:02X} rcs_appl=0x{iv(f,'rcs_applied_mask') & 0xFF:02X} "
        f"STOP={iv(f,'stop_latched')} "
        f"SD={iv(f,'sd_ready')}/{iv(f,'sd_logging')} "
        f"NRF={iv(f,'nrf_link')} age={iv(f,'nrf_packet_age_ms')}ms"
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='COM5', help='Seri port (orn. COM5, /dev/ttyACM0)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--duration', type=float, default=0, help='Saniye; 0 veya negatif = suresiz (Ctrl+C ile durdur)')
    ap.add_argument('--out', default=None, help='Ham satirlarin kaydedilecegi dosya (varsayilan: otomatik isim)')
    ap.add_argument('--dump-all', action='store_true', help='Her satirda TUM alanlari yazdir (varsayilan: kisa ozet satiri)')
    a = ap.parse_args()

    import serial

    out_path = Path(a.out) if a.out else Path(f"uart_r16_pe9_capture_{time.strftime('%Y%m%d_%H%M%S')}.txt")
    valid = 0
    rejected = 0
    needle_min = None
    needle_max = None
    t0 = time.monotonic()

    print("TARAGAY-T1 R16/PE9 canli UART izleme")
    print(f"Port={a.port} Baud={a.baud} Sure={'suresiz' if a.duration<=0 else str(a.duration)+'s'}")
    print(f"Kayit -> {out_path.resolve()}")
    print("Ctrl+C ile durdurabilirsiniz.\n")

    with serial.Serial(a.port, baudrate=a.baud, timeout=0.25) as ser, out_path.open('w', encoding='utf-8') as fp:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        try:
            while a.duration <= 0 or time.monotonic() - t0 < a.duration:
                b = ser.readline()
                if not b:
                    continue
                line = b.decode('ascii', errors='replace')
                try:
                    f, raw = dec(line)
                except Exception as e:
                    rejected += 1
                    if rejected <= 12:
                        print(f'REJECT[{rejected}]: {e}')
                    continue

                valid += 1
                fp.write(raw + '\n')
                fp.flush()

                adc = iv(f, 'raw_adc')
                needle_min = adc if needle_min is None else min(needle_min, adc)
                needle_max = adc if needle_max is None else max(needle_max, adc)

                if a.dump_all:
                    print(', '.join(f'{k}={f[k]}' for k in FIELDS))
                else:
                    print(live_line(f))
        except KeyboardInterrupt:
            print('\n(Ctrl+C ile durduruldu)')

    print(f'\nvalid={valid} rejected={rejected}')
    if valid:
        print(f'needle_adc min/max: {needle_min}/{needle_max}')
    return 0 if valid else 2


if __name__ == '__main__':
    raise SystemExit(main())
