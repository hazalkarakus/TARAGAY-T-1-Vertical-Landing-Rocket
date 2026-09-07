#!/usr/bin/env python3
"""TARAGAY-T1 R8R35R3R7R1 no-runtime-HAL_Delay + SDIO diagnostic - standalone live monitor."""
from __future__ import annotations
import argparse,binascii,time
from pathlib import Path

PREFIX="$TGY73,"
FIELDS=('frame', 'seq', 'time_ms', 'ready', 'flight_active', 'pe9_open', 'auth', 'imu_valid', 'imu_age_ms', 'baro_valid', 'baro_fresh', 'baro_pa', 'baro_tcdeg', 'lidar_valid', 'lidar_fresh', 'lidar_age_ms', 'lidar_state', 'lidar_errors', 'lidar_timeouts', 'lidar_recoveries', 'lidar_rec_active', 'lidar_rec_step', 'lidar_rec_attempts', 'lidar_rec_success', 'lidar_rec_failures', 'lidar_rec_last_us', 'lidar_rec_max_us', 'eskf_ok', 'eskf_age_ms', 'eskf_cov_ok', 'eskf_cov_checks', 'eskf_corr_us', 'eskf_corr_max_us', 'eskf_predict_us', 'eskf_predict_max_us', 'eskf_gravity_us', 'eskf_gravity_max_us', 'eskf_stationary_us', 'eskf_stationary_max_us', 'eskf_baro_aid_us', 'eskf_baro_aid_max_us', 'eskf_lidar_aid_us', 'eskf_lidar_aid_max_us', 'eskf_public_us', 'eskf_public_max_us', 'eskf_public_count', 'gravity_updates', 'gravity_rejects', 'gravity_joseph_updates', 'gravity_joseph_faults', 'cpu_x100', 'cpu_bg_x100', 'cpu_imu_x100', 'cpu_baro_x100', 'cpu_lidar_x100', 'cpu_eskf_x100', 'cov_us', 'cov_max_us', 'task_imu_us', 'task_baro_us', 'task_lidar_us', 'task_eskf_us', 'task_imu_max_us', 'task_baro_max_us', 'task_lidar_max_us', 'task_eskf_max_us', 'miss_imu', 'miss_baro', 'miss_lidar', 'miss_eskf', 'system_ok', 'system_fault', 'sys_imu_fresh', 'sys_lidar_fresh', 'sys_eskf_fresh', 'scheduler_realigns', 'imu_late_us', 'imu_late_max_us', 'baro_late_us', 'baro_late_max_us', 'eskf_late_us', 'eskf_late_max_us', 'baro_defer_count', 'baro_defer_streak', 'baro_defer_streak_max', 'baro_defer_slack_us', 'eskf_defer_count', 'eskf_defer_streak', 'eskf_defer_streak_max', 'eskf_defer_slack_us', 'bg_fresh_us', 'bg_fresh_max_us', 'bg_remote_us', 'bg_remote_max_us', 'bg_control_us', 'bg_control_max_us', 'sd_update_us', 'sd_update_max_us', 'bg_uart_us', 'bg_uart_max_us', 'uart_defers', 'sd_ready', 'nrf_connected', 'nrf_link', 'nrf_flags', 'nrf_age_ms', 'nrf_rx_count', 'nrf_valid_count', 'nrf_errors', 'nrf_invalid', 'nrf_irq', 'nrf_status', 'nrf_config', 'nrf_channel', 'nrf_rf_setup', 'nrf_fifo', 'cpu_nrf_x100', 'task_nrf_us', 'task_nrf_max_us', 'miss_nrf', 'nrf_tlm_schedule', 'nrf_tlm_tx_start', 'nrf_tlm_tx_success', 'nrf_tlm_tx_fail', 'nrf_tlm_pending_replace', 'nrf_tlm_last_tx_us', 'nrf_tlm_max_tx_us', 'stop_latched', 'estop_close_active', 'estop_close_complete', 'estop_close_failed', 'estop_close_fail_reason', 'estop_close_start_count', 'estop_close_elapsed_ms', 'p110_pwm', 'p111_moves', 'p111_fault', 'needle_fault', 'rcs_mask', 'hardoff', 'isr_max_us', 'needle_adc', 'needle_closed_ref_adc', 'needle_target_adc', 'needle_error_adc', 'needle_ref_valid', 'needle_position_locked', 'needle_auto_state', 'needle_move_in_progress', 'p110_direction', 'needle_lpwm', 'needle_rpwm', 'needle_cmd_x10000', 'needle_pos_x10000', 'estop_override_used', 'fl_synth', 'fl_state', 'fl_step_count', 'fl_elapsed_ms', 'fl_z_cg_mm', 'fl_vz_mms', 'fl_x_mm', 'fl_y_mm', 'fl_pitch_cdeg', 'fl_yaw_cdeg', 'fl_zref_mm', 'fl_hover_best_ms', 'fl_target_force_cN', 'fl_valve_x10000', 'fl_target_pitch_cdeg', 'fl_target_yaw_cdeg', 'fl_thrust_shortage', 'fl_rcs_fault', 'fl_rcs_req_mask', 'fl_rcs_applied_mask', 'fl_rcs_pitch_mode', 'fl_rcs_yaw_mode', 'fl_pred_pitch_cdeg', 'fl_pred_yaw_cdeg', 'fl_tgo_ms', 'fl_real', 'fl_input_valid', 'fl_input_reject', 'fl_hpos_valid', 'fl_vpos_valid', 'fl_origin_zeroed', 'fl_eskf_inhibit', 'fl_input_eskf_age_ms', 'fl_input_imu_age_ms', 'fl_vx_mms', 'fl_vy_mms', 'fl_pitch_rate_cdeg_s', 'fl_yaw_rate_cdeg_s', 'fl_invalid_count', 'fl_horiz_gated', 'fl_v1_events', 'fl_v3_events', 'fl_v5_events', 'fl_v7_events', 'cal_phase', 'cal_valid', 'cal_fault', 'cal_upright_samples', 'cal_tilt_samples', 'cal_tilt_cdeg', 'cal_r00_x10000', 'cal_r01_x10000', 'cal_r02_x10000', 'cal_r10_x10000', 'cal_r11_x10000', 'cal_r12_x10000', 'cal_r20_x10000', 'cal_r21_x10000', 'cal_r22_x10000', 'cal_y_tilt_samples', 'cal_y_tilt_cdeg', 'cal_xy_angle_cdeg', 'cal_axis_agree_x10000', 'cal_ortho_err_x10000', 'cal_det_x10000', 'cal_neg_x_samples', 'cal_neg_x_tilt_cdeg', 'cal_neg_y_samples', 'cal_neg_y_tilt_cdeg', 'cal_x_opp_x10000', 'cal_y_opp_x10000', 'cal_z_agree_x10000', 'needle_auto_fault', 'needle_stall_ms', 'needle_cmd_rejects', 'p110_state', 'p110_result', 'p110_abort_reason', 'p110_powered_ms', 'p110_speed_adc_s', 'p110_stop_distance_adc', 'p110_corrections', 'rcs_pulse_complete_count', 'rcs_last_pulse_ms', 'rcs_max_pulse_ms', 'p83_diag_flags', 'p83_adc1_raw', 'p83_adc2_raw', 'p83_pair_diff', 'p83_pair_candidate', 'p83_median7', 'p83_filtered_adc', 'p83_feedback_valid', 'p83_confidence_pct', 'p83_pair_reject_count', 'p83_rate_reject_count', 'p83_quarantine_count', 'p83_reacquire_count', 'p83_mode', 'p83_acq_progress_pct', 'p83_adc_timeout_count', 'p83_win_raw_pp', 'p83_win_filtered_pp', 'p83_vref_win_pp_raw12', 'sd_initialized', 'sd_mount_ok', 'sd_file_open', 'sd_logging', 'sd_last_result', 'sd_disk_status', 'sd_mount_retries', 'sd_errors', 'sd_write_errors', 'sd_dropped', 'sd_ring_overruns', 'sd_async_timeouts', 'sd_ring_count', 'sd_ring_high_water', 'sd_backpressure_level', 'sd_guard_pending', 'sd_guard_deferred', 'sd_card_busy_polls', 'sd_runtime_reinits', 'sd_runtime_reinit_success', 'sd_runtime_reinit_failures', 'sd_runtime_last_error', 'sd_runtime_rec_count', 'sd_runtime_rec_success', 'sd_runtime_rec_failures', 'sd_runtime_rec_flight_aborts', 'sd_runtime_rec_last_us', 'sd_runtime_rec_max_us', 'sd_frames_total', 'sd_ring_push_total', 'sd_ring_pop_total', 'sd_dma_start_total', 'sd_dma_complete_total', 'sd_update_count_total', 'sd_defer_total', 'sd_suppressed_total', 'sd_drain_last_us_r3', 'sd_drain_max_us_r3', 'sd_drain_yields_total', 'sd_write_max_us_r3', 'sd_host_busy_defers_r4', 'sd_bsp_hal_state_r4', 'sd_dma_start_errors_r4', 'sd_r7_fail_count', 'sd_r7_first_time_ms', 'sd_r7_first_hal_status', 'sd_r7_first_hsd_state', 'sd_r7_first_hsd_error', 'sd_r7_first_hsd_context', 'sd_r7_first_dma_state', 'sd_r7_first_dma_error', 'sd_r7_first_sdio_sta', 'sd_r7_first_sdio_dctrl', 'sd_r7_first_sdio_dcount', 'sd_r7_last_time_ms', 'sd_r7_last_hal_status', 'sd_r7_last_hsd_state', 'sd_r7_last_hsd_error', 'sd_r7_last_hsd_context', 'sd_r7_last_dma_state', 'sd_r7_last_dma_error', 'sd_r7_last_sdio_sta', 'sd_r7_last_sdio_dctrl', 'sd_r7_last_sdio_dcount', 'runtime_hal_delay_violations', 'runtime_hal_delay_last_ms')
N=len(FIELDS)
assert N==325,(N,325)

HAL_STATUS={0:'HAL_OK',1:'HAL_ERROR',2:'HAL_BUSY',3:'HAL_TIMEOUT'}
HAL_SD_STATE={0:'RESET',1:'READY',2:'TIMEOUT',3:'BUSY',4:'PROGRAMMING',5:'RECEIVING',6:'TRANSFER',15:'ERROR'}
HAL_DMA_STATE={0:'RESET',1:'READY',2:'BUSY',3:'TIMEOUT',4:'ERROR',5:'ABORT'}

def n(v):
    try:return int(v,0)
    except Exception:
        try:return float(v)
        except Exception:return v

def iv(f,k):
    try:return int(f.get(k,0))
    except Exception:return 0

def hx(v): return f'0x{iv({"v":v},"v") & 0xFFFFFFFF:08X}'

def dec(line):
    s=line.strip()
    if not s.startswith(PREFIX) or "*" not in s: raise ValueError("not TGY73")
    body,crc=s.rsplit("*",1)
    rx=int(crc,16); calc=binascii.crc_hqx(body.encode("ascii"),0xFFFF)
    if rx!=calc: raise ValueError(f"CRC {rx:04X}!={calc:04X}")
    a=body.split(",")
    if len(a)!=N: raise ValueError(f"fields {len(a)} != {N}")
    return {k:n(v) for k,v in zip(FIELDS,a)},s

def fmt_snapshot(f,prefix):
    hs=iv(f,prefix+'_hal_status'); ss=iv(f,prefix+'_hsd_state'); ds=iv(f,prefix+'_dma_state')
    return (
        f"t={iv(f,prefix+'_time_ms')}ms "
        f"HAL={hs}({HAL_STATUS.get(hs,'?')}) "
        f"HSD.state={ss}({HAL_SD_STATE.get(ss,'?')}) "
        f"HSD.err=0x{iv(f,prefix+'_hsd_error') & 0xFFFFFFFF:08X} "
        f"ctx=0x{iv(f,prefix+'_hsd_context') & 0xFFFFFFFF:08X} "
        f"DMA.state={ds}({HAL_DMA_STATE.get(ds,'?')}) "
        f"DMA.err=0x{iv(f,prefix+'_dma_error') & 0xFFFFFFFF:08X} "
        f"STA=0x{iv(f,prefix+'_sdio_sta') & 0xFFFFFFFF:08X} "
        f"DCTRL=0x{iv(f,prefix+'_sdio_dctrl') & 0xFFFFFFFF:08X} "
        f"DCOUNT={iv(f,prefix+'_sdio_dcount')}"
    )

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--port',default='COM21')
    ap.add_argument('--baud',type=int,default=115200)
    ap.add_argument('--duration',type=float,default=35)
    ap.add_argument('--out',default='uart_p112r12r8r35r3r7_sdio_motor_emi_diag.txt')
    a=ap.parse_args()
    import serial
    rows=[]; rejected=0; last_fail_count=0
    t0=time.monotonic(); out=Path(a.out)
    print('R8R35R3R7R1 NO-RUNTIME-HAL_DELAY + SDIO DIAGNOSTIC - LIVE')
    print('BASINCISIZ/INERT: READY=1 bekle, PE9 ayir; needle motor gucu BAGLI olsun.')
    print('R3R6 retry davranisi aynen duruyor; R3R7 sadece HAL/SDIO failure snapshot ekler.')
    print(f'Capture -> {out.resolve()}')
    with serial.Serial(a.port,baudrate=a.baud,timeout=.25) as ser, out.open('w',encoding='utf-8') as fp:
        try:ser.reset_input_buffer()
        except Exception:pass
        while a.duration<=0 or time.monotonic()-t0<a.duration:
            b=ser.readline()
            if not b: continue
            line=b.decode('ascii',errors='replace')
            try:f,raw=dec(line)
            except Exception as e:
                rejected+=1
                if rejected<=12: print(f'REJECT[{rejected}]: {e}')
                continue
            rows.append(f); fp.write(raw+'\n'); fp.flush()
            fc=iv(f,'sd_r7_fail_count')
            print(
                f"t={iv(f,'time_ms'):6d} ready={iv(f,'ready')} flight={iv(f,'flight_active')} "
                f"needle={iv(f,'needle_adc'):4d} SD={iv(f,'sd_ready')}/{iv(f,'sd_logging')} "
                f"ring={iv(f,'sd_ring_count'):3d} drop={iv(f,'sd_dropped')} "
                f"dma={iv(f,'sd_dma_start_total')}/{iv(f,'sd_dma_complete_total')} "
                f"starterr={iv(f,'sd_dma_start_errors_r4')} r7fail={fc} wr={iv(f,'sd_write_errors')}"
            )
            if fc>last_fail_count:
                print('  >>> NEW SD DMA START FAILURE')
                if last_fail_count==0: print('  FIRST:',fmt_snapshot(f,'sd_r7_first'))
                print('  LAST :',fmt_snapshot(f,'sd_r7_last'))
                last_fail_count=fc
    print(f'\nvalid={len(rows)} rejected={rejected}')
    if not rows: return 2
    first,last=rows[0],rows[-1]
    print('\n================ R8R35R3R7 DIAGNOSTIC OZET ================')
    print(f"Needle min ADC                : {min(iv(x,'needle_adc') for x in rows)}")
    print(f"SD final ready/log            : {iv(last,'sd_ready')}/{iv(last,'sd_logging')}")
    print(f"SD write/drop/overrun         : {iv(last,'sd_write_errors')}/{iv(last,'sd_dropped')}/{iv(last,'sd_ring_overruns')}")
    print(f"R7 write-start fail count     : {iv(last,'sd_r7_fail_count')}")
    print(f"R6/R4 start-error counter     : {iv(last,'sd_dma_start_errors_r4')}")
    if iv(last,'sd_r7_fail_count')>0:
        print('FIRST:',fmt_snapshot(last,'sd_r7_first'))
        print('LAST :',fmt_snapshot(last,'sd_r7_last'))
        print('HAL status kodlari            : 0=OK 1=ERROR 2=BUSY 3=TIMEOUT')
        print('HSD state kodlari             : 0=RESET 1=READY 2=TIMEOUT 3=BUSY 4=PROGRAMMING 5=RECEIVING 6=TRANSFER 15=ERROR')
        print('DMA state kodlari             : 0=RESET 1=READY 2=BUSY 3=TIMEOUT 4=ERROR 5=ABORT')
    else:
        print('No start failure captured in this run.')
    print('============================================================')
    return 0

    if rows:
        lf=rows[-1]
        print(f"Runtime HAL_Delay violations      : {iv(lf,'runtime_hal_delay_violations')}")
        print(f"Last blocked-delay request [ms]  : {iv(lf,'runtime_hal_delay_last_ms')}")

if __name__=='__main__':
    raise SystemExit(main())
