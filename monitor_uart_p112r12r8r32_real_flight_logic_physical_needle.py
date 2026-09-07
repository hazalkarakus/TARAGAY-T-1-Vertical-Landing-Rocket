#!/usr/bin/env python3
"""TARAGAY-T1 P112R12R8R32 real flight logic -> physical needle observer."""
from __future__ import annotations
import argparse, binascii, json, statistics, sys, time
from pathlib import Path

PREFIX="$TGY73,"; N=227; BAUD=115200
FIELDS=(
"frame","seq","time_ms","ready","flight_active","pe9_open","auth",
"imu_valid","imu_age_ms","baro_valid","baro_fresh","baro_pa","baro_tcdeg",
"lidar_valid","lidar_fresh","lidar_age_ms","lidar_state","lidar_errors","lidar_timeouts","lidar_recoveries",
"lidar_rec_active","lidar_rec_step","lidar_rec_attempts","lidar_rec_success","lidar_rec_failures","lidar_rec_last_us","lidar_rec_max_us",
"eskf_ok","eskf_age_ms","eskf_cov_ok","eskf_cov_checks",
"eskf_corr_us","eskf_corr_max_us","eskf_predict_us","eskf_predict_max_us",
"eskf_gravity_us","eskf_gravity_max_us","eskf_stationary_us","eskf_stationary_max_us",
"eskf_baro_aid_us","eskf_baro_aid_max_us","eskf_lidar_aid_us","eskf_lidar_aid_max_us",
"eskf_public_us","eskf_public_max_us","eskf_public_count","gravity_updates","gravity_rejects","gravity_joseph_updates","gravity_joseph_faults",
"cpu_x100","cpu_bg_x100","cpu_imu_x100","cpu_baro_x100","cpu_lidar_x100","cpu_eskf_x100","cov_us","cov_max_us",
"task_imu_us","task_baro_us","task_lidar_us","task_eskf_us","task_imu_max_us","task_baro_max_us","task_lidar_max_us","task_eskf_max_us",
"miss_imu","miss_baro","miss_lidar","miss_eskf","system_ok","system_fault","sys_imu_fresh","sys_lidar_fresh","sys_eskf_fresh",
"scheduler_realigns","imu_late_us","imu_late_max_us","baro_late_us","baro_late_max_us","eskf_late_us","eskf_late_max_us",
"baro_defer_count","baro_defer_streak","baro_defer_streak_max","baro_defer_slack_us",
"eskf_defer_count","eskf_defer_streak","eskf_defer_streak_max","eskf_defer_slack_us",
"bg_fresh_us","bg_fresh_max_us","bg_remote_us","bg_remote_max_us","bg_control_us","bg_control_max_us","sd_update_us","sd_update_max_us",
"bg_uart_us","bg_uart_max_us","uart_defers","sd_ready",
"nrf_connected","nrf_link","nrf_flags","nrf_age_ms","nrf_rx_count","nrf_valid_count","nrf_errors","nrf_invalid","nrf_irq",
"nrf_status","nrf_config","nrf_channel","nrf_rf_setup","nrf_fifo","cpu_nrf_x100","task_nrf_us","task_nrf_max_us","miss_nrf",
"nrf_tlm_schedule","nrf_tlm_tx_start","nrf_tlm_tx_success","nrf_tlm_tx_fail","nrf_tlm_pending_replace","nrf_tlm_last_tx_us","nrf_tlm_max_tx_us",
"stop_latched","estop_close_active","estop_close_complete","estop_close_failed","estop_close_fail_reason","estop_close_start_count","estop_close_elapsed_ms",
"p110_pwm","p111_moves","p111_fault","needle_fault","rcs_mask","hardoff","isr_max_us",
"needle_adc","needle_closed_ref_adc","needle_target_adc","needle_error_adc","needle_ref_valid","needle_position_locked",
"needle_auto_state","needle_move_in_progress","p110_direction","needle_lpwm","needle_rpwm","needle_cmd_x10000","needle_pos_x10000","estop_override_used",
"fl_synth","fl_state","fl_step_count","fl_elapsed_ms","fl_z_cg_mm","fl_vz_mms","fl_x_mm","fl_y_mm","fl_pitch_cdeg","fl_yaw_cdeg",
"fl_zref_mm","fl_hover_best_ms","fl_target_force_cN","fl_valve_x10000","fl_target_pitch_cdeg","fl_target_yaw_cdeg",
"fl_thrust_shortage","fl_rcs_fault","fl_rcs_req_mask","fl_rcs_applied_mask","fl_rcs_pitch_mode","fl_rcs_yaw_mode",
"fl_pred_pitch_cdeg","fl_pred_yaw_cdeg","fl_tgo_ms",
"fl_real","fl_input_valid","fl_input_reject","fl_hpos_valid","fl_vpos_valid","fl_origin_zeroed","fl_eskf_inhibit",
"fl_input_eskf_age_ms","fl_input_imu_age_ms","fl_vx_mms","fl_vy_mms","fl_pitch_rate_cdeg_s","fl_yaw_rate_cdeg_s","fl_invalid_count",
"fl_horiz_gated","fl_v1_events","fl_v3_events","fl_v5_events","fl_v7_events",
"cal_phase","cal_valid","cal_fault","cal_upright_samples","cal_tilt_samples","cal_tilt_cdeg",
"cal_r00_x10000","cal_r01_x10000","cal_r02_x10000","cal_r10_x10000","cal_r11_x10000","cal_r12_x10000",
"cal_r20_x10000","cal_r21_x10000","cal_r22_x10000",
"cal_y_tilt_samples","cal_y_tilt_cdeg","cal_xy_angle_cdeg","cal_axis_agree_x10000","cal_ortho_err_x10000","cal_det_x10000",
"cal_neg_x_samples","cal_neg_x_tilt_cdeg","cal_neg_y_samples","cal_neg_y_tilt_cdeg","cal_x_opp_x10000","cal_y_opp_x10000","cal_z_agree_x10000")
assert len(FIELDS)==N,(len(FIELDS),N)

REJECT_REASON={0:"OK",1:"NO_SOURCE_PTR",2:"ESKF_NOT_READY",3:"VERTICAL_INVALID",4:"ESKF_INHIBITED",5:"COV_INVALID",6:"IMU_INVALID",7:"ESKF_STALE",8:"IMU_STALE",9:"ORIGIN_NOT_ZEROED",10:"NONFINITE_INPUT",11:"MOUNT_CAL_IN_PROGRESS",12:"MOUNT_CAL_FAULT"}
CAL_PHASE={0:"WAIT_SOURCE",1:"HOLD_UPRIGHT",2:"TILT_TOP_TO_PLUS_X",3:"HOLD_PLUS_X_TILT",4:"TILT_TOP_TO_MINUS_X",5:"HOLD_MINUS_X_TILT",6:"TILT_TOP_TO_PLUS_Y",7:"HOLD_PLUS_Y_TILT",8:"TILT_TOP_TO_MINUS_Y",9:"HOLD_MINUS_Y_TILT",10:"CALIBRATED",11:"CAL_FAULT",12:"FIXED_MATRIX_LOADED"}

def n(v):
    try:return int(v,0)
    except Exception:
        try:return float(v)
        except Exception:return v

def iv(f,k):
    try:return int(f.get(k,0))
    except Exception:return 0

def pct(f,k):return iv(f,k)/100.0

def dec(line):
    s=line.strip()
    if not s.startswith(PREFIX) or "*" not in s: raise ValueError("not TGY73")
    body,crc=s.rsplit("*",1); rx=int(crc,16); calc=binascii.crc_hqx(body.encode("ascii"),0xFFFF)
    if rx!=calc: raise ValueError(f"CRC {rx:04X}!={calc:04X}")
    a=body.split(",")
    if len(a)!=N: raise ValueError(f"fields {len(a)} != {N}")
    return {k:n(v) for k,v in zip(FIELDS,a)},s

def serial_lines(port,baud,duration):
    try:import serial
    except ImportError:
        print("Kur: py -m pip install pyserial"); raise
    t0=time.monotonic()
    with serial.Serial(port,baudrate=baud,timeout=.5) as s:
        try:s.reset_input_buffer()
        except Exception:pass
        print("="*110)
        print("R8R32 REAL FLIGHT LOGIC -> PHYSICAL NEEDLE")
        print("BASINCSIZ TEST. READY ONCESI NEEDLE HAREKET ETMEMELI; PE9 AYRILINCA FLIGHT LOGIC NEEDLE'I SURMELI.")
        print("1) READY=1, SYS=1, ESKF=1 ve CAL=12/1/0 bekle.")
        print("2) Basincsiz duzende PE9'u ayir; flight_active/auth=1 olmasini gozle.")
        print("3) fl_valve ile needle_cmd/ADC/PWM hareketini gozle.")
        print("4) RCS physical mask test boyunca 0 kalmali; needle travel 585 ADC'yi asmamali.")
        print("="*110)
        while duration<=0 or time.monotonic()-t0<duration:
            b=s.readline()
            if b:yield b.decode("ascii",errors="replace")

def file_lines(path):
    for line in Path(path).open(errors="replace"):
        q=line.strip()
        if q.startswith("{"):
            try:
                d=json.loads(q)
                if isinstance(d.get("raw_frame"),str):yield d["raw_frame"];continue
            except Exception:pass
        yield line


def status(f):
    return (f"t={iv(f,'time_ms')/1000:7.2f}s READY={iv(f,'ready')} FLIGHT={iv(f,'flight_active')} AUTH={iv(f,'auth')} "
            f"SYS={iv(f,'system_ok')}/{iv(f,'system_fault')} ESKF={iv(f,'eskf_ok')} CAL={iv(f,'cal_phase')}/{iv(f,'cal_valid')}/{iv(f,'cal_fault')} | "
            f"FL state={iv(f,'fl_state')} valid={iv(f,'fl_input_valid')} L={iv(f,'fl_valve_x10000')/10000:.3f} "
            f"needle_cmd={iv(f,'needle_cmd_x10000')/10000:.3f} ADC={iv(f,'needle_adc')} target={iv(f,'needle_target_adc')} "
            f"PWM={iv(f,'p110_pwm')} move={iv(f,'needle_move_in_progress')} p111={iv(f,'p111_moves')} | "
            f"RCS req/app/phys=0x{iv(f,'fl_rcs_req_mask'):X}/0x{iv(f,'fl_rcs_applied_mask'):X}/0x{iv(f,'rcs_mask'):X}")

def run(a):
    lines=file_lines(a.capture) if a.capture else serial_lines(a.port,a.baud,a.duration)
    log=None if a.capture else Path(a.log).open('w',buffering=1)
    valid=reject=0; first=None; latest=None; cpu=[]; basec={}
    flight_seen=False; flight_first_ms=None; ready_seen=False; fixed_seen=False
    preflight_pwm=False; preflight_move=False; max_pwm=0; min_adc=65535; max_adc=0
    max_target_travel=0; max_actual_travel=0; max_p111=0; max_p111_fault=0; max_needle_fault=0
    physical_rcs_seen=False; relation_samples=0; relation_bad=0; relation_max_err=0
    fl_valid_after_flight=False; nonzero_fl_after_flight=False; nonzero_cmd_after_flight=False
    try:
        for line in lines:
            if not line.strip().startswith(PREFIX): continue
            try: f,raw=dec(line)
            except Exception as e:
                reject+=1; print('REJECT',e,file=sys.stderr); continue
            valid+=1; latest=f; first=f if first is None else first
            if log:
                o=dict(f); o['cpu_pct']=pct(f,'cpu_x100'); o['raw_frame']=raw
                log.write(json.dumps(o,sort_keys=True)+'\n')
            if not basec:
                for k in ('miss_imu','miss_baro','miss_lidar','miss_nrf','miss_eskf','scheduler_realigns','p111_moves'):
                    basec[k]=iv(f,k)
            if iv(f,'cpu_x100')>0: cpu.append(pct(f,'cpu_x100'))
            ready_seen |= (iv(f,'ready')==1 and iv(f,'system_ok')==1 and iv(f,'eskf_ok')==1)
            fixed_seen |= (iv(f,'cal_phase')==12 and iv(f,'cal_valid')==1 and iv(f,'cal_fault')==0)
            if iv(f,'flight_active')==0:
                preflight_pwm |= iv(f,'p110_pwm')!=0
                preflight_move |= iv(f,'needle_move_in_progress')!=0
            else:
                if not flight_seen: flight_first_ms=iv(f,'time_ms')
                flight_seen=True
                fl_valid_after_flight |= (iv(f,'fl_real')==1 and iv(f,'fl_synth')==0 and iv(f,'fl_input_valid')==1)
                nonzero_fl_after_flight |= iv(f,'fl_valve_x10000')>0
                nonzero_cmd_after_flight |= iv(f,'needle_cmd_x10000')>0
                if iv(f,'auth')==1 and iv(f,'fl_input_valid')==1:
                    # fl_valve is L in 0..0.30 -> actuator normalized 0..1
                    expected=min(10000,max(0,round(iv(f,'fl_valve_x10000')*10000/3000)))
                    err=abs(iv(f,'needle_cmd_x10000')-expected)
                    relation_samples+=1; relation_max_err=max(relation_max_err,err)
                    if err>350: relation_bad+=1
            physical_rcs_seen |= (iv(f,'rcs_mask')!=0 or iv(f,'fl_rcs_applied_mask')!=0)
            max_pwm=max(max_pwm,iv(f,'p110_pwm')); max_p111=max(max_p111,iv(f,'p111_moves'))
            max_p111_fault=max(max_p111_fault,iv(f,'p111_fault')); max_needle_fault=max(max_needle_fault,iv(f,'needle_fault'))
            adc=iv(f,'needle_adc'); ref=iv(f,'needle_closed_ref_adc'); tgt=iv(f,'needle_target_adc')
            min_adc=min(min_adc,adc); max_adc=max(max_adc,adc)
            if ref>0:
                max_actual_travel=max(max_actual_travel,max(0,ref-adc))
                max_target_travel=max(max_target_travel,max(0,ref-tgt))
            if a.print_every>0 and valid%a.print_every==0: print(status(f))
    finally:
        if log: log.close()
    if latest is None:
        print('TGY73 frame yok'); return 2
    dur=max(.001,(iv(latest,'time_ms')-iv(first,'time_ms'))/1000.0)
    def delta(k): return max(0,iv(latest,k)-basec.get(k,0))
    avg=statistics.mean(cpu) if cpu else 0; mx=max(cpu) if cpu else 0
    print('\n================ R8R32 OZET ================')
    print(f'Valid/rejected              : {valid}/{reject}')
    print(f'Capture duration            : {dur:.3f}s')
    print(f'CPU avg/max                 : {avg:.2f}% / {mx:.2f}%')
    print(f'Ready healthy seen          : {ready_seen}')
    print(f'Fixed matrix phase12 seen   : {fixed_seen}')
    print(f'Flight/PE9 separation seen  : {flight_seen}  first_ms={flight_first_ms}')
    print(f'Pre-flight PWM/motion       : {preflight_pwm}/{preflight_move}  (MUST False/False)')
    print(f'FL real valid after flight  : {fl_valid_after_flight}')
    print(f'FL nonzero L after flight   : {nonzero_fl_after_flight}')
    print(f'Needle nonzero cmd after FL : {nonzero_cmd_after_flight}')
    print(f'Needle PWM max              : {max_pwm}')
    print(f'P111 moves delta/max        : {delta("p111_moves")}/{max_p111}')
    print(f'Needle ADC min/max          : {min_adc}/{max_adc}')
    print(f'Max target/actual travel    : {max_target_travel}/{max_actual_travel} ADC (limit 585)')
    print(f'L->needle relation samples  : {relation_samples}; bad={relation_bad}; max_err={relation_max_err}/10000')
    print(f'P111/needle fault max       : {max_p111_fault}/{max_needle_fault}')
    print(f'Physical RCS seen           : {physical_rcs_seen} (MUST False)')
    print(f'Miss delta I/B/L/N/E        : {delta("miss_imu")}/{delta("miss_baro")}/{delta("miss_lidar")}/{delta("miss_nrf")}/{delta("miss_eskf")}')
    print(f'Scheduler realign delta     : {delta("scheduler_realigns")}')
    pass_ok=(dur>=8 and ready_seen and fixed_seen and flight_seen and not preflight_pwm and not preflight_move and
             fl_valid_after_flight and nonzero_fl_after_flight and nonzero_cmd_after_flight and max_pwm>0 and
             delta('p111_moves')>0 and max_target_travel<=585 and max_actual_travel<=600 and relation_samples>0 and
             relation_bad==0 and max_p111_fault==0 and max_needle_fault==0 and not physical_rcs_seen and
             delta('miss_imu')==0 and delta('miss_baro')==0 and delta('miss_lidar')==0 and delta('miss_eskf')==0 and
             delta('scheduler_realigns')==0)
    print('\nSONUC:', 'PASS - R8R32 REAL FLIGHT LOGIC -> PHYSICAL NEEDLE' if pass_ok else
          'INCELE - R8R32 entegrasyon kosullari tamamlanmadi')
    if not flight_seen:
        print('NOT: PE9/flight_active gorulmedi. READY sonrasi basincsiz duzende PE9 ayrilma testi yapilmadan fiziksel handoff test edilemez.')
    return 0 if pass_ok else 1

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port',default='COM21'); p.add_argument('--baud',type=int,default=BAUD); p.add_argument('--duration',type=float,default=30.0)
    p.add_argument('--log',default='uart_p112r12r8r32_real_flight_logic_physical_needle.txt'); p.add_argument('--capture'); p.add_argument('--print-every',type=int,default=2)
    return run(p.parse_args())
if __name__=='__main__': raise SystemExit(main())
