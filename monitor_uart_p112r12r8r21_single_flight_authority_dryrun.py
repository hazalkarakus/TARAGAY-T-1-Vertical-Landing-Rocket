#!/usr/bin/env python3
"""TARAGAY-T1 P112R12R8R21 flight logic dry-run observer — READ ONLY."""
from __future__ import annotations
import argparse, binascii, json, statistics, sys, time
from pathlib import Path

PREFIX="$TGY73,"; N=180; BAUD=115200
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
"fl_pred_pitch_cdeg","fl_pred_yaw_cdeg","fl_tgo_ms")
assert len(FIELDS)==N, (len(FIELDS),N)

def n(v):
    try:return int(v,0)
    except Exception:
        try:return float(v)
        except Exception:return v

def iv(f,k):
    try:return int(f.get(k,0))
    except Exception:return 0

def pct(f,k): return iv(f,k)/100.0

def dec(line):
    s=line.strip()
    if not s.startswith(PREFIX) or "*" not in s: raise ValueError("not TGY73")
    body,crc=s.rsplit("*",1); rx=int(crc,16); calc=binascii.crc_hqx(body.encode("ascii"),0xFFFF)
    if rx!=calc: raise ValueError(f"CRC {rx:04X}!={calc:04X}")
    a=body.split(",")
    if len(a)!=N: raise ValueError(f"fields {len(a)} != {N}")
    return {k:n(v) for k,v in zip(FIELDS,a)},s

def serial_lines(port,baud,duration):
    try: import serial
    except ImportError:
        print("Kur: py -m pip install pyserial"); raise
    t0=time.monotonic()
    with serial.Serial(port,baudrate=baud,timeout=.5) as s:
        try:s.reset_input_buffer()
        except Exception:pass
        print("="*120)
        print("P112R12R8R21 FLIGHT LOGIC DRYRUN / MATLAB V19.6 + V7.13.4 / READ ONLY")
        print("BASINCSIZ/INERT. HICBIR ACTUATOR/RCS FIZIKSEL CIKISI BEKLENMEZ. Yaklasik 11-15 s sadece UART izle.")
        print("="*120)
        while duration<=0 or time.monotonic()-t0<duration:
            b=s.readline()
            if b: yield b.decode("ascii",errors="replace")

def file_lines(path):
    for line in Path(path).open(errors="replace"):
        q=line.strip()
        if q.startswith("{"):
            try:
                d=json.loads(q)
                if isinstance(d.get("raw_frame"),str): yield d["raw_frame"]; continue
            except Exception:pass
        yield line

def status(f,tag=""):
    return (f"t={iv(f,'time_ms')/1000:7.2f}s {tag:>6} | CPU={pct(f,'cpu_x100'):5.2f}% "
            f"SYS={iv(f,'system_ok')}/{iv(f,'system_fault')} ESKF={iv(f,'eskf_ok')} | "
            f"FL state={iv(f,'fl_state')} sim={iv(f,'fl_synth')} tl={iv(f,'fl_elapsed_ms')}ms "
            f"z={iv(f,'fl_z_cg_mm')/1000:.3f} vz={iv(f,'fl_vz_mms')/1000:.3f} "
            f"F={iv(f,'fl_target_force_cN')/100:.1f}N L={iv(f,'fl_valve_x10000')/10000:.4f} | "
            f"RCS req/app=0x{iv(f,'fl_rcs_req_mask'):X}/0x{iv(f,'fl_rcs_applied_mask'):X} fault={iv(f,'fl_rcs_fault')} "
            f"modes={iv(f,'fl_rcs_pitch_mode')}/{iv(f,'fl_rcs_yaw_mode')} | "
            f"PHYS needlePWM={iv(f,'p110_pwm')} rcs={iv(f,'rcs_mask')}")

def run(a):
    lines=file_lines(a.capture) if a.capture else serial_lines(a.port,a.baud,a.duration)
    log=None if a.capture else Path(a.log).open("w",buffering=1)
    valid=reject=0; first=None; latest=None; cpu=[]
    states=set(); req_masks=set(); max_valve=0; max_force=0; max_rcs_fault=0
    max_phys_pwm=0; phys_rcs_seen=False; ready_seen=False; unhealthy_after=0; healthy_started=False
    base={}
    try:
        for line in lines:
            if not line.strip().startswith(PREFIX): continue
            try:f,raw=dec(line)
            except Exception as e: reject+=1; print("REJECT",e,file=sys.stderr); continue
            valid+=1; latest=f; first=f if first is None else first
            if log:
                o=dict(f); o['cpu_pct']=pct(f,'cpu_x100'); o['raw_frame']=raw; log.write(json.dumps(o,sort_keys=True)+"\n")
            if not base:
                for k in ('miss_imu','miss_baro','miss_lidar','miss_nrf','miss_eskf','scheduler_realigns','p111_moves'):
                    base[k]=iv(f,k)
            if iv(f,'cpu_x100')>0: cpu.append(pct(f,'cpu_x100'))
            states.add(iv(f,'fl_state'))
            if iv(f,'fl_rcs_req_mask')!=0: req_masks.add(iv(f,'fl_rcs_req_mask'))
            max_valve=max(max_valve,iv(f,'fl_valve_x10000'))
            max_force=max(max_force,iv(f,'fl_target_force_cN'))
            max_rcs_fault=max(max_rcs_fault,iv(f,'fl_rcs_fault'))
            max_phys_pwm=max(max_phys_pwm,iv(f,'p110_pwm'))
            phys_rcs_seen |= (iv(f,'rcs_mask')!=0 or iv(f,'fl_rcs_applied_mask')!=0)
            ready_seen |= iv(f,'ready')==1
            healthy=(iv(f,'ready')==1 and iv(f,'system_ok')==1 and iv(f,'eskf_ok')==1 and iv(f,'baro_fresh')==1 and iv(f,'lidar_fresh')==1)
            if healthy: healthy_started=True
            elif healthy_started: unhealthy_after+=1
            if a.print_every>0 and valid%a.print_every==0: print(status(f))
    finally:
        if log: log.close()
    if latest is None: print("TGY73 frame yok"); return 2
    dur=max(.001,(iv(latest,'time_ms')-iv(first,'time_ms'))/1000.0)
    def delta(k): return max(0,iv(latest,k)-base.get(k,0))
    avg=statistics.mean(cpu) if cpu else 0.0; mx=max(cpu) if cpu else 0.0
    print("\n================ R8R21 SINGLE-AUTHORITY FLIGHT LOGIC DRYRUN OZET ================")
    print(f"Valid/rejected             : {valid}/{reject}")
    print(f"Capture duration           : {dur:.3f} s")
    print(f"CPU avg/max                : {avg:.2f}% / {mx:.2f}%")
    print(f"Seen flight states         : {sorted(states)}")
    print(f"Final state                : {iv(latest,'fl_state')}  (0 ARM,1 ASCENT,2 HOVER,3 DESCENT,4 TOUCHDOWN)")
    print(f"Logic step/elapsed         : {iv(latest,'fl_step_count')} / {iv(latest,'fl_elapsed_ms')} ms")
    print(f"Max target force / valve   : {max_force/100:.2f} N / {max_valve/10000:.4f}")
    print(f"Hover best                 : {iv(latest,'fl_hover_best_ms')/1000:.3f} s")
    print(f"RCS requested masks seen   : {[hex(x) for x in sorted(req_masks)]}")
    print(f"RCS fault max/final        : {max_rcs_fault}/{iv(latest,'fl_rcs_fault')}")
    print(f"Physical RCS applied seen  : {phys_rcs_seen} (MUST be False)")
    print(f"Physical needle PWM max    : {max_phys_pwm} (MUST be 0)")
    print(f"Physical needle move delta : {delta('p111_moves')} (MUST be 0)")
    print(f"Miss delta I/B/L/N/E       : {delta('miss_imu')}/{delta('miss_baro')}/{delta('miss_lidar')}/{delta('miss_nrf')}/{delta('miss_eskf')}")
    print(f"Scheduler realign delta    : {delta('scheduler_realigns')}")
    print(f"Ready/system/ESKF final    : {iv(latest,'ready')}/{iv(latest,'system_ok')}/{iv(latest,'eskf_ok')}")
    print(f"Unhealthy after healthy    : {unhealthy_after}")
    expected_states={0,1,2,3,4}
    pass_ok=(dur>=10.0 and expected_states.issubset(states) and iv(latest,'fl_state')==4 and
             iv(latest,'fl_synth')==1 and max_valve>0 and len(req_masks)>0 and max_rcs_fault==0 and
             not phys_rcs_seen and max_phys_pwm==0 and delta('p111_moves')==0 and
             delta('miss_imu')==0 and delta('miss_lidar')==0 and delta('miss_nrf')==0 and delta('miss_eskf')==0 and
             iv(latest,'system_ok')==1 and iv(latest,'eskf_ok')==1)
    print("\nSONUC:", "PASS - MATLAB FLIGHT LOGIC PORT + STATE/RCS DRYRUN" if pass_ok else "PASS DEGIL - TXT LOGU GONDER")
    return 0 if pass_ok else 1

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('capture',nargs='?'); ap.add_argument('--port',default='COM21'); ap.add_argument('--baud',type=int,default=BAUD); ap.add_argument('--duration',type=float,default=25.0); ap.add_argument('--print-every',type=int,default=10); ap.add_argument('--log',default='uart_p112r12r8r20_three_turn_flight_logic_dryrun.txt'); a=ap.parse_args(); raise SystemExit(run(a))
if __name__=='__main__': main()
