#!/usr/bin/env python3
"""TARAGAY-T1 P112R12R8R29 paired-axis-derived-Z IMU->rocket frame calibration + relay bench observer."""
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
CAL_PHASE={0:"WAIT_SOURCE",1:"HOLD_UPRIGHT",2:"TILT_TOP_TO_PLUS_X",3:"HOLD_PLUS_X_TILT",4:"TILT_TOP_TO_MINUS_X",5:"HOLD_MINUS_X_TILT",6:"TILT_TOP_TO_PLUS_Y",7:"HOLD_PLUS_Y_TILT",8:"TILT_TOP_TO_MINUS_Y",9:"HOLD_MINUS_Y_TILT",10:"CALIBRATED",11:"CAL_FAULT"}

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
        print("R8R29 PAIRED-AXIS-DERIVED-Z IMU -> ROCKET FRAME CALIBRATION + RELAY BENCH")
        print("BASINCSIZ. NEEDLE/MOTOR CALISMAMALI. PE9 GEREKMEZ.")
        print("1) READY sonrasi roketi olabildigince DIK ve SABIT tut. Bu poz final Z cozumu degil, sadece dogrulama.")
        print("2) +X promptunda USTU fiziksel/CAD +X yonune 12-20 deg eg ve sabit tut.")
        print("3) -X promptunda once DIK'e don, sonra USTU -X yonune 12-20 deg eg ve sabit tut.")
        print("4) +Y promptunda once DIK'e don, sonra USTU +Y yonune 12-20 deg eg ve sabit tut.")
        print("5) -Y promptunda once DIK'e don, sonra USTU -Y yonune 12-20 deg eg ve sabit tut.")
        print("6) CALIBRATED gorunce dik konuma don; sonra +/-X ve +/-Y rolelerini gozle.")
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

def matrix(f):
    vals=[iv(f,k)/10000.0 for k in ("cal_r00_x10000","cal_r01_x10000","cal_r02_x10000","cal_r10_x10000","cal_r11_x10000","cal_r12_x10000","cal_r20_x10000","cal_r21_x10000","cal_r22_x10000")]
    return [vals[0:3],vals[3:6],vals[6:9]]

def status(f):
    ph=iv(f,"cal_phase")
    return (f"t={iv(f,'time_ms')/1000:7.2f}s READY={iv(f,'ready')} SYS={iv(f,'system_ok')}/{iv(f,'system_fault')} "
            f"CAL={ph}:{CAL_PHASE.get(ph,'?')} valid={iv(f,'cal_valid')} fault={iv(f,'cal_fault')} "
            f"upr/+X/-X/+Y/-Y={iv(f,'cal_upright_samples')}/{iv(f,'cal_tilt_samples')}/{iv(f,'cal_neg_x_samples')}/{iv(f,'cal_y_tilt_samples')}/{iv(f,'cal_neg_y_samples')} "
            f"tilt +X/-X/+Y/-Y={iv(f,'cal_tilt_cdeg')/100:.1f}/{iv(f,'cal_neg_x_tilt_cdeg')/100:.1f}/{iv(f,'cal_y_tilt_cdeg')/100:.1f}/{iv(f,'cal_neg_y_tilt_cdeg')/100:.1f}deg "
            f"oppX/Y={iv(f,'cal_x_opp_x10000')/10000:.3f}/{iv(f,'cal_y_opp_x10000')/10000:.3f} XY={iv(f,'cal_xy_angle_cdeg')/100:.1f} agree={iv(f,'cal_axis_agree_x10000')/10000:.3f} z={iv(f,'cal_z_agree_x10000')/10000:.3f} | "
            f"rocket A/B={iv(f,'fl_pitch_cdeg')/100:.2f}/{iv(f,'fl_yaw_cdeg')/100:.2f}deg "
            f"RCS req/app=0x{iv(f,'fl_rcs_req_mask'):X}/0x{iv(f,'fl_rcs_applied_mask'):X} "
            f"ev={iv(f,'fl_v1_events')}/{iv(f,'fl_v3_events')}/{iv(f,'fl_v5_events')}/{iv(f,'fl_v7_events')} PWM={iv(f,'p110_pwm')}")

def run(a):
    lines=file_lines(a.capture) if a.capture else serial_lines(a.port,a.baud,a.duration)
    log=None if a.capture else Path(a.log).open("w",buffering=1)
    valid=reject=0; first=None; latest=None; last_phase=None; cpu=[]; base={}
    calibrated_seen=False; phys_seen=False; max_pwm=0; max_fault=0; phase_seen=set()
    try:
        for line in lines:
            if not line.strip().startswith(PREFIX):continue
            try:f,raw=dec(line)
            except Exception as e:reject+=1;print("REJECT",e,file=sys.stderr);continue
            valid+=1;latest=f;first=f if first is None else first
            if log:
                o=dict(f);o["cpu_pct"]=pct(f,"cpu_x100");o["raw_frame"]=raw;log.write(json.dumps(o,sort_keys=True)+"\n")
            if not base:
                for k in ("miss_imu","miss_baro","miss_lidar","miss_nrf","miss_eskf","scheduler_realigns","p111_moves"):
                    base[k]=iv(f,k)
            if iv(f,"cpu_x100")>0:cpu.append(pct(f,"cpu_x100"))
            ph=iv(f,"cal_phase");phase_seen.add(ph);calibrated_seen|=(iv(f,"cal_valid")==1)
            phys_seen|=(iv(f,"fl_rcs_applied_mask")!=0 or iv(f,"rcs_mask")!=0)
            max_pwm=max(max_pwm,iv(f,"p110_pwm"));max_fault=max(max_fault,iv(f,"fl_rcs_fault"))
            if ph!=last_phase:
                print("\n>>> CAL PHASE:",ph,CAL_PHASE.get(ph,"?"))
                if ph==1: print(">>> ROKETI OLABILDIGINCE DIK VE SABIT TUT. Bu poz sadece dogrulama; final Z +/- ciftlerden cikacak.")
                elif ph==2: print(">>> SIMDI USTU FIZIKSEL/CAD +X YONUNE 12-20 DEG EG.")
                elif ph==3: print(">>> +X EGIMINI SABIT TUT. Yaklasik 1.2 saniye.")
                elif ph==4: print(">>> +X KAYDEDILDI. DIK'E DON; SIMDI USTU -X YONUNE 12-20 DEG EG.")
                elif ph==5: print(">>> -X EGIMINI SABIT TUT. Yaklasik 1.2 saniye.")
                elif ph==6: print(">>> X CIFTI KAYDEDILDI. DIK'E DON; SIMDI USTU +Y YONUNE 12-20 DEG EG.")
                elif ph==7: print(">>> +Y EGIMINI SABIT TUT. Yaklasik 1.2 saniye.")
                elif ph==8: print(">>> +Y KAYDEDILDI. DIK'E DON; SIMDI USTU -Y YONUNE 12-20 DEG EG.")
                elif ph==9: print(">>> -Y EGIMINI SABIT TUT. Yaklasik 1.2 saniye.")
                elif ph==10:
                    print(">>> R8R29 COZUMU TAMAM. +Z, +/-X ve +/-Y ciftlerinden turetildi. DIK KONUMA DON VE +/-X +/-Y ROLE TESTINI YAP.")
                    r=matrix(f);print(">>> R_rocket_from_imu =");[print("    "," ".join(f"{x:+.4f}" for x in row)) for row in r]
                    print(f">>> QUALITY oppX/Y={iv(f,'cal_x_opp_x10000')/10000:.4f}/{iv(f,'cal_y_opp_x10000')/10000:.4f} XY={iv(f,'cal_xy_angle_cdeg')/100:.2f}deg agree={iv(f,'cal_axis_agree_x10000')/10000:.4f} zAgree={iv(f,'cal_z_agree_x10000')/10000:.4f} det={iv(f,'cal_det_x10000')/10000:.4f} ortho={iv(f,'cal_ortho_err_x10000')/10000:.5f}")
                elif ph==11: print(">>> KALIBRASYON HATASI. Gucu kesmeden logu kaydet ve incele.")
                last_phase=ph
            if a.print_every>0 and valid%a.print_every==0:print(status(f))
    finally:
        if log:log.close()
    if latest is None:print("TGY73 frame yok");return 2
    dur=max(.001,(iv(latest,"time_ms")-iv(first,"time_ms"))/1000.0)
    def delta(k):return max(0,iv(latest,k)-base.get(k,0))
    avg=statistics.mean(cpu) if cpu else 0;mx=max(cpu) if cpu else 0
    print("\n================ R8R29 OZET ================")
    print(f"Valid/rejected             : {valid}/{reject}")
    print(f"Capture duration           : {dur:.3f}s")
    print(f"CPU avg/max                : {avg:.2f}% / {mx:.2f}%")
    print(f"Calibration phases seen    : {sorted(phase_seen)}")
    print(f"Calibration valid/fault    : {iv(latest,'cal_valid')}/{iv(latest,'cal_fault')}")
    print(f"Calibration tilt +X/-X     : {iv(latest,'cal_tilt_cdeg')/100:.2f} / {iv(latest,'cal_neg_x_tilt_cdeg')/100:.2f} deg")
    print(f"Calibration tilt +Y/-Y     : {iv(latest,'cal_y_tilt_cdeg')/100:.2f} / {iv(latest,'cal_neg_y_tilt_cdeg')/100:.2f} deg")
    print(f"Pair opposition X/Y        : {iv(latest,'cal_x_opp_x10000')/10000:.4f} / {iv(latest,'cal_y_opp_x10000')/10000:.4f}")
    print(f"XY angle / axis agreement  : {iv(latest,'cal_xy_angle_cdeg')/100:.2f} deg / {iv(latest,'cal_axis_agree_x10000')/10000:.4f}")
    print(f"Z agreement                : {iv(latest,'cal_z_agree_x10000')/10000:.4f}")
    print(f"det / ortho error          : {iv(latest,'cal_det_x10000')/10000:.5f} / {iv(latest,'cal_ortho_err_x10000')/10000:.5f}")
    if iv(latest,"cal_valid")==1:
        print("R_rocket_from_imu:")
        for row in matrix(latest):print("  "," ".join(f"{x:+.5f}" for x in row))
    print(f"RCS events V1/V3/V5/V7    : {iv(latest,'fl_v1_events')}/{iv(latest,'fl_v3_events')}/{iv(latest,'fl_v5_events')}/{iv(latest,'fl_v7_events')}")
    print(f"Physical RCS seen          : {phys_seen}")
    print(f"RCS fault max              : {max_fault}")
    print(f"Needle PWM max/move delta  : {max_pwm}/{delta('p111_moves')}  (MUST 0/0)")
    print(f"Miss delta I/B/L/N/E       : {delta('miss_imu')}/{delta('miss_baro')}/{delta('miss_lidar')}/{delta('miss_nrf')}/{delta('miss_eskf')}")
    print(f"Scheduler realign delta    : {delta('scheduler_realigns')}")
    print(f"Ready/system/ESKF final    : {iv(latest,'ready')}/{iv(latest,'system_ok')}/{iv(latest,'eskf_ok')}")
    quality_ok=(8000 <= iv(latest,"cal_xy_angle_cdeg") <= 10000 and
                iv(latest,"cal_axis_agree_x10000") >= 9500 and
                iv(latest,"cal_x_opp_x10000") >= 9400 and
                iv(latest,"cal_y_opp_x10000") >= 9400 and
                iv(latest,"cal_z_agree_x10000") >= 9700 and
                9850 <= iv(latest,"cal_det_x10000") <= 10150 and
                iv(latest,"cal_ortho_err_x10000") <= 200)
    pass_ok=(dur>=8 and calibrated_seen and iv(latest,"cal_valid")==1 and iv(latest,"cal_fault")==0 and quality_ok and max_fault==0 and max_pwm==0 and delta("p111_moves")==0 and iv(latest,"system_ok")==1 and iv(latest,"eskf_ok")==1)
    print("\nSONUC:","PASS - R8R29 PAIRED-AXIS-DERIVED-Z CALIBRATION" if pass_ok else "INCELE - calibration/geometry/gate tamamlanmadi")
    return 0 if pass_ok else 1

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--port",default="COM21");p.add_argument("--baud",type=int,default=BAUD);p.add_argument("--duration",type=float,default=45.0)
    p.add_argument("--log",default="uart_p112r12r8r29_paired_axis_derived_z_cal.txt");p.add_argument("--capture");p.add_argument("--print-every",type=int,default=3)
    return run(p.parse_args())
if __name__=="__main__":raise SystemExit(main())
