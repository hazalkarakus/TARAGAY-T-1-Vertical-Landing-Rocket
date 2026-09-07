#!/usr/bin/env python3
"""TARAGAY-T1 P112R12R8R11 bounded-background qualification observer — READ ONLY.

Firmware emits compact 109-field $TGY71 timing telemetry. It separates pure ESKF
correction/predict/aid-section timing from the enclosing task-4 timing.
"""
from __future__ import annotations
import argparse, binascii, json, statistics, sys, time
from pathlib import Path

PREFIX="$TGY71,"; N=109; BAUD=115200
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
"bg_uart_us","bg_uart_max_us","uart_defers","sd_ready","p110_pwm","p111_moves","p111_fault","needle_fault","rcs_mask","hardoff","isr_max_us")
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
    if not s.startswith(PREFIX) or "*" not in s: raise ValueError("not TGY71")
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
        print("="*118)
        print("P112R12R8R11 BOUNDED BACKGROUND / COMPACT TGY71 / READ ONLY")
        print("Basinçsiz/INERT bench. UART TX/komut YOK; monitor yalnizca okur.")
        print("="*118)
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
    post=max(0,iv(f,"task_eskf_us")-iv(f,"eskf_corr_us"))
    return (f"t={iv(f,'time_ms')/1000:7.2f}s {tag:>10} | "
            f"CPU={pct(f,'cpu_x100'):5.2f}% | B={iv(f,'baro_valid')}/{iv(f,'baro_fresh')} "
            f"L={iv(f,'lidar_valid')}/{iv(f,'lidar_fresh')} E={iv(f,'eskf_ok')}/{iv(f,'sys_eskf_fresh')} | "
            f"ESKF corr={iv(f,'eskf_corr_us')}/{iv(f,'eskf_corr_max_us')}us task={iv(f,'task_eskf_us')}/{iv(f,'task_eskf_max_us')} post~{post}us "
            f"grav={iv(f,'eskf_gravity_us')}/{iv(f,'eskf_gravity_max_us')} stat={iv(f,'eskf_stationary_us')}/{iv(f,'eskf_stationary_max_us')} "
            f"baroAid={iv(f,'eskf_baro_aid_us')}/{iv(f,'eskf_baro_aid_max_us')} lidarAid={iv(f,'eskf_lidar_aid_us')}/{iv(f,'eskf_lidar_aid_max_us')} "
            f"pub={iv(f,'eskf_public_us')}/{iv(f,'eskf_public_max_us')} pred={iv(f,'eskf_predict_us')}/{iv(f,'eskf_predict_max_us')} | "
            f"miss I/B/L/E={iv(f,'miss_imu')}/{iv(f,'miss_baro')}/{iv(f,'miss_lidar')}/{iv(f,'miss_eskf')} "
            f"late I/E={iv(f,'imu_late_us')}/{iv(f,'eskf_late_us')} max={iv(f,'imu_late_max_us')}/{iv(f,'eskf_late_max_us')} "
            f"def E={iv(f,'eskf_defer_count')} streakMax={iv(f,'eskf_defer_streak_max')} slack={iv(f,'eskf_defer_slack_us')} SYS={iv(f,'system_ok')}/{iv(f,'system_fault')}")

def run(a):
    lines=file_lines(a.capture) if a.capture else serial_lines(a.port,a.baud,a.duration)
    log=None if a.capture else Path(a.log).open("w",buffering=1)
    if log: log.write(f"# P112R12R8R11 bounded background | port={a.port} baud={a.baud}\n")
    valid=reject=0; latest=None; first_t=None
    cpu=[]; first_miss=None; first_baro_miss=None; first_eskf_miss=None; first_public=None
    baro_good=False; baro_drop_start=None; baro_drops=[]
    lidar_good=False; lidar_drop_start=None; lidar_drops=[]
    last_rec_att=None; rec_events=[]
    max_pwm=max_moves=max_hardoff=max_isr=0
    max_task_imu=max_task_baro=max_task_lidar=max_task_eskf=max_cov=max_uart=0
    max_corr=max_pred=max_grav=max_stat=max_baro_aid=max_lidar_aid=max_public=0
    ready=sd=False; safety_fault=False; rcs_seen=False
    try:
        for line in lines:
            if not line.strip().startswith(PREFIX): continue
            try:f,raw=dec(line)
            except Exception as e: reject+=1; print("REJECT",e,file=sys.stderr); continue
            valid+=1; latest=f; t=iv(f,"time_ms"); first_t=t if first_t is None else first_t
            if log:
                out=dict(f); out["cpu_pct"]=pct(f,"cpu_x100"); out["raw_frame"]=raw
                log.write(json.dumps(out,sort_keys=True)+"\n")
            if first_miss is None:first_miss=iv(f,"miss_imu")
            if first_baro_miss is None:first_baro_miss=iv(f,"miss_baro")
            if first_eskf_miss is None:first_eskf_miss=iv(f,"miss_eskf")
            if first_public is None:first_public=iv(f,"eskf_public_count")
            ready|=bool(iv(f,"ready")); sd|=bool(iv(f,"sd_ready"))
            if iv(f,"cpu_x100")>0: cpu.append(pct(f,"cpu_x100"))
            max_pwm=max(max_pwm,iv(f,"p110_pwm")); max_moves=max(max_moves,iv(f,"p111_moves")); max_hardoff=max(max_hardoff,iv(f,"hardoff")); max_isr=max(max_isr,iv(f,"isr_max_us"))
            safety_fault|=bool(iv(f,"p111_fault") or iv(f,"needle_fault")); rcs_seen|=bool(iv(f,"rcs_mask"))
            max_task_imu=max(max_task_imu,iv(f,"task_imu_max_us")); max_task_baro=max(max_task_baro,iv(f,"task_baro_max_us")); max_task_lidar=max(max_task_lidar,iv(f,"task_lidar_max_us")); max_task_eskf=max(max_task_eskf,iv(f,"task_eskf_max_us")); max_cov=max(max_cov,iv(f,"cov_max_us")); max_uart=max(max_uart,iv(f,"bg_uart_max_us"))
            max_corr=max(max_corr,iv(f,"eskf_corr_max_us")); max_pred=max(max_pred,iv(f,"eskf_predict_max_us")); max_grav=max(max_grav,iv(f,"eskf_gravity_max_us")); max_stat=max(max_stat,iv(f,"eskf_stationary_max_us")); max_baro_aid=max(max_baro_aid,iv(f,"eskf_baro_aid_max_us")); max_lidar_aid=max(max_lidar_aid,iv(f,"eskf_lidar_aid_max_us")); max_public=max(max_public,iv(f,"eskf_public_max_us"))

            b=bool(iv(f,"baro_valid") and iv(f,"baro_fresh"))
            if b and not baro_good:
                baro_good=True; print("\n>>> BARO FIRST HEALTHY",status(f,"BARO_OK"))
            if baro_good:
                if not b and baro_drop_start is None: baro_drop_start=t; print("\n>>> BARO DROP",status(f,"B_DROP"))
                elif b and baro_drop_start is not None: baro_drops.append((baro_drop_start,t,t-baro_drop_start)); print(f"\n>>> BARO RECOVER {t-baro_drop_start}ms"); baro_drop_start=None

            l=bool(iv(f,"lidar_valid") and iv(f,"lidar_fresh") and iv(f,"sys_lidar_fresh"))
            if l: lidar_good=True
            if lidar_good:
                if not l and lidar_drop_start is None: lidar_drop_start=t; print("\n>>> LIDAR FRESHNESS DROP",status(f,"L_DROP"))
                elif l and lidar_drop_start is not None: lidar_drops.append((lidar_drop_start,t,t-lidar_drop_start)); print(f"\n>>> LIDAR RECOVER {t-lidar_drop_start}ms",status(f,"L_REC")); lidar_drop_start=None

            ra=iv(f,"lidar_rec_attempts")
            if last_rec_att is not None and ra!=last_rec_att:
                rec_events.append((t,ra,iv(f,"lidar_rec_success"),iv(f,"lidar_rec_last_us")))
            last_rec_att=ra
            if a.print_every>0 and valid%a.print_every==0: print(status(f))
    finally:
        if log:log.close()
    if latest is None: print("TGY71 frame yok"); return 2
    if baro_drop_start is not None: baro_drops.append((baro_drop_start,iv(latest,"time_ms"),iv(latest,"time_ms")-baro_drop_start))
    if lidar_drop_start is not None: lidar_drops.append((lidar_drop_start,iv(latest,"time_ms"),iv(latest,"time_ms")-lidar_drop_start))
    dur=max(.001,(iv(latest,"time_ms")-(first_t or iv(latest,"time_ms")))/1000.0)
    imu_delta=max(0,iv(latest,"miss_imu")-(first_miss or 0)); baro_delta=max(0,iv(latest,"miss_baro")-(first_baro_miss or 0)); eskf_delta=max(0,iv(latest,"miss_eskf")-(first_eskf_miss or 0))
    avg=lambda x: statistics.mean(x) if x else 0.0
    print("\n================ P112R12R8R11 BOUNDED-BACKGROUND OZET ================")
    print(f"Valid / rejected          : {valid} / {reject}")
    print(f"Capture duration s        : {dur:.3f}")
    print(f"Ready / SD                : {ready} / {sd}")
    print(f"BARO runtime drops        : {baro_drops}")
    print(f"LiDAR freshness drops     : {lidar_drops}")
    print(f"LiDAR recoveries final    : att={iv(latest,'lidar_rec_attempts')} success={iv(latest,'lidar_rec_success')} fail={iv(latest,'lidar_rec_failures')} max={iv(latest,'lidar_rec_max_us')}us")
    print(f"CPU avg / max             : {avg(cpu):.2f}% / {(max(cpu) if cpu else 0):.2f}%")
    public_delta=max(0,iv(latest,"eskf_public_count")-(first_public or 0))
    public_rate=public_delta/dur
    print(f"Task max I/B/L/E          : {max_task_imu}/{max_task_baro}/{max_task_lidar}/{max_task_eskf} us")
    print(f"PURE ESKF corr/predict max: {max_corr}/{max_pred} us")
    print(f"ESKF grav/stat max        : {max_grav}/{max_stat} us")
    print(f"ESKF baro/lidar aid max   : {max_baro_aid}/{max_lidar_aid} us")
    print(f"ESKF public max           : {max_public} us")
    print(f"ESKF public rate          : {public_rate:.2f} Hz ({public_delta} outputs)")
    print(f"Gravity update/reject/J/F : {iv(latest,'gravity_updates')}/{iv(latest,'gravity_rejects')}/{iv(latest,'gravity_joseph_updates')}/{iv(latest,'gravity_joseph_faults')}")
    print(f"Covariance max            : {max_cov} us")
    print(f"UART background max       : {max_uart} us")
    print(f"IMU miss delta / rate     : {imu_delta} / {imu_delta/dur:.3f} per s")
    print(f"BARO miss delta / rate    : {baro_delta} / {baro_delta/dur:.3f} per s")
    print(f"ESKF miss delta / rate    : {eskf_delta} / {eskf_delta/dur:.3f} per s")
    print(f"Final B/L/ESKF/SYS        : {iv(latest,'baro_valid')}/{iv(latest,'lidar_valid')}/{iv(latest,'eskf_ok')}/{iv(latest,'system_ok')}")
    print(f"Safety PWM/moves/faults   : {max_pwm}/{max_moves}/{iv(latest,'p111_fault')}/{iv(latest,'needle_fault')} RCS={iv(latest,'rcs_mask'):02X} HO={max_hardoff} ISR={max_isr}us")
    print(f"Scheduler lateness I/B/E  : last={iv(latest,'imu_late_us')}/{iv(latest,'baro_late_us')}/{iv(latest,'eskf_late_us')} us max={iv(latest,'imu_late_max_us')}/{iv(latest,'baro_late_max_us')}/{iv(latest,'eskf_late_max_us')} us")
    print(f"BARO defer count/streak   : {iv(latest,'baro_defer_count')} / {iv(latest,'baro_defer_streak_max')} lastSlack={iv(latest,'baro_defer_slack_us')}us")
    print(f"ESKF defer count/streak   : {iv(latest,'eskf_defer_count')} / {iv(latest,'eskf_defer_streak_max')} lastSlack={iv(latest,'eskf_defer_slack_us')}us")
    print(f"BG fresh last/max         : {iv(latest,'bg_fresh_us')}/{iv(latest,'bg_fresh_max_us')} us")
    print(f"BG remote last/max        : {iv(latest,'bg_remote_us')}/{iv(latest,'bg_remote_max_us')} us")
    print(f"BG control last/max       : {iv(latest,'bg_control_us')}/{iv(latest,'bg_control_max_us')} us")
    print(f"SD update last/max        : {iv(latest,'sd_update_us')}/{iv(latest,'sd_update_max_us')} us")

    pass_ok=(dur>=90.0 and baro_good and not baro_drops and lidar_good and not lidar_drops and
             iv(latest,"baro_valid")==1 and iv(latest,"baro_fresh")==1 and
             iv(latest,"lidar_valid")==1 and iv(latest,"lidar_fresh")==1 and
             iv(latest,"eskf_ok")==1 and iv(latest,"system_ok")==1 and
             max_corr<700 and max_pred<500 and max_task_eskf<900 and max_cov<650 and max_uart<600 and iv(latest,"sd_update_max_us")<520 and
             public_rate>=195.0 and imu_delta<=10 and baro_delta<=10 and eskf_delta<=5 and
             iv(latest,"gravity_joseph_faults")==0 and (not cpu or max(cpu)<75.0) and
             not safety_fault and not rcs_seen and max_pwm==0 and max_moves==0 and max_hardoff==0 and max_isr<=250)
    if pass_ok:
        print("\nSONUC: PASS - R8R10 SCHEDULER ROOT-CAUSE INERT")
        return 0
    print("\nSONUC: PASS DEGIL - TXT LOGU GONDER")
    return 1

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("capture",nargs="?"); ap.add_argument("--port",default="COM21"); ap.add_argument("--baud",type=int,default=BAUD); ap.add_argument("--duration",type=float,default=120.0); ap.add_argument("--print-every",type=int,default=10); ap.add_argument("--log",default="uart_p112r12r8r11_bounded_background.txt"); a=ap.parse_args(); raise SystemExit(run(a))
if __name__=="__main__": main()
