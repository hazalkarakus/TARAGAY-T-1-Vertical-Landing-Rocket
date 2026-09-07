#!/usr/bin/env python3
"""TARAGAY-T1 P112R12R8R13 P73 fast/redundant nRF coexistence qualification observer — READ ONLY."""
from __future__ import annotations
import argparse, binascii, json, statistics, sys, time
from pathlib import Path

PREFIX="$TGY72,"; N=134; BAUD=115200
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
"p110_pwm","p111_moves","p111_fault","needle_fault","rcs_mask","hardoff","isr_max_us")
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
    if not s.startswith(PREFIX) or "*" not in s: raise ValueError("not TGY72")
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
        print("P112R12R8R13 P73 FAST-REDUNDANT NRF / COMPACT TGY72 / READ ONLY")
        print("BASINCSIZ/INERT. Ilk 120 s ground switch-1 ve switch-2 OFF kalmali.")
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
    return (f"t={iv(f,'time_ms')/1000:7.2f}s {tag:>8} | CPU={pct(f,'cpu_x100'):5.2f}% "
            f"SYS={iv(f,'system_ok')}/{iv(f,'system_fault')} B/L/E={iv(f,'baro_fresh')}/{iv(f,'lidar_fresh')}/{iv(f,'sys_eskf_fresh')} | "
            f"NRF hw/link={iv(f,'nrf_connected')}/{iv(f,'nrf_link')} age={iv(f,'nrf_age_ms')}ms rx={iv(f,'nrf_rx_count')} valid={iv(f,'nrf_valid_count')} "
            f"err/inv={iv(f,'nrf_errors')}/{iv(f,'nrf_invalid')} tlm ok/fail={iv(f,'nrf_tlm_tx_success')}/{iv(f,'nrf_tlm_tx_fail')} "
            f"BGrem={iv(f,'bg_remote_us')}/{iv(f,'bg_remote_max_us')}us NRFtask={iv(f,'task_nrf_us')}/{iv(f,'task_nrf_max_us')}us | "
            f"miss I/B/L/N/E={iv(f,'miss_imu')}/{iv(f,'miss_baro')}/{iv(f,'miss_lidar')}/{iv(f,'miss_nrf')}/{iv(f,'miss_eskf')}")

def run(a):
    lines=file_lines(a.capture) if a.capture else serial_lines(a.port,a.baud,a.duration)
    log=None if a.capture else Path(a.log).open("w",buffering=1)
    if log: log.write(f"# P112R12R8R13 P73 fast/redundant nRF coexistence | port={a.port} baud={a.baud}\n")
    valid=reject=0; first=None; latest=None; cpu=[]
    ready_seen=False; healthy_t=None; healthy_frames=0; unhealthy_after=0
    base={}; maxv={k:0 for k in ('bg_remote_max_us','task_nrf_max_us','bg_uart_max_us','sd_update_max_us','task_imu_max_us','task_baro_max_us','task_lidar_max_us','task_eskf_max_us','cov_max_us','isr_max_us')}
    try:
        for line in lines:
            if not line.strip().startswith(PREFIX): continue
            try:f,raw=dec(line)
            except Exception as e: reject+=1; print("REJECT",e,file=sys.stderr); continue
            valid+=1; latest=f; first=f if first is None else first
            if log:
                o=dict(f); o['cpu_pct']=pct(f,'cpu_x100'); o['raw_frame']=raw; log.write(json.dumps(o,sort_keys=True)+"\n")
            if not base:
                for k in ('nrf_rx_count','nrf_valid_count','nrf_tlm_tx_success','nrf_tlm_tx_fail','miss_imu','miss_baro','miss_lidar','miss_nrf','miss_eskf','eskf_public_count'):
                    base[k]=iv(f,k)
            if iv(f,'cpu_x100')>0: cpu.append(pct(f,'cpu_x100'))
            for k in maxv: maxv[k]=max(maxv[k],iv(f,k))
            if iv(f,'ready')==1: ready_seen=True
            healthy=(iv(f,'ready')==1 and iv(f,'system_ok')==1 and iv(f,'baro_valid')==1 and iv(f,'baro_fresh')==1 and iv(f,'lidar_valid')==1 and iv(f,'lidar_fresh')==1 and iv(f,'eskf_ok')==1 and iv(f,'sys_eskf_fresh')==1)
            if healthy:
                healthy_frames+=1
                if healthy_t is None: healthy_t=iv(f,'time_ms')
            elif healthy_t is not None:
                unhealthy_after+=1
            if a.print_every>0 and valid%a.print_every==0: print(status(f))
    finally:
        if log: log.close()
    if latest is None: print("TGY72 frame yok"); return 2
    dur=max(.001,(iv(latest,'time_ms')-iv(first,'time_ms'))/1000.0)
    def delta(k): return max(0,iv(latest,k)-base.get(k,0))
    def rate(k): return delta(k)/dur
    avg=statistics.mean(cpu) if cpu else 0.0; mx=max(cpu) if cpu else 0.0
    print("\n================ R8R13 P73 FAST-REDUNDANT NRF OZET ================")
    print(f"Valid/rejected             : {valid}/{reject}")
    print(f"Capture duration           : {dur:.3f} s")
    print(f"Ready / healthy frames     : {ready_seen} / {healthy_frames}; unhealthy-after={unhealthy_after}")
    print(f"CPU avg/max                : {avg:.2f}% / {mx:.2f}%")
    print(f"NRF hw/link/ch/rf          : {iv(latest,'nrf_connected')}/{iv(latest,'nrf_link')}/{iv(latest,'nrf_channel')}/{iv(latest,'nrf_rf_setup')}")
    print(f"NRF age/errors/invalid     : {iv(latest,'nrf_age_ms')} ms / {iv(latest,'nrf_errors')} / {iv(latest,'nrf_invalid')}")
    print(f"NRF RX/valid delta/rate    : {delta('nrf_rx_count')}/{delta('nrf_valid_count')} | {rate('nrf_valid_count'):.2f} Hz valid")
    print(f"TDD TX success/fail delta  : {delta('nrf_tlm_tx_success')}/{delta('nrf_tlm_tx_fail')} | {rate('nrf_tlm_tx_success'):.2f} Hz success")
    print(f"TDD TX last/max duration   : {iv(latest,'nrf_tlm_last_tx_us')}/{iv(latest,'nrf_tlm_max_tx_us')} us")
    print(f"BG remote max              : {maxv['bg_remote_max_us']} us")
    print(f"NRF task max / CPU         : {maxv['task_nrf_max_us']} us / {pct(latest,'cpu_nrf_x100'):.2f}%")
    print(f"Task max I/B/L/E           : {maxv['task_imu_max_us']}/{maxv['task_baro_max_us']}/{maxv['task_lidar_max_us']}/{maxv['task_eskf_max_us']} us")
    print(f"Cov/SD/UART/ISR max        : {maxv['cov_max_us']}/{maxv['sd_update_max_us']}/{maxv['bg_uart_max_us']}/{maxv['isr_max_us']} us")
    print(f"Miss delta I/B/L/N/E       : {delta('miss_imu')}/{delta('miss_baro')}/{delta('miss_lidar')}/{delta('miss_nrf')}/{delta('miss_eskf')}")
    print(f"Scheduler realigns         : {iv(latest,'scheduler_realigns')}")
    print(f"ESKF public rate           : {rate('eskf_public_count'):.2f} Hz")
    print(f"Safety PWM/moves/RCS/HO    : {iv(latest,'p110_pwm')}/{iv(latest,'p111_moves')}/{iv(latest,'rcs_mask')}/{iv(latest,'hardoff')}")

    pass_ok=(dur>=90 and ready_seen and healthy_frames>0 and unhealthy_after==0 and
             iv(latest,'nrf_connected')==1 and iv(latest,'nrf_link')==1 and iv(latest,'nrf_channel')==76 and iv(latest,'nrf_rf_setup')==6 and
             iv(latest,'nrf_errors')==0 and iv(latest,'nrf_invalid')==0 and rate('nrf_valid_count')>=10.0 and rate('nrf_tlm_tx_success')>=10.0 and delta('nrf_tlm_tx_fail')<=2 and
             delta('miss_imu')==0 and delta('miss_baro')==0 and delta('miss_lidar')==0 and delta('miss_nrf')==0 and delta('miss_eskf')==0 and iv(latest,'scheduler_realigns')==0 and
             maxv['task_nrf_max_us']<80 and maxv['bg_remote_max_us']<400 and maxv['sd_update_max_us']<520 and maxv['bg_uart_max_us']<600 and maxv['isr_max_us']<=250 and mx<75 and
             iv(latest,'p110_pwm')==0 and iv(latest,'p111_moves')==0 and iv(latest,'rcs_mask')==0)
    print("\nSONUC:", "PASS - P73 FAST NRF + R8R11 FROZEN CORE COEXISTENCE" if pass_ok else "PASS DEGIL - TXT LOGU GONDER")
    return 0 if pass_ok else 1

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('capture',nargs='?'); ap.add_argument('--port',default='COM21'); ap.add_argument('--baud',type=int,default=BAUD); ap.add_argument('--duration',type=float,default=120.0); ap.add_argument('--print-every',type=int,default=10); ap.add_argument('--log',default='uart_p112r12r8r13_p73_fast_nrf.txt'); a=ap.parse_args(); raise SystemExit(run(a))
if __name__=='__main__': main()
