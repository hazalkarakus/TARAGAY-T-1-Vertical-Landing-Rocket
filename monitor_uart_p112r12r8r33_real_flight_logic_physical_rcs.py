#!/usr/bin/env python3
"""TARAGAY-T1 P112R12R8R33 physical RCS integration observer."""
from __future__ import annotations
import argparse, json, statistics, sys
from pathlib import Path
import monitor_uart_p112r12r8r32_real_flight_logic_physical_needle as base

BAUD=115200

def iv(f,k):
    try:return int(f.get(k,0))
    except Exception:return 0

def pct(f,k): return iv(f,k)/100.0

def status(f):
    return (f"t={iv(f,'time_ms')/1000:7.2f}s READY={iv(f,'ready')} FLIGHT={iv(f,'flight_active')} AUTH={iv(f,'auth')} "
            f"SYS={iv(f,'system_ok')}/{iv(f,'system_fault')} ESKF={iv(f,'eskf_ok')} CAL={iv(f,'cal_phase')}/{iv(f,'cal_valid')}/{iv(f,'cal_fault')} | "
            f"FL={iv(f,'fl_state')} valid={iv(f,'fl_input_valid')} pitch={iv(f,'fl_pitch_cdeg')/100:.1f} yaw={iv(f,'fl_yaw_cdeg')/100:.1f} | "
            f"RCS req/app/phys=0x{iv(f,'fl_rcs_req_mask'):X}/0x{iv(f,'fl_rcs_applied_mask'):X}/0x{iv(f,'rcs_mask'):X} "
            f"events={iv(f,'fl_v1_events')}/{iv(f,'fl_v3_events')}/{iv(f,'fl_v5_events')}/{iv(f,'fl_v7_events')} | "
            f"needle={iv(f,'needle_adc')} tgt={iv(f,'needle_target_adc')} pwm={iv(f,'p110_pwm')}")

def run(a):
    lines=base.file_lines(a.capture) if a.capture else base.serial_lines(a.port,a.baud,a.duration)
    log=None if a.capture else Path(a.log).open('w',buffering=1)
    valid=reject=0; first=latest=None; cpu=[]; basec={}
    ready=False; fixed=False; flight=False; flight_first=None
    preflight_rcs=False; post_req=False; post_app=False; post_phys=False
    invalid_mask=False; opposing=False; rcs_fault=False
    fl_valid=False; needle_cmd=False; needle_pwm=False; needle_fault=False; p111_fault=False
    apply_frames=0; req_frames=0; phys_frames=0
    try:
        for line in lines:
            if not line.strip().startswith(base.PREFIX): continue
            try:f,raw=base.dec(line)
            except Exception as e:
                reject+=1; print('REJECT',e,file=sys.stderr); continue
            valid+=1; latest=f; first=f if first is None else first
            if log:
                o=dict(f); o['cpu_pct']=pct(f,'cpu_x100'); o['raw_frame']=raw
                log.write(json.dumps(o,sort_keys=True)+'\n')
            if not basec:
                for k in ('miss_imu','miss_baro','miss_lidar','miss_nrf','miss_eskf','scheduler_realigns'):
                    basec[k]=iv(f,k)
            if iv(f,'cpu_x100')>0: cpu.append(pct(f,'cpu_x100'))
            ready |= iv(f,'ready')==1 and iv(f,'system_ok')==1 and iv(f,'eskf_ok')==1
            fixed |= iv(f,'cal_phase')==12 and iv(f,'cal_valid')==1 and iv(f,'cal_fault')==0
            req=iv(f,'fl_rcs_req_mask'); app=iv(f,'fl_rcs_applied_mask'); phys=iv(f,'rcs_mask')
            for m in (req,app,phys):
                invalid_mask |= (m & ~0x0F)!=0
                opposing |= ((m & 0x03)==0x03) or ((m & 0x0C)==0x0C)
            rcs_fault |= iv(f,'fl_rcs_fault')!=0
            if iv(f,'flight_active')==0:
                preflight_rcs |= app!=0 or phys!=0
            else:
                if not flight: flight_first=iv(f,'time_ms')
                flight=True
                fl_valid |= iv(f,'fl_real')==1 and iv(f,'fl_synth')==0 and iv(f,'fl_input_valid')==1
                post_req |= req!=0; post_app |= app!=0; post_phys |= phys!=0
                req_frames += int(req!=0); apply_frames += int(app!=0); phys_frames += int(phys!=0)
                needle_cmd |= iv(f,'needle_cmd_x10000')>0
                needle_pwm |= iv(f,'p110_pwm')>0
            needle_fault |= iv(f,'needle_fault')!=0
            p111_fault |= iv(f,'p111_fault')!=0
            if a.print_every>0 and valid%a.print_every==0: print(status(f))
    finally:
        if log: log.close()
    if latest is None:
        print('TGY73 frame yok'); return 2
    dur=max(.001,(iv(latest,'time_ms')-iv(first,'time_ms'))/1000.0)
    def delta(k):return max(0,iv(latest,k)-basec.get(k,0))
    avg=statistics.mean(cpu) if cpu else 0; mx=max(cpu) if cpu else 0
    print('\n================ R8R33 OZET ================')
    print(f'Valid/rejected              : {valid}/{reject}')
    print(f'Capture duration            : {dur:.3f}s')
    print(f'CPU avg/max                 : {avg:.2f}% / {mx:.2f}%')
    print(f'Ready/fixed matrix          : {ready}/{fixed}')
    print(f'Flight PE9 seen             : {flight} first_ms={flight_first}')
    print(f'FL real valid after flight  : {fl_valid}')
    print(f'Preflight physical RCS      : {preflight_rcs}  (MUST False)')
    print(f'Postflight req/app/phys     : {post_req}/{post_app}/{post_phys}')
    print(f'Req/app/phys frame counts   : {req_frames}/{apply_frames}/{phys_frames}')
    print(f'Invalid/opposing mask       : {invalid_mask}/{opposing}  (MUST False/False)')
    print(f'RCS fault seen              : {rcs_fault}  (MUST False)')
    print(f'Needle cmd/PWM seen         : {needle_cmd}/{needle_pwm}')
    print(f'Needle/P111 fault seen      : {needle_fault}/{p111_fault}')
    print(f'Miss delta I/B/L/N/E        : {delta("miss_imu")}/{delta("miss_baro")}/{delta("miss_lidar")}/{delta("miss_nrf")}/{delta("miss_eskf")}')
    print(f'Scheduler realign delta     : {delta("scheduler_realigns")}')
    ok=(dur>=8 and ready and fixed and flight and fl_valid and not preflight_rcs and post_req and post_app and post_phys and
        not invalid_mask and not opposing and not rcs_fault and needle_cmd and not needle_fault and not p111_fault and
        delta('miss_imu')==0 and delta('miss_baro')==0 and delta('miss_lidar')==0 and delta('miss_eskf')==0 and delta('scheduler_realigns')==0)
    print('\nSONUC:', 'PASS - R8R33 PHYSICAL RCS AUTHORITY' if ok else 'INCELE - R8R33 kosullari tamamlanmadi')
    if flight and not post_req:
        print('NOT: Flight goruldu ancak RCS request yok. Basincsiz/solenoid yuku ayrilmis duzende roketi yavasca egerek RCS istegi olustur.')
    return 0 if ok else 1

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port',default='COM21'); p.add_argument('--baud',type=int,default=BAUD); p.add_argument('--duration',type=float,default=30.0)
    p.add_argument('--log',default='uart_p112r12r8r33_real_flight_logic_physical_rcs.txt'); p.add_argument('--capture'); p.add_argument('--print-every',type=int,default=2)
    return run(p.parse_args())
if __name__=='__main__': raise SystemExit(main())
