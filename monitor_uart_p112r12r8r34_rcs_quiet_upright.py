#!/usr/bin/env python3
"""TARAGAY-T1 P112R12R8R34 quiet-upright / anti-chatter observer."""
from __future__ import annotations
import argparse, json, statistics, sys
from pathlib import Path
import monitor_uart_p112r12r8r33_real_flight_logic_physical_rcs as r33
import monitor_uart_p112r12r8r32_real_flight_logic_physical_needle as base

BAUD = 115200

def iv(f,k):
    try: return int(f.get(k,0))
    except Exception: return 0

def pct(f,k): return iv(f,k)/100.0

def status(f):
    return (f"t={iv(f,'time_ms')/1000:7.2f}s READY={iv(f,'ready')} FLIGHT={iv(f,'flight_active')} AUTH={iv(f,'auth')} "
            f"SYS={iv(f,'system_ok')}/{iv(f,'system_fault')} ESKF={iv(f,'eskf_ok')} | "
            f"pitch/yaw={iv(f,'fl_pitch_cdeg')/100:+.2f}/{iv(f,'fl_yaw_cdeg')/100:+.2f}deg "
            f"rate={iv(f,'fl_pitch_rate_cdeg_s')/100:+.2f}/{iv(f,'fl_yaw_rate_cdeg_s')/100:+.2f}dps | "
            f"RCS req/app/phys=0x{iv(f,'fl_rcs_req_mask'):X}/0x{iv(f,'fl_rcs_applied_mask'):X}/0x{iv(f,'rcs_mask'):X}")

def run(a):
    lines = base.file_lines(a.capture) if a.capture else base.serial_lines(a.port,a.baud,a.duration)
    log = None if a.capture else Path(a.log).open('w',buffering=1)
    valid=reject=0; first=latest=None; cpu=[]; basec={}
    ready=fixed=flight=fl_valid=False; flight_first=None
    preflight_phys=False; invalid_mask=False; opposing=False; rcs_fault=False
    quiet_frames=0; quiet_phys_frames=0; quiet_req_frames=0
    tilt_frames=0; tilt_req=False; tilt_phys=False
    max_abs_pitch=max_abs_yaw=max_abs_pr=max_abs_yr=0.0
    try:
        for line in lines:
            if not line.strip().startswith(base.PREFIX): continue
            try: f,raw=base.dec(line)
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
                preflight_phys |= phys!=0
            else:
                if not flight: flight_first=iv(f,'time_ms')
                flight=True
                fl_valid |= iv(f,'fl_real')==1 and iv(f,'fl_synth')==0 and iv(f,'fl_input_valid')==1
                pitch=abs(iv(f,'fl_pitch_cdeg')/100.0); yaw=abs(iv(f,'fl_yaw_cdeg')/100.0)
                pr=abs(iv(f,'fl_pitch_rate_cdeg_s')/100.0); yr=abs(iv(f,'fl_yaw_rate_cdeg_s')/100.0)
                max_abs_pitch=max(max_abs_pitch,pitch); max_abs_yaw=max(max_abs_yaw,yaw)
                max_abs_pr=max(max_abs_pr,pr); max_abs_yr=max(max_abs_yr,yr)
                # Quiet-upright observation window is state-based, not clock-based:
                # <=2 deg and <=1.5 dps on both axes must not physically pulse.
                quiet=(pitch<=2.0 and yaw<=2.0 and pr<=1.5 and yr<=1.5)
                if quiet:
                    quiet_frames+=1
                    quiet_req_frames += int(req!=0)
                    quiet_phys_frames += int(phys!=0)
                deliberate=(pitch>=3.0 or yaw>=3.0)
                if deliberate:
                    tilt_frames+=1
                    tilt_req |= req!=0
                    tilt_phys |= phys!=0
            if a.print_every>0 and valid%a.print_every==0: print(status(f))
    finally:
        if log: log.close()
    if latest is None:
        print('TGY73 frame yok'); return 2
    dur=max(.001,(iv(latest,'time_ms')-iv(first,'time_ms'))/1000.0)
    def delta(k): return max(0,iv(latest,k)-basec.get(k,0))
    avg=statistics.mean(cpu) if cpu else 0.0; mx=max(cpu) if cpu else 0.0
    print('\n================ R8R34 OZET ================')
    print(f'Valid/rejected              : {valid}/{reject}')
    print(f'Capture duration            : {dur:.3f}s')
    print(f'CPU avg/max                 : {avg:.2f}% / {mx:.2f}%')
    print(f'Ready/fixed matrix          : {ready}/{fixed}')
    print(f'Flight/FL real valid        : {flight}/{fl_valid} first_ms={flight_first}')
    print(f'Preflight physical RCS      : {preflight_phys}  (MUST False)')
    print(f'Quiet frames req/phys       : {quiet_frames} / {quiet_req_frames} / {quiet_phys_frames}')
    print(f'Deliberate tilt frames      : {tilt_frames}; req/phys={tilt_req}/{tilt_phys}')
    print(f'Max |pitch/yaw| deg         : {max_abs_pitch:.2f}/{max_abs_yaw:.2f}')
    print(f'Max |pitch/yaw rate| dps    : {max_abs_pr:.2f}/{max_abs_yr:.2f}')
    print(f'Invalid/opposing mask       : {invalid_mask}/{opposing}  (MUST False/False)')
    print(f'RCS fault                   : {rcs_fault}  (MUST False)')
    print(f'Miss delta I/B/L/N/E        : {delta("miss_imu")}/{delta("miss_baro")}/{delta("miss_lidar")}/{delta("miss_nrf")}/{delta("miss_eskf")}')
    print(f'Scheduler realign delta     : {delta("scheduler_realigns")}')
    quiet_ok=(quiet_frames>=5 and quiet_phys_frames==0)
    tilt_ok=(tilt_frames>=3 and tilt_req and tilt_phys)
    ok=(dur>=8 and ready and fixed and flight and fl_valid and not preflight_phys and quiet_ok and tilt_ok and
        not invalid_mask and not opposing and not rcs_fault and
        delta('miss_imu')==0 and delta('miss_baro')==0 and delta('miss_lidar')==0 and
        delta('miss_eskf')==0 and delta('scheduler_realigns')==0)
    print('\nSONUC:', 'PASS - R8R34 QUIET UPRIGHT + DELIBERATE TILT RCS' if ok else 'INCELE - R8R34 kosullari tamamlanmadi')
    if quiet_frames<5: print('NOT: PE9 sonrasi roketi birkac saniye gercekten dik ve hareketsiz tut.')
    if quiet_phys_frames>0: print('UYARI: Dik/hareketsiz kosulda fiziksel RCS pulse goruldu.')
    if not tilt_ok: print('NOT: Quiet bolumden sonra roketi yavasca 4-6 derece egerek fiziksel RCS cevabini olustur.')
    return 0 if ok else 1

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port',default='COM21'); p.add_argument('--baud',type=int,default=BAUD); p.add_argument('--duration',type=float,default=30.0)
    p.add_argument('--log',default='uart_p112r12r8r34_rcs_quiet_upright.txt'); p.add_argument('--capture'); p.add_argument('--print-every',type=int,default=2)
    return run(p.parse_args())
if __name__=='__main__': raise SystemExit(main())
