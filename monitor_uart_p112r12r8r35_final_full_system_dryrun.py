#!/usr/bin/env python3
"""TARAGAY-T1 P112R12R8R35 final full-system pressureless dry-run observer."""
from __future__ import annotations
import argparse, binascii, json, statistics, sys, time
from pathlib import Path
import monitor_uart_p112r12r8r32_real_flight_logic_physical_needle as base

PREFIX="$TGY73,"; BAUD=115200
EXTRA=(
    "needle_auto_fault","needle_stall_ms","needle_cmd_rejects",
    "p110_state","p110_result","p110_abort_reason","p110_powered_ms",
    "p110_speed_adc_s","p110_stop_distance_adc","p110_corrections",
    "rcs_pulse_complete_count","rcs_last_pulse_ms","rcs_max_pulse_ms",
)
FIELDS=base.FIELDS+EXTRA
N=len(FIELDS)
assert N==240,(N,240)

P110_STATE={0:"WAIT",1:"SEARCH",2:"DRIVE",3:"BRAKE",4:"CORR_DWELL",5:"DONE"}
P110_RESULT={0:"RUNNING",1:"PASS",2:"ABORT",3:"OVERSHOOT",4:"NO_BREAKAWAY",5:"POWER_TIMEOUT"}
AUTO_FAULT={0:"NONE",1:"REFERENCE",2:"FEEDBACK",3:"COMMAND_TIMEOUT",4:"LOW_LEVEL",5:"TARGET_RANGE"}

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
        print("="*118)
        print("R8R35 FINAL FULL-SYSTEM PRESSURELESS DRY-RUN")
        print("1) READY=1 / SYS=1 / ESKF=1 / CAL=12 bekle; PE9 bagliyken needle ve RCS fiziksel cikis 0 olmali.")
        print("2) PE9'u ayir. Needle Hover V19.6 komutunu takip etmeli ve fault vermemeli.")
        print("3) Ilk 4-5 s roketi dik/hareketsiz tut: RCS sessiz. Sonra 4-6 deg yavas eg: RCS fiziksel pulse vermeli.")
        print("4) Bu test BASINCSIZ/DEPRESSURIZED. Ground vent/servo bu build'de fiziksel olarak izole kalir.")
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
            except Exception: pass
        yield line

def status(f):
    return (f"t={iv(f,'time_ms')/1000:7.2f}s READY={iv(f,'ready')} FL={iv(f,'flight_active')} AUTH={iv(f,'auth')} "
            f"SYS={iv(f,'system_ok')}/{iv(f,'system_fault')} ESKF={iv(f,'eskf_ok')} | "
            f"N ADC={iv(f,'needle_adc')} tgt={iv(f,'needle_target_adc')} pwm={iv(f,'p110_pwm')} "
            f"af={iv(f,'needle_auto_fault')} p110={P110_STATE.get(iv(f,'p110_state'),iv(f,'p110_state'))}/"
            f"{P110_RESULT.get(iv(f,'p110_result'),iv(f,'p110_result'))} abort={iv(f,'p110_abort_reason')} | "
            f"att={iv(f,'fl_pitch_cdeg')/100:+.1f}/{iv(f,'fl_yaw_cdeg')/100:+.1f}deg "
            f"RCS req/app/phys=0x{iv(f,'fl_rcs_req_mask'):X}/0x{iv(f,'fl_rcs_applied_mask'):X}/0x{iv(f,'rcs_mask'):X} "
            f"pulse n/last/max={iv(f,'rcs_pulse_complete_count')}/{iv(f,'rcs_last_pulse_ms')}/{iv(f,'rcs_max_pulse_ms')}ms")

def run(a):
    lines=file_lines(a.capture) if a.capture else serial_lines(a.port,a.baud,a.duration)
    log=None if a.capture else Path(a.log).open('w',buffering=1)
    valid=reject=0; first=latest=None; cpu=[]; basec={}
    ready=fixed=flight=fl_valid=False; first_flight_ms=None
    preflight_pwm=preflight_move=preflight_rcs=False
    needle_cmd_seen=needle_pwm_seen=needle_move_seen=False
    needle_fault=False; max_auto_fault=max_needle_fault=max_p111_fault=0
    max_stall=max_powered=max_reject=0; root_diag=None
    quiet_frames=quiet_phys=quiet_req=0; tilt_frames=0; tilt_req=tilt_phys=False
    invalid_mask=opposing=rcs_fault=False; rcs_mismatch=0; rcs_compare=0
    pulse_count_max=pulse_last_max=pulse_dur_max=0
    max_target_travel=max_actual_travel=0
    try:
        for line in lines:
            if not line.strip().startswith(PREFIX): continue
            try:f,raw=dec(line)
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
            ready |= iv(f,'ready')==1 and iv(f,'system_ok')==1 and iv(f,'eskf_ok')==1
            fixed |= iv(f,'cal_phase')==12 and iv(f,'cal_valid')==1 and iv(f,'cal_fault')==0
            req,app,phys=iv(f,'fl_rcs_req_mask'),iv(f,'fl_rcs_applied_mask'),iv(f,'rcs_mask')
            for m in (req,app,phys):
                invalid_mask |= (m & ~0x0F)!=0
                opposing |= ((m & 0x03)==0x03) or ((m & 0x0C)==0x0C)
            rcs_fault |= iv(f,'fl_rcs_fault')!=0
            # R8R35 getter should make fl applied telemetry equal current physical mask.
            rcs_compare += 1
            if app!=phys: rcs_mismatch += 1
            pulse_count_max=max(pulse_count_max,iv(f,'rcs_pulse_complete_count'))
            pulse_last_max=max(pulse_last_max,iv(f,'rcs_last_pulse_ms'))
            pulse_dur_max=max(pulse_dur_max,iv(f,'rcs_max_pulse_ms'))
            ref,adc,tgt=iv(f,'needle_closed_ref_adc'),iv(f,'needle_adc'),iv(f,'needle_target_adc')
            if ref>0:
                max_target_travel=max(max_target_travel,max(0,ref-tgt))
                max_actual_travel=max(max_actual_travel,max(0,ref-adc))
            max_auto_fault=max(max_auto_fault,iv(f,'needle_auto_fault'))
            max_needle_fault=max(max_needle_fault,iv(f,'needle_fault'))
            max_p111_fault=max(max_p111_fault,iv(f,'p111_fault'))
            max_stall=max(max_stall,iv(f,'needle_stall_ms')); max_powered=max(max_powered,iv(f,'p110_powered_ms'))
            max_reject=max(max_reject,iv(f,'needle_cmd_rejects'))
            if iv(f,'needle_auto_fault')!=0 or iv(f,'needle_fault')!=0:
                needle_fault=True
                root_diag=(iv(f,'time_ms'),iv(f,'needle_auto_fault'),iv(f,'needle_fault'),iv(f,'p110_state'),iv(f,'p110_result'),iv(f,'p110_abort_reason'),iv(f,'p110_powered_ms'),iv(f,'needle_error_adc'))
            if iv(f,'flight_active')==0:
                preflight_pwm |= iv(f,'p110_pwm')!=0
                preflight_move |= iv(f,'needle_move_in_progress')!=0
                preflight_rcs |= phys!=0
            else:
                if not flight:first_flight_ms=iv(f,'time_ms')
                flight=True
                fl_valid |= iv(f,'fl_real')==1 and iv(f,'fl_synth')==0 and iv(f,'fl_input_valid')==1
                needle_cmd_seen |= iv(f,'needle_cmd_x10000')>0
                needle_pwm_seen |= iv(f,'p110_pwm')>0
                needle_move_seen |= iv(f,'needle_move_in_progress')>0 or iv(f,'p111_moves')>basec.get('p111_moves',0)
                pitch=abs(iv(f,'fl_pitch_cdeg')/100.0); yaw=abs(iv(f,'fl_yaw_cdeg')/100.0)
                pr=abs(iv(f,'fl_pitch_rate_cdeg_s')/100.0); yr=abs(iv(f,'fl_yaw_rate_cdeg_s')/100.0)
                quiet=(pitch<=2.0 and yaw<=2.0 and pr<=1.5 and yr<=1.5)
                if quiet:
                    quiet_frames+=1; quiet_req+=int(req!=0); quiet_phys+=int(phys!=0)
                deliberate=(pitch>=3.0 or yaw>=3.0)
                if deliberate:
                    tilt_frames+=1; tilt_req|=req!=0; tilt_phys|=phys!=0
            if a.print_every>0 and valid%a.print_every==0: print(status(f))
    finally:
        if log:log.close()
    if latest is None:
        print('TGY73 frame yok'); return 2
    dur=max(.001,(iv(latest,'time_ms')-iv(first,'time_ms'))/1000.0)
    def delta(k): return max(0,iv(latest,k)-basec.get(k,0))
    avg=statistics.mean(cpu) if cpu else 0.0; mx=max(cpu) if cpu else 0.0
    pulse_ok=(pulse_count_max>=1 and 30<=pulse_dur_max<=70)
    quiet_ok=(quiet_frames>=5 and quiet_phys==0)
    tilt_ok=(tilt_frames>=3 and tilt_req and tilt_phys)
    rcs_telem_ok=(rcs_compare>0 and rcs_mismatch==0)
    misses_ok=(delta('miss_imu')==0 and delta('miss_baro')==0 and delta('miss_lidar')==0 and delta('miss_eskf')==0 and delta('scheduler_realigns')==0)
    needle_ok=(needle_cmd_seen and needle_pwm_seen and needle_move_seen and not needle_fault and max_auto_fault==0 and max_needle_fault==0 and max_target_travel<=585)
    ok=(dur>=12 and ready and fixed and flight and fl_valid and not preflight_pwm and not preflight_move and not preflight_rcs and
        needle_ok and quiet_ok and tilt_ok and not invalid_mask and not opposing and not rcs_fault and rcs_telem_ok and pulse_ok and misses_ok)
    print('\n================ R8R35 FINAL DRY-RUN OZET ================')
    print(f'Valid/rejected                 : {valid}/{reject}')
    print(f'Capture duration               : {dur:.3f}s')
    print(f'CPU avg/max                    : {avg:.2f}% / {mx:.2f}%')
    print(f'Ready/fixed/flight/FL valid    : {ready}/{fixed}/{flight}/{fl_valid} first_flight_ms={first_flight_ms}')
    print(f'Preflight PWM/move/RCS         : {preflight_pwm}/{preflight_move}/{preflight_rcs} (MUST False/False/False)')
    print(f'Needle cmd/PWM/move            : {needle_cmd_seen}/{needle_pwm_seen}/{needle_move_seen}')
    print(f'Needle max target/actual travel: {max_target_travel}/{max_actual_travel} ADC')
    print(f'Needle auto/legacy fault max   : {max_auto_fault}/{max_needle_fault}; p111_abort_max={max_p111_fault}')
    print(f'Needle stall/powered/reject max: {max_stall}/{max_powered} ms / {max_reject}')
    if root_diag:
        t,af,nf,st,res,ab,pw,err=root_diag
        print(f'Needle ROOT DIAG                : t={t}ms auto={af}:{AUTO_FAULT.get(af,"?")} needle={nf} p110={st}:{P110_STATE.get(st,"?")} result={res}:{P110_RESULT.get(res,"?")} abort={ab} powered={pw}ms err={err}ADC')
    print(f'Quiet frames req/phys          : {quiet_frames}/{quiet_req}/{quiet_phys}')
    print(f'Deliberate tilt frames req/phys: {tilt_frames}/{tilt_req}/{tilt_phys}')
    print(f'RCS applied telemetry mismatch : {rcs_mismatch}/{rcs_compare} (MUST 0)')
    print(f'RCS pulse count/last/max       : {pulse_count_max}/{pulse_last_max}/{pulse_dur_max} ms (expect ~40 ms, gate 30..70)')
    print(f'Invalid/opposing/RCS fault      : {invalid_mask}/{opposing}/{rcs_fault}')
    print(f'Miss delta I/B/L/N/E           : {delta("miss_imu")}/{delta("miss_baro")}/{delta("miss_lidar")}/{delta("miss_nrf")}/{delta("miss_eskf")}')
    print(f'Scheduler realign delta        : {delta("scheduler_realigns")}')
    print('\nSONUC:', 'PASS - R8R35 FINAL FULL-SYSTEM PRESSURELESS DRY-RUN' if ok else 'INCELE - R8R35 final dry-run gate tamamlanmadi')
    if not pulse_ok: print('NOT: En az bir tamamlanmis fiziksel RCS pulse ~40 ms olmali; 30..70 ms gate kullaniliyor.')
    if not quiet_ok: print('NOT: PE9 sonrasi dik/hareketsiz bolumde RCS fiziksel cikis olmamali.')
    if not tilt_ok: print('NOT: Quiet bolumden sonra 4-6 derece kontrollu egim ile fiziksel RCS cevabi olustur.')
    if needle_fault: print('UYARI: Needle fault tekrarlandi; ROOT DIAG satiri low-level nedeni ayirmak icin eklendi.')
    return 0 if ok else 1

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port',default='COM21'); p.add_argument('--baud',type=int,default=BAUD); p.add_argument('--duration',type=float,default=35.0)
    p.add_argument('--log',default='uart_p112r12r8r35_final_full_system_dryrun.txt'); p.add_argument('--capture'); p.add_argument('--print-every',type=int,default=2)
    return run(p.parse_args())
if __name__=='__main__': raise SystemExit(main())
