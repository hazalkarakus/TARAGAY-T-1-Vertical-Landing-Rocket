#!/usr/bin/env python3
"""TARAGAY-T1 P112R12R8R2 LiDAR bus-clear validation observer - READ ONLY.

Uses P112R12R8R2 production-candidate firmware with bounded I2C2 GPIO bus-clear recovery. This script does not transmit commands and does not
modify the STM32. It only watches whether a previously-valid LiDAR becomes
persistently invalid and whether the vertical estimator degrades/falls back.
"""
from __future__ import annotations
import argparse, binascii, json, sys, time
from pathlib import Path

PREFIX = "$TGY68,"
N = 498
BAUD = 115200
I = {
    "seq":1,"time_ms":2,"preflight_state":3,"flight_active":4,
    "pe9_raw_open":5,"pe9_debounced_open":6,"preflight_ready":7,"preflight_fault":8,
    "connector_seen":10,"actuator_authorized":11,"imu_valid":12,
    "baro_valid":89,"lidar_valid":90,"eskf_init":91,"eskf_ok":92,"eskf_inhibit":93,"eskf_reacquire":94,
    "vertical_source_mask":132,"vertical_degraded":133,
    "rcs_req_mask":176,"rcs_applied_mask":177,
    "main_cmd_x10000":190,"main_output_valid":194,"needle_req_x10000":196,
    "needle_adc":198,"needle_target_adc":199,"needle_enabled":200,"needle_fault":202,"needle_zero_adc":203,
    "stop_latched":225,"sd_ready":229,"system_fault":322,"system_lidar_fresh":324,
    "lidar_state":155,"lidar_age_ms":156,"lidar_reads":157,"lidar_errors":158,"lidar_timeouts":159,"lidar_wait_timeouts":160,
    "lidar_recoveries":161,"lidar_rec_active":162,"lidar_rec_step":163,"lidar_rec_attempts":164,"lidar_rec_success":165,"lidar_rec_failures":166,
    "lidar_rec_last_us":167,"lidar_rec_max_us":168,"lidar_rec_step_max_us":169,
    "cpu_lidar_x100":297,"task_lidar_us":302,"task_lidar_max_us":307,"miss_lidar":312,
    "p83_feedback_valid":344,"p83_confidence_pct":345,
    "p110_state":435,"p110_result":436,"p110_active_pwm":442,"p110_abort_reason":457,
    "p111_state":458,"p111_moves_completed":464,"p111_baseline_adc":465,"p111_target_adc":466,"p111_fault":481,
    "p112_hard_off_count":484,"p112_isr_max_us":488,
}
S111={0:"WAIT_REFERENCE",1:"READY",2:"TRACKING",3:"HOLD",4:"REVOKED",5:"FAULT"}
S110={0:"WAIT",1:"SEARCH",2:"DRIVE",3:"BRAKE",4:"CORRECTION_DWELL",5:"DONE"}
R110={0:"RUNNING",1:"PASS",2:"ABORT",3:"OVERSHOOT",4:"NO_BREAKAWAY",5:"POWER_TIMEOUT"}

def n(v):
    try: return int(v,0)
    except Exception:
        try: return float(v)
        except Exception: return v

def iv(f,k):
    try: return int(f.get(k,0))
    except Exception: return 0

def dec(line):
    s=line.strip()
    if not s.startswith(PREFIX) or '*' not in s:
        raise ValueError('not TGY68')
    body,crc=s.rsplit('*',1)
    rx=int(crc,16)
    calc=binascii.crc_hqx(body.encode('ascii'),0xffff)
    if rx!=calc:
        raise ValueError(f'CRC {rx:04X}!={calc:04X}')
    a=body.split(',')
    if len(a)!=N:
        raise ValueError(f'fields {len(a)} != {N}')
    return {k:n(a[idx]) for k,idx in I.items()}, s

def serial_lines(port,baud,duration):
    try:
        import serial
    except ImportError:
        print('Kur: py -m pip install pyserial')
        raise
    t0=time.monotonic()
    with serial.Serial(port,baudrate=baud,timeout=.5) as s:
        try:s.reset_input_buffer()
        except Exception:pass
        print('='*104)
        print('P112R12R8R2 LIDAR BUS-CLEAR VALIDATION / READ ONLY UART')
        print('Tamamen BASINCSIZ/INERT bench. Gaz/propulsion/pyro/ignition yok; RCS/vent fiziksel yukleri izole.')
        print('READY_PE9_CONNECTED gorulmeden PE9 ayirma. Sonra sisteme dokunmadan testi tamamla.')
        print('='*104)
        while duration<=0 or time.monotonic()-t0<duration:
            b=s.readline()
            if b:
                yield b.decode('ascii',errors='replace')

def file_lines(path):
    for line in Path(path).open(errors='replace'):
        q=line.strip()
        if q.startswith('{'):
            try:
                d=json.loads(q)
                if isinstance(d.get('raw_frame'),str):
                    yield d['raw_frame']; continue
            except Exception: pass
        yield line

def status(f, tag=''):
    return (
        f"t={iv(f,'time_ms')/1000:7.2f}s {tag:>16} | "
        f"READY={iv(f,'preflight_ready')} PE9={'OPEN' if iv(f,'pe9_debounced_open') else 'CONN'} AUTH={iv(f,'actuator_authorized')} SD={iv(f,'sd_ready')} SYSF={iv(f,'system_fault')} | "
        f"IMU/BARO/LID/ESKF={iv(f,'imu_valid')}/{iv(f,'baro_valid')}/{iv(f,'lidar_valid')}/{iv(f,'eskf_ok')} "
        f"SRC=0x{iv(f,'vertical_source_mask'):02X} DEG={iv(f,'vertical_degraded')} | "
        f"GNC={iv(f,'main_cmd_x10000')/10000:.4f} VALID={iv(f,'main_output_valid')} REQ={iv(f,'needle_req_x10000')/10000:.4f} | "
        f"ADC={iv(f,'needle_adc')} TGT={iv(f,'needle_target_adc')} P111={S111.get(iv(f,'p111_state'),iv(f,'p111_state'))} F={iv(f,'p111_fault')} | "
        f"P110={S110.get(iv(f,'p110_state'),iv(f,'p110_state'))}/{R110.get(iv(f,'p110_result'),iv(f,'p110_result'))} PWM={iv(f,'p110_active_pwm')} MOV={iv(f,'p111_moves_completed')} | "
        f"LST={iv(f,'lidar_state')} age={iv(f,'lidar_age_ms')} read={iv(f,'lidar_reads')} err/to={iv(f,'lidar_errors')}/{iv(f,'lidar_timeouts')} "
        f"REC={iv(f,'lidar_rec_active')}/{iv(f,'lidar_rec_step')} att/s/f={iv(f,'lidar_rec_attempts')}/{iv(f,'lidar_rec_success')}/{iv(f,'lidar_rec_failures')} stepmax={iv(f,'lidar_rec_step_max_us')}us | "
        f"RCS={iv(f,'rcs_applied_mask'):02X} HO={iv(f,'p112_hard_off_count')} ISRmax={iv(f,'p112_isr_max_us')}us"
    )

def run(a):
    if a.capture:
        lines=file_lines(a.capture); log=None
    else:
        lines=serial_lines(a.port,a.baud,a.duration)
        log=Path(a.log).open('w',buffering=1)
        log.write(f'# P112R12R8R2 lidar bus-clear validation | port={a.port} baud={a.baud}\n')

    valid=reject=0
    latest=None
    ready=pe9=auth=sd=False
    lidar_ever_valid=False
    lidar_drop_start=None
    lidar_drop_events=[]
    persistent_drop=False
    recovery_events=[]
    last_lid=None
    last_src=None
    last_deg=None
    max_pwm=max_moves=max_hardoff=max_isr=0
    rcs_applied=False
    actuator_fault=False
    system_fault_after_ready=False
    healthy_post_auth=False

    try:
        for line in lines:
            if not line.strip().startswith(PREFIX):
                continue
            try:
                f,raw=dec(line)
            except Exception as e:
                reject+=1
                print('REJECT',e,file=sys.stderr)
                continue
            valid+=1; latest=f
            t=iv(f,'time_ms')
            ready |= bool(iv(f,'preflight_ready'))
            pe9 |= bool(iv(f,'pe9_debounced_open'))
            auth |= bool(iv(f,'actuator_authorized'))
            sd |= bool(iv(f,'sd_ready'))
            max_pwm=max(max_pwm,iv(f,'p110_active_pwm'))
            max_moves=max(max_moves,iv(f,'p111_moves_completed'))
            max_hardoff=max(max_hardoff,iv(f,'p112_hard_off_count'))
            max_isr=max(max_isr,iv(f,'p112_isr_max_us'))
            rcs_applied |= bool(iv(f,'rcs_applied_mask'))
            actuator_fault |= bool(iv(f,'needle_fault') or iv(f,'p111_fault'))
            if ready and iv(f,'system_fault'):
                system_fault_after_ready=True
            if iv(f,'actuator_authorized') and iv(f,'main_output_valid') and iv(f,'vertical_source_mask') and not iv(f,'system_fault'):
                healthy_post_auth=True

            lid=iv(f,'lidar_valid')
            src=iv(f,'vertical_source_mask')
            deg=iv(f,'vertical_degraded')
            if lid:
                lidar_ever_valid=True
                if lidar_drop_start is not None:
                    dur=t-lidar_drop_start
                    recovery_events.append((t,dur))
                    print(f'\n>>> LIDAR RECOVERED at {t/1000:.3f}s after {dur} ms invalid')
                    lidar_drop_start=None
            elif lidar_ever_valid and lidar_drop_start is None:
                lidar_drop_start=t
                lidar_drop_events.append(t)
                print(f'\n>>> LIDAR DROPOUT START at {t/1000:.3f}s')

            if lidar_drop_start is not None and (t-lidar_drop_start)>=a.persistent_ms:
                persistent_drop=True

            transition = (last_lid is not None and lid!=last_lid) or (last_src is not None and src!=last_src) or (last_deg is not None and deg!=last_deg)
            tag=''
            if transition:
                tag='SENSOR_CHANGE'
                print('\n>>> SENSOR/SOURCE TRANSITION')
            if transition or valid<=3 or (valid%a.print_every)==0:
                print(status(f,tag),flush=True)
            last_lid,last_src,last_deg=lid,src,deg

            if log:
                d=dict(f); d['raw_frame']=raw; log.write(json.dumps(d,sort_keys=True)+'\n')
    except KeyboardInterrupt:
        print('\nDurduruldu.')
    finally:
        if log: log.close()

    print('\n================ P112R12R8R2 LiDAR BUS-CLEAR OZET ================')
    print('Valid / rejected         :',valid,'/',reject)
    print('Ready / PE9 / AUTH / SD  :',ready,pe9,auth,sd)
    print('Healthy post-auth frame  :',healthy_post_auth)
    print('LiDAR ever valid         :',lidar_ever_valid)
    print('LiDAR dropout count      :',len(lidar_drop_events))
    print('Dropout starts [s]       :',[round(x/1000,3) for x in lidar_drop_events])
    print('Recoveries (t,dur ms)    :',[(round(t/1000,3),dur) for t,dur in recovery_events])
    if latest and lidar_drop_start is not None:
        print('Final ongoing dropout ms :',iv(latest,'time_ms')-lidar_drop_start)
    print('Persistent dropout >=ms  :',a.persistent_ms,'=>',persistent_drop)
    print('Max PWM / moves          :',max_pwm,'/',max_moves)
    print('Actuator fault seen      :',actuator_fault)
    print('RCS applied != 0         :',rcs_applied)
    print('Hard-off max             :',max_hardoff)
    print('System fault after ready :',system_fault_after_ready)
    print('ISR max us               :',max_isr)
    if latest:
        print('LiDAR reads/errors/to     :',iv(latest,'lidar_reads'),iv(latest,'lidar_errors'),iv(latest,'lidar_timeouts'))
        print('Recovery att/success/fail :',iv(latest,'lidar_rec_attempts'),iv(latest,'lidar_rec_success'),iv(latest,'lidar_rec_failures'))
        print('Recovery step/max us      :',iv(latest,'lidar_rec_step'),iv(latest,'lidar_rec_step_max_us'))
        print('Task LiDAR max us         :',iv(latest,'task_lidar_max_us'))
    if latest:
        print('Final LID/SRC/DEG        :',iv(latest,'lidar_valid'),f"0x{iv(latest,'vertical_source_mask'):02X}",iv(latest,'vertical_degraded'))
        print('Final SYSF/P111/NF/PWM   :',iv(latest,'system_fault'),iv(latest,'p111_fault'),iv(latest,'needle_fault'),iv(latest,'p110_active_pwm'))

    if not latest:
        print('\nSONUC: VERI YOK')
        return 2
    base_fail = (not ready or not sd or (pe9 and not auth) or not healthy_post_auth or actuator_fault or rcs_applied or max_hardoff or max_pwm or max_moves or iv(latest,'system_fault')!=0)
    if base_fail:
        print('\nSONUC: DRY-RUN TEMEL KRITERLERI TAM PASS DEGIL - TXT LOGU GONDER')
        return 1
    if persistent_drop:
        print('\nSONUC: FAIL - LiDAR persistent dropout survived R12R8R2 bus-clear; TXT logu gonder.')
        return 3
    if lidar_drop_events:
        print('\nSONUC: RECOVERY PASS/REVIEW - LiDAR dropout oldu ve geri geldi; TXT logu gonder.')
        return 4
    print('\nSONUC: PASS - R12R8R2 LiDAR 90s stability dry-run temiz; dropout yok.')
    return 0

def main():
    p=argparse.ArgumentParser()
    p.add_argument('capture',nargs='?')
    p.add_argument('--port',default='COM21')
    p.add_argument('--baud',type=int,default=BAUD)
    p.add_argument('--duration',type=float,default=90)
    p.add_argument('--persistent-ms',type=int,default=1000)
    p.add_argument('--print-every',type=int,default=10)
    p.add_argument('--log',default='uart_p112r12r8r1_lidar_stability.txt')
    raise SystemExit(run(p.parse_args()))

if __name__=='__main__':
    main()
