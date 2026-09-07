#!/usr/bin/env python3
"""TARAGAY-T1 P112R12R3 clean production-autonomous 950+ INERT monitor - READ ONLY."""
from __future__ import annotations
import argparse,binascii,json,sys,time
from pathlib import Path

PREFIX="$TGY68,"; N=498; BAUD=115200
I={
"seq":1,"time_ms":2,"preflight_state":3,"flight_active":4,"pe9_raw_open":5,"pe9_debounced_open":6,"preflight_ready":7,"preflight_fault":8,"connector_seen":10,"actuator_authorized":11,
"imu_valid":12,"baro_valid":89,"lidar_valid":90,"eskf_init":91,"eskf_ok":92,"eskf_inhibit":93,"eskf_reacquire":94,
"vertical_source_mask":132,"vertical_degraded":133,
"rcs_req_mask":176,"rcs_applied_mask":177,"rcs_flight_authorized":187,"rcs_safety_inhibited":188,
"main_cmd_x10000":190,"main_burn_active":193,"main_output_valid":194,"main_physical_enabled":195,"needle_req_x10000":196,"needle_limited_x10000":197,
"needle_adc":198,"needle_target_adc":199,"needle_enabled":200,"needle_fault":202,"needle_zero_adc":203,"needle_error_adc":204,"needle_zero_valid":207,
"system_fault":322,"p83_filtered_adc":343,"p83_feedback_valid":344,"p83_confidence_pct":345,"p83_mode":350,
"p110_state":435,"p110_result":436,"p110_direction":437,"p110_start_adc":438,"p110_target_adc":439,"p110_current_adc":440,"p110_error_adc":441,"p110_active_pwm":442,"p110_learned_breakaway_pwm":443,"p110_sustain_pwm":444,"p110_speed_adc_s":445,"p110_stop_distance_adc":446,"p110_brake_entry_adc":447,"p110_coast_max_adc":448,"p110_final_adc":449,"p110_final_error_adc":450,"p110_total_powered_ms":454,"p110_correction_count":455,"p110_breakaway_found":456,"p110_abort_reason":457,
"p111_state":458,"p111_result":459,"p111_command_sequence":460,"p111_moves_completed":464,"p111_baseline_adc":465,"p111_target_adc":466,"p111_learned_open_pwm":467,"p111_learned_close_pwm":468,"p111_learned_open_coast":469,"p111_learned_close_coast":470,"p111_last_open_error_adc":471,"p111_last_close_error_adc":472,"p111_fault":481,
"p112_control_ticks":482,"p112_adc_samples":483,"p112_hard_off_count":484,"p112_uart_suppressed_count":485,"p112_sd_suppressed_count":486,"p112_isr_last_us":487,"p112_isr_max_us":488,"p112_timing_critical":489,"p112_decel_active":490,"p112_decel_pwm":491,
}
S111={0:"WAIT_REFERENCE",1:"READY",2:"TRACKING",3:"HOLD",4:"REVOKED",5:"FAULT"}
S110={0:"WAIT",1:"SEARCH",2:"DRIVE",3:"BRAKE",4:"CORRECTION_DWELL",5:"DONE"}
R110={0:"RUNNING",1:"PASS",2:"ABORT",3:"OVERSHOOT",4:"NO_BREAKAWAY",5:"POWER_TIMEOUT"}
M83={0:"BOOT",1:"NORMAL",2:"QUARANTINE",3:"TRAJECTORY"}

def n(v):
 try:return int(v,0)
 except:
  try:return float(v)
  except:return v

def iv(f,k):
 try:return int(f.get(k,0))
 except:return 0

def dec(line):
 s=line.strip()
 if not s.startswith(PREFIX) or '*' not in s: raise ValueError('not TGY68')
 b,c=s.rsplit('*',1); rx=int(c,16); cc=binascii.crc_hqx(b.encode('ascii'),0xffff)
 if rx!=cc: raise ValueError(f'CRC {rx:04X}!={cc:04X}')
 a=b.split(',')
 if len(a)!=N: raise ValueError(f'fields {len(a)} != {N}')
 return {k:n(a[x]) for k,x in I.items()},s

def phase(f):
 if iv(f,'needle_fault') or iv(f,'p111_fault'): return 'ACTUATOR_FAULT'
 if iv(f,'needle_zero_valid')==0: return 'WAIT_REFERENCE'
 if iv(f,'preflight_ready')==0: return 'WAIT_PREFLIGHT'
 if iv(f,'pe9_debounced_open')==0: return 'READY_PE9_CONNECTED'
 if iv(f,'actuator_authorized')==0: return 'WAIT_AUTH'
 if iv(f,'p110_state') in (1,2,3,4): return 'MOTOR_MOVING'
 if abs(iv(f,'p111_target_adc')-iv(f,'p111_baseline_adc'))>8: return 'NONZERO_TARGET'
 return 'AUTHORIZED_CLOSED'

def status(f):
 return (f"t={iv(f,'time_ms')/1000:7.2f}s {phase(f):>20} | "
 f"PE9={'OPEN' if iv(f,'pe9_debounced_open') else 'CONN'} READY={iv(f,'preflight_ready')} AUTH={iv(f,'actuator_authorized')} SYSF={iv(f,'system_fault')} | "
 f"GNC={iv(f,'main_cmd_x10000')/10000:0.4f} VALID={iv(f,'main_output_valid')} SRC=0x{iv(f,'vertical_source_mask'):02X} BURN={iv(f,'main_burn_active')} | "
 f"ADC={iv(f,'needle_adc'):4d} ZERO={iv(f,'needle_zero_adc'):4d} TGT={iv(f,'needle_target_adc'):4d} REQ={iv(f,'needle_req_x10000')/10000:0.4f} NF={iv(f,'needle_fault')} | "
 f"P83={iv(f,'p83_feedback_valid')}/{M83.get(iv(f,'p83_mode'),iv(f,'p83_mode'))} C={iv(f,'p83_confidence_pct'):2d}% | "
 f"P111={S111.get(iv(f,'p111_state'),iv(f,'p111_state'))} MOV={iv(f,'p111_moves_completed')} F={iv(f,'p111_fault')} | "
 f"P110={S110.get(iv(f,'p110_state'),iv(f,'p110_state'))}/{R110.get(iv(f,'p110_result'),iv(f,'p110_result'))} PWM={iv(f,'p110_active_pwm'):3d} BRK={iv(f,'p110_breakaway_found')} | "
 f"RCS={iv(f,'rcs_req_mask'):02X}/{iv(f,'rcs_applied_mask'):02X} HO={iv(f,'p112_hard_off_count')} ISR={iv(f,'p112_isr_last_us'):3d}/{iv(f,'p112_isr_max_us'):3d}us")

def serial_lines(port,baud,dur):
 try: import serial
 except ImportError: print('Kur: py -m pip install pyserial'); raise
 t=time.monotonic()
 with serial.Serial(port,baudrate=baud,timeout=.5) as s:
  try:s.reset_input_buffer()
  except:pass
  print('='*86)
  print('P112R12R3 CLEAN PRODUCTION AUTONOMOUS 950+ / INERT - READ ONLY UART')
  print('BASINCSIZ/INERT. RCS+VENT fiziksel yukleri izole. Sentetik GNC hareketi YOK.')
  print('PE9 sadece preflight READY sonrasi inert bench dogrulamasi icin ayrilir.')
  print('='*86)
  while dur<=0 or time.monotonic()-t<dur:
   b=s.readline()
   if b: yield b.decode('ascii',errors='replace')

def file_lines(path):
 for l in Path(path).open(errors='replace'):
  q=l.strip()
  if q.startswith('{'):
   try:
    d=json.loads(q)
    if isinstance(d.get('raw_frame'),str): yield d['raw_frame']; continue
   except: pass
  yield l

def run(a):
 if a.capture: lines=file_lines(a.capture); log=None
 else:
  lines=serial_lines(a.port,a.baud,a.duration); log=Path(a.log).open('w',buffering=1)
  log.write(f'# P112R12R3 clean production autonomous 950+ inert | port={a.port} baud={a.baud}\n')
 valid=rej=0; latest=None; last=None
 ready=False; pe9open=False; auth=False; mainvalid=False; maxmoves=0; maxho=0; rcs_nonzero=False; max_target_dev=0; closed_only=True; max_closed_error=0
 try:
  for l in lines:
   if not l.strip().startswith(PREFIX): continue
   try:f,raw=dec(l)
   except Exception as e: rej+=1; print('REJECT',e,file=sys.stderr); continue
   valid+=1; latest=f
   ready|=bool(iv(f,'preflight_ready')); pe9open|=bool(iv(f,'pe9_debounced_open')); auth|=bool(iv(f,'actuator_authorized')); mainvalid|=bool(iv(f,'main_output_valid'))
   maxmoves=max(maxmoves,iv(f,'p111_moves_completed')); maxho=max(maxho,iv(f,'p112_hard_off_count'))
   rcs_nonzero |= bool(iv(f,'rcs_applied_mask'))
   base=iv(f,'p111_baseline_adc'); tgt=iv(f,'p111_target_adc')
   if base:
    dev=abs(tgt-base); max_target_dev=max(max_target_dev,dev)
    if dev>8: closed_only=False
    if dev<=8: max_closed_error=max(max_closed_error,abs(iv(f,'needle_adc')-tgt))
   ph=phase(f)
   if ph!=last: print('\n>>>',ph); last=ph
   print(status(f),flush=True)
   if log:
    d=dict(f); d['phase']=ph; d['raw_frame']=raw; log.write(json.dumps(d,sort_keys=True)+'\n')
 except KeyboardInterrupt: print('\nDurduruldu.')
 finally:
  if log: log.close()
 print('\n=============== P112R12R3 OZET ===============')
 print('Valid/rejected     :',valid,'/',rej)
 print('Preflight ready    :',ready)
 print('PE9 open seen      :',pe9open)
 print('Authorization seen :',auth)
 print('GNC valid seen     :',mainvalid)
 print('Max moves          :',maxmoves)
 print('Max target dev ADC :',max_target_dev)
 print('Closed-only target  :',closed_only)
 print('Max closed err ADC  :',max_closed_error)
 print('RCS applied != 0   :',rcs_nonzero)
 print('Hard-off max       :',maxho)
 if not latest:return 2
 base=iv(latest,'p111_baseline_adc'); final=iv(latest,'needle_adc')
 print('Final baseline/ADC :',base,'/',final)
 print('Final P111/P110    :',S111.get(iv(latest,'p111_state'),iv(latest,'p111_state')),'/',R110.get(iv(latest,'p110_result'),iv(latest,'p110_result')))
 print('Fault needle/P111  :',iv(latest,'needle_fault'),iv(latest,'p111_fault'))
 fail=(not ready or base<950 or rcs_nonzero or maxho or iv(latest,'needle_fault') or iv(latest,'p111_fault') or iv(latest,'p110_active_pwm'))
 if pe9open and not auth: fail=True
 if maxmoves and iv(latest,'p110_result') not in (0,1): fail=True
 if pe9open and auth and closed_only and maxmoves>0: fail=True
 if fail:
  print('\nSONUC: TAM PASS DEGIL - LOGU GONDER')
  return 1
 if not pe9open:
  print('\nSONUC: PREFLIGHT PASS; PE9/AUTHORIZATION HENUZ TEST EDILMEDI')
 elif maxmoves==0:
  print('\nSONUC: PASS - CLOSED TARGET SABIT; SAME-TARGET GUARD GEREKSIZ MOTOR HAREKETI BASLATMADI')
 else:
  print('\nSONUC: PASS - GERCEK GNC TARGET DEGISIMIYLE PRODUCTION MOTOR HAREKETI GOZLENDI')
 return 0

def main():
 p=argparse.ArgumentParser();p.add_argument('capture',nargs='?');p.add_argument('--port',default='COM21');p.add_argument('--baud',type=int,default=BAUD);p.add_argument('--duration',type=float,default=40);p.add_argument('--log',default='uart_p112r12r3_prod_auto_inert.txt');return run(p.parse_args())
if __name__=='__main__': raise SystemExit(main())
