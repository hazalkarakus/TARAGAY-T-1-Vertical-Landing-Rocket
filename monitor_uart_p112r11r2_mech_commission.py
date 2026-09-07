#!/usr/bin/env python3
"""P112R11R2 mechanical commissioning UART monitor - READ ONLY."""
from __future__ import annotations
import argparse,binascii,json,sys,time
from pathlib import Path

PREFIX="$TGY68,"; N=498; BAUD=115200
I={
"seq":1,"time_ms":2,"preflight_state":3,"flight_active":4,"pe9_raw_open":5,"pe9_debounced_open":6,"preflight_ready":7,"preflight_fault":8,"connector_seen":10,"actuator_authorized":11,
"imu_valid":12,"baro_valid":89,"lidar_valid":90,"eskf_init":91,"eskf_ok":92,
"needle_adc":198,"needle_target_adc":199,"needle_enabled":200,"needle_fault":202,"needle_zero_adc":203,"needle_error_adc":204,"needle_zero_valid":207,
"system_fault":322,"p83_filtered_adc":343,"p83_feedback_valid":344,"p83_confidence_pct":345,"p83_mode":350,
"p110_state":435,"p110_result":436,"p110_direction":437,"p110_start_adc":438,"p110_target_adc":439,"p110_current_adc":440,"p110_error_adc":441,"p110_active_pwm":442,"p110_learned_breakaway_pwm":443,"p110_sustain_pwm":444,"p110_speed_adc_s":445,"p110_stop_distance_adc":446,"p110_brake_entry_adc":447,"p110_coast_max_adc":448,"p110_final_adc":449,"p110_final_error_adc":450,"p110_total_powered_ms":454,"p110_correction_count":455,"p110_breakaway_found":456,"p110_abort_reason":457,
"p111_state":458,"p111_result":459,"p111_command_sequence":460,"p111_moves_completed":464,"p111_baseline_adc":465,"p111_target_adc":466,"p111_learned_open_pwm":467,"p111_learned_close_pwm":468,"p111_learned_open_coast":469,"p111_learned_close_coast":470,"p111_last_open_error_adc":471,"p111_last_close_error_adc":472,"p111_fault":481,
"p112_control_ticks":482,"p112_adc_samples":483,"p112_hard_off_count":484,"p112_uart_suppressed_count":485,"p112_sd_suppressed_count":486,"p112_isr_last_us":487,"p112_isr_max_us":488,"p112_timing_critical":489,"p112_decel_active":490,"p112_decel_pwm":491,
"r11_state":493,"r11_abort_reason":494,"r11_abort_count":495,"r11_button":496,"r11_open_target_adc":497}
S111={0:"WAIT_REFERENCE",1:"READY",2:"TRACKING",3:"HOLD",4:"REVOKED",5:"FAULT"}
S110={0:"WAIT",1:"SEARCH",2:"DRIVE",3:"BRAKE",4:"CORRECTION_DWELL",5:"DONE"}
R110={0:"RUNNING",1:"PASS",2:"ABORT",3:"OVERSHOOT",4:"NO_BREAKAWAY",5:"POWER_TIMEOUT"}
SR11={0:"WAIT_SAFE",1:"ARMED",2:"OPENING",3:"HOLD_OPEN",4:"CLOSING",5:"DONE",6:"ABORT"}
AR11={0:"NONE",1:"PE9_OR_CONNECTOR",2:"FLIGHT_ACTIVE",3:"AUTONOMOUS_FAULT",4:"P83_FEEDBACK",5:"NEEDLE_FAULT",6:"TIMEOUT",7:"OPERATOR_PA0",8:"OPEN_SUBMIT",9:"HOLD_SUBMIT",10:"CLOSE_SUBMIT"}

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
 return SR11.get(iv(f,'r11_state'),f"STATE_{iv(f,'r11_state')}")

def status(f):
 return (f"t={iv(f,'time_ms')/1000:7.2f}s R11={phase(f):>10} AB={AR11.get(iv(f,'r11_abort_reason'),iv(f,'r11_abort_reason')):<18} | "
 f"PE9={'OPEN' if iv(f,'pe9_debounced_open') else 'CONN'} READY={iv(f,'preflight_ready')} SYSF={iv(f,'system_fault')} | "
 f"ADC={iv(f,'needle_adc'):4d} TGT={iv(f,'needle_target_adc'):4d} ZERO={iv(f,'needle_zero_adc'):4d} EN={iv(f,'needle_enabled')} NF={iv(f,'needle_fault')} | "
 f"P83={iv(f,'p83_feedback_valid')}/M{iv(f,'p83_mode')} C={iv(f,'p83_confidence_pct'):2d}% | "
 f"P111={S111.get(iv(f,'p111_state'),iv(f,'p111_state'))} TGT={iv(f,'p111_target_adc'):4d} MOV={iv(f,'p111_moves_completed')} F={iv(f,'p111_fault')} | "
 f"P110={S110.get(iv(f,'p110_state'),iv(f,'p110_state'))} {R110.get(iv(f,'p110_result'),iv(f,'p110_result'))} DIR={iv(f,'p110_direction')} PWM={iv(f,'p110_active_pwm'):3d} BRK={iv(f,'p110_breakaway_found')} SPD={iv(f,'p110_speed_adc_s'):4d} STOP={iv(f,'p110_stop_distance_adc'):3d} COAST={iv(f,'p110_coast_max_adc'):3d} CORR={iv(f,'p110_correction_count')} | "
 f"HO={iv(f,'p112_hard_off_count')} ISR={iv(f,'p112_isr_last_us'):3d}/{iv(f,'p112_isr_max_us'):3d}us")

def serial_lines(port,baud,dur):
 try:
  import serial
 except ImportError: print('Kur: py -m pip install pyserial'); raise
 t=time.monotonic()
 with serial.Serial(port,baudrate=baud,timeout=.5) as s:
  try:s.reset_input_buffer()
  except:pass
  print('='*78); print('P112R11R2 MECHANICAL COMMISSIONING - READ ONLY UART')
  print('PE9 CONNECTED kalacak. BASINCSIZ/INERT. PA0 bir kez bas-birak.')
  print('='*78)
  while dur<=0 or time.monotonic()-t<dur:
   b=s.readline()
   if b:yield b.decode('ascii',errors='replace')
def file_lines(path):
 for l in Path(path).open(errors='replace'):
  q=l.strip()
  if q.startswith('{'):
   try:
    d=json.loads(q)
    if isinstance(d.get('raw_frame'),str):yield d['raw_frame'];continue
   except:pass
  yield l

def run(a):
 if a.capture: lines=file_lines(a.capture); log=None
 else:
  lines=serial_lines(a.port,a.baud,a.duration); log=Path(a.log).open('w',buffering=1)
  log.write(f'# P112R11R2 mechanical commissioning | port={a.port} baud={a.baud}\n')
 valid=rej=0; latest=None; last=None; ready=False; pe9open=False; maxmoves=0; maxho=0; minadc=65535; state_seen=set()
 try:
  for l in lines:
   if not l.strip().startswith(PREFIX):continue
   try:f,raw=dec(l)
   except Exception as e: rej+=1; print('REJECT',e,file=sys.stderr); continue
   valid+=1;latest=f;ready|=bool(iv(f,'preflight_ready'));pe9open|=bool(iv(f,'pe9_debounced_open'))
   maxmoves=max(maxmoves,iv(f,'p111_moves_completed')); maxho=max(maxho,iv(f,'p112_hard_off_count')); minadc=min(minadc,iv(f,'needle_adc')); state_seen.add(iv(f,'r11_state'))
   ph=phase(f)
   if ph!=last:
    print('\n>>> R11R2 STATE:',ph)
    if iv(f,'r11_abort_reason'): print('>>> ABORT REASON:',AR11.get(iv(f,'r11_abort_reason'),iv(f,'r11_abort_reason')))
    last=ph
   print(status(f),flush=True)
   if log:
    d=dict(f);d['phase']=ph;d['abort_name']=AR11.get(iv(f,'r11_abort_reason'),iv(f,'r11_abort_reason'));d['raw_frame']=raw;log.write(json.dumps(d,sort_keys=True)+'\n')
 except KeyboardInterrupt:print('\nDurduruldu.')
 finally:
  if log:log.close()
 print('\n================ P112R11R2 OZET ================')
 print('Valid frame        :',valid);print('Rejected frame     :',rej);print('Preflight ready    :',ready);print('PE9 ever OPEN      :',pe9open);print('States seen        :',','.join(SR11.get(x,str(x)) for x in sorted(state_seen)));print('Max moves          :',maxmoves);print('Min needle ADC     :',minadc if minadc!=65535 else '-');print('Hard-off max       :',maxho)
 if not latest:return 2
 base=iv(latest,'p111_baseline_adc');final=iv(latest,'needle_adc'); st=iv(latest,'r11_state'); ab=iv(latest,'r11_abort_reason')
 print(f"Final baseline/ADC : {base}/{final} (floor=950)");print('Open/close errors  :',iv(latest,'p111_last_open_error_adc'),iv(latest,'p111_last_close_error_adc'));print('Learned PWM O/C    :',iv(latest,'p111_learned_open_pwm'),iv(latest,'p111_learned_close_pwm'));print('Learned coast O/C  :',iv(latest,'p111_learned_open_coast'),iv(latest,'p111_learned_close_coast'));print('Fault needle/P111  :',iv(latest,'needle_fault'),iv(latest,'p111_fault'));print('R11 abort          :',AR11.get(ab,ab),'count=',iv(latest,'r11_abort_count'))
 fail=(not ready or pe9open or base<950 or st!=5 or ab!=0 or maxmoves!=2 or not base or abs(final-base)>8 or iv(latest,'needle_fault') or iv(latest,'p111_fault') or maxho or iv(latest,'p110_active_pwm'))
 print('\nSONUC:', 'PASS - P112R11R2 SHORT MECHANICAL COMMISSIONING' if not fail else 'TAM PASS DEGIL - LOGU GONDER')
 return 1 if fail else 0

def main():
 p=argparse.ArgumentParser();p.add_argument('capture',nargs='?');p.add_argument('--port',default='COM21');p.add_argument('--baud',type=int,default=BAUD);p.add_argument('--duration',type=float,default=35);p.add_argument('--log',default='uart_p112r11r2_mech_commission.txt');return run(p.parse_args())
if __name__=='__main__':raise SystemExit(main())
