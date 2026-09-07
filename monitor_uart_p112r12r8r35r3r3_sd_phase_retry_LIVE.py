#!/usr/bin/env python3
from __future__ import annotations
import argparse,time,serial
import monitor_uart_p112r12r8r35r3_sd_service_closure as m

def iv(d,k):
    try:return int(d.get(k,0))
    except:return 0

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--port',default='COM21'); ap.add_argument('--baud',type=int,default=115200); ap.add_argument('--duration',type=float,default=35)
    a=ap.parse_args(); t0=time.monotonic(); valid=bad=0
    with serial.Serial(a.port,a.baud,timeout=.5) as s:
        try:s.reset_input_buffer()
        except Exception:pass
        print('R8R35R3R3 SD PHASE RETRY - LIVE')
        while time.monotonic()-t0<a.duration:
            b=s.readline()
            if not b: continue
            line=b.decode('ascii',errors='replace').strip()
            try:
                d,_=m.dec(line); valid+=1
                print(f"t={iv(d,'time_ms'):>6} ready={iv(d,'ready')} flight={iv(d,'flight_active')} SD={iv(d,'sd_ready')}/{iv(d,'sd_logging')} ring={iv(d,'sd_ring_count'):>3} drop={iv(d,'sd_dropped'):>5} push/pop={iv(d,'sd_ring_push_total')}/{iv(d,'sd_ring_pop_total')} upd={iv(d,'sd_update_count_total')} defer={iv(d,'sd_defer_total')} wrerr={iv(d,'sd_write_errors')}")
            except Exception as e:
                bad+=1; print('REJECT:',e)
    print(f'valid={valid} rejected={bad}')
if __name__=='__main__': main()
