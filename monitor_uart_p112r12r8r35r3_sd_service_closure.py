#!/usr/bin/env python3
"""TARAGAY-T1 R8R35R3 SD-only 200 Hz logger closure monitor."""
from __future__ import annotations
import argparse,binascii,json,statistics,time
from pathlib import Path
import monitor_uart_p112r12r8r35r2_closure_fixes as r2
PREFIX="$TGY73,"
SD3_EXTRA=(
 "sd_frames_total","sd_ring_push_total","sd_ring_pop_total",
 "sd_dma_start_total","sd_dma_complete_total","sd_update_count_total",
 "sd_defer_total","sd_suppressed_total","sd_drain_last_us_r3",
 "sd_drain_max_us_r3","sd_drain_yields_total","sd_write_max_us_r3",
)
FIELDS=r2.FIELDS+SD3_EXTRA
N=len(FIELDS)
assert N==299,(N,299)
def n(v):
    try:return int(v,0)
    except Exception:
        try:return float(v)
        except Exception:return v
def iv(f,k):
    try:return int(f.get(k,0))
    except Exception:return 0
def dec(line):
    s=line.strip()
    if not s.startswith(PREFIX) or "*" not in s: raise ValueError("not TGY73")
    body,crc=s.rsplit("*",1); rx=int(crc,16); calc=binascii.crc_hqx(body.encode("ascii"),0xFFFF)
    if rx!=calc: raise ValueError(f"CRC {rx:04X}!={calc:04X}")
    a=body.split(",")
    if len(a)!=N: raise ValueError(f"fields {len(a)} != {N}")
    return {k:n(v) for k,v in zip(FIELDS,a)},s
def file_lines(path):
    for line in Path(path).open(errors="replace"):
        q=line.strip()
        if q.startswith("{"):
            try:
                d=json.loads(q)
                if isinstance(d.get("raw_frame"),str): yield d["raw_frame"]; continue
            except Exception: pass
        yield line
def serial_lines(port,baud,duration):
    import serial
    t0=time.monotonic()
    with serial.Serial(port,baudrate=baud,timeout=.5) as s:
        try:s.reset_input_buffer()
        except Exception:pass
        print("R8R35R3 SD SERVICE CLOSURE - BASINCSIZ")
        print("READY=1 bekle, PE9 ayir; normal full-system dry-run. Bu monitor SADECE SD kapanisini puanlar.")
        while duration<=0 or time.monotonic()-t0<duration:
            b=s.readline()
            if b: yield b.decode("ascii",errors="replace")
def run(a):
    lines=file_lines(a.capture) if a.capture else serial_lines(a.port,a.baud,a.duration)
    rows=[]; rejected=0
    for line in lines:
        try:f,raw=dec(line); rows.append(f)
        except Exception: rejected+=1; continue
    if not rows:
        print("TGY73 frame yok"); return 2
    first,last=rows[0],rows[-1]
    dur=max(.001,(iv(last,'time_ms')-iv(first,'time_ms'))/1000.0)
    def d(k): return max(0,iv(last,k)-iv(first,k))
    cpu=[float(x.get('cpu_pct',0.0)) for x in rows]
    flight=any(iv(x,'flight_active') for x in rows)
    ready=any(iv(x,'ready') for x in rows)
    sd_alive_all_after_ready=True; ready_seen=False
    max_ring=0; max_bp=0
    first_bad=None
    for x in rows:
        if iv(x,'ready'): ready_seen=True
        if ready_seen and (iv(x,'sd_ready')==0 or iv(x,'sd_logging')==0):
            sd_alive_all_after_ready=False
            if first_bad is None: first_bad=x
        max_ring=max(max_ring,iv(x,'sd_ring_count'))
        max_bp=max(max_bp,iv(x,'sd_backpressure_level'))
    frame_rate=d('sd_frames_total')/dur
    push_rate=d('sd_ring_push_total')/dur
    pop_rate=d('sd_ring_pop_total')/dur
    update_rate=d('sd_update_count_total')/dur
    dma_start=d('sd_dma_start_total'); dma_done=d('sd_dma_complete_total')
    misses=sum(d(k) for k in ('miss_imu','miss_baro','miss_lidar','miss_nrf','miss_eskf'))
    realign=d('scheduler_realigns')
    sd_ok=(ready and flight and sd_alive_all_after_ready and iv(last,'sd_ready')==1 and iv(last,'sd_logging')==1 and
           d('sd_errors')==0 and d('sd_write_errors')==0 and d('sd_dropped')==0 and d('sd_ring_overruns')==0 and
           d('sd_async_timeouts')==0 and d('sd_runtime_rec_failures')==0 and d('sd_runtime_rec_flight_aborts')==0 and
           d('sd_suppressed_total')==0 and max_ring < 127 and frame_rate>=190.0 and push_rate>=190.0 and pop_rate>=190.0 and
           update_rate>=200.0 and dma_done>0 and dma_done<=dma_start and misses==0 and realign==0)
    print("\n================ R8R35R3 SD CLOSURE OZET ================")
    print(f"Valid/rejected                 : {len(rows)}/{rejected}")
    print(f"Duration                       : {dur:.3f}s")
    print(f"CPU avg/max                    : {statistics.mean(cpu):.2f}% / {max(cpu):.2f}%")
    print(f"Ready/flight                   : {ready}/{flight}")
    print(f"SD final ready/log             : {iv(last,'sd_ready')}/{iv(last,'sd_logging')}")
    print(f"SD err/write/drop/overrun d    : {d('sd_errors')}/{d('sd_write_errors')}/{d('sd_dropped')}/{d('sd_ring_overruns')}")
    print(f"SD async/recovery flight d     : {d('sd_async_timeouts')}/{d('sd_runtime_rec_flight_aborts')}")
    print(f"SD frame push/pop rate         : {frame_rate:.1f}/{push_rate:.1f}/{pop_rate:.1f} Hz")
    print(f"SD update rate                 : {update_rate:.1f} Hz")
    print(f"SD DMA start/complete delta    : {dma_start}/{dma_done}")
    print(f"SD ring max / backpressure max : {max_ring}/127 / {max_bp}")
    print(f"SD defer/suppress delta        : {d('sd_defer_total')}/{d('sd_suppressed_total')}")
    print(f"SD drain last/max us           : {iv(last,'sd_drain_last_us_r3')}/{iv(last,'sd_drain_max_us_r3')}")
    print(f"SD write max us (async age)    : {iv(last,'sd_write_max_us_r3')}")
    print(f"Scheduler misses/realign       : {misses}/{realign}")
    if first_bad is not None:
        print(f"FIRST SD BAD                   : t={iv(first_bad,'time_ms')}ms ring={iv(first_bad,'sd_ring_count')} drop={iv(first_bad,'sd_dropped')} err={iv(first_bad,'sd_errors')} last={iv(first_bad,'sd_last_result')}")
    print("\nSONUC:","PASS - SD 200 Hz sustained logging kapandi" if sd_ok else "INCELE - SD closure kosullari tamamlanmadi")
    return 0 if sd_ok else 1
if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--port',default='COM21'); ap.add_argument('--baud',type=int,default=115200); ap.add_argument('--duration',type=float,default=35); ap.add_argument('--capture');
    raise SystemExit(run(ap.parse_args()))
