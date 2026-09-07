#!/usr/bin/env python3
"""Decode TARAGAY-T1 R8R35R3R10 flight.bin.

V14 frames are normal preflight records. V15 + marker 0xF10A are the compact
RAM-flight records replayed to the existing 384-byte file after touchdown or
latched E-stop.
"""
from __future__ import annotations
import argparse,binascii,csv,struct
from pathlib import Path

FRAME=384
MAGIC=0x54475931
RAM_VERSION=15
RAM_MARKER=0xF10A

def u8(b,o): return b[o]
def u16(b,o): return struct.unpack_from('<H',b,o)[0]
def i16(b,o): return struct.unpack_from('<h',b,o)[0]
def u32(b,o): return struct.unpack_from('<I',b,o)[0]
def i32(b,o): return struct.unpack_from('<i',b,o)[0]

def crc_ok(b:bytes)->bool:
    return binascii.crc_hqx(b[:382],0xFFFF)==u16(b,382)

def ram_row(b:bytes):
    flags_low=u16(b,378); flags_high=u16(b,380)
    return {
        'sequence':u32(b,4),'time_us':u32(b,8),'time_ms':u32(b,12),
        'pressure_pa':i32(b,16),'altitude_m':i32(b,20)/100.0,
        'lidar_m':u16(b,92)/1000.0,
        'eskf_x_m':i16(b,112)/100.0,'eskf_y_m':i16(b,114)/100.0,
        'eskf_z_m':i32(b,96)/1000.0,
        'eskf_vx_mps':i16(b,116)/100.0,'eskf_vy_mps':i16(b,118)/100.0,
        'eskf_vz_mps':i32(b,100)/1000.0,
        'roll_deg':i16(b,134)/100.0,'pitch_deg':i16(b,136)/100.0,'yaw_deg':i16(b,138)/100.0,
        'pitch_rate_dps':i16(b,76)/100.0,'yaw_rate_dps':i16(b,78)/100.0,
        'q_w':i16(b,126)/32767.0,'q_x':i16(b,128)/32767.0,
        'q_y':i16(b,130)/32767.0,'q_z':i16(b,132)/32767.0,
        'cpu_pct':u16(b,56)/100.0,'health_flags':u8(b,58),'fault_code':u8(b,59),
        'valve_cmd':u16(b,252)/10000.0,
        'needle_cmd':u16(b,254)/10000.0,'needle_limited_cmd':u16(b,256)/10000.0,
        'needle_adc':u16(b,258),'needle_target_adc':u16(b,262),'needle_error_adc':i16(b,264),
        'needle_rpwm':u8(b,266),'needle_lpwm':u8(b,267),'needle_state_flags':u8(b,268),'needle_fault':u8(b,269),
        'imu_age_us':u16(b,370),'baro_age_us':u16(b,372),'lidar_age_us':u16(b,374),
        'mission_state':u8(b,376),'rcs_requested_mask':u8(b,377),'rcs_applied_mask':flags_low & 0xFF,
        'flight_input_valid':(flags_low>>8)&0xFF,'flight_rcs_fault':flags_high&0xFF,'mount_cal_valid':(flags_high>>8)&0xFF,
        'full_eskf_flags':u8(b,110),'full_eskf_attitude_flags':u8(b,146),
    }

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('bin',nargs='?',default='flight.bin')
    ap.add_argument('--out',default='flight_r3r10_ram_replay.csv')
    a=ap.parse_args()
    data=Path(a.bin).read_bytes()
    normal=ram=bad_crc=bad_magic=0; rows=[]
    for off in range(0,len(data)-FRAME+1,FRAME):
        b=data[off:off+FRAME]
        if u32(b,0)!=MAGIC:
            bad_magic+=1
            # Preallocated/zero tail starts here; later sectors are not useful.
            if bad_magic>=4: break
            continue
        if not crc_ok(b):
            bad_crc+=1; continue
        ver=u16(b,36); marker=u16(b,286)
        if ver==RAM_VERSION and marker==RAM_MARKER:
            ram+=1; rows.append(ram_row(b))
        else:
            normal+=1
    if rows:
        with Path(a.out).open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]))
            w.writeheader(); w.writerows(rows)
    print(f'valid V14/other frames : {normal}')
    print(f'R3R10 RAM replay frames: {ram}')
    print(f'bad CRC               : {bad_crc}')
    print(f'bad/zero magic         : {bad_magic}')
    if rows:
        print(f'RAM replay duration    : {(rows[-1]["time_ms"]-rows[0]["time_ms"])/1000.0:.3f} s')
        print(f'CSV                    : {Path(a.out).resolve()}')
    else:
        print('No R3R10 replay frames found. If this was a bench run, trigger safe E-stop and wait for flush_complete=1 before power-off.')
    return 0

if __name__=='__main__': raise SystemExit(main())
