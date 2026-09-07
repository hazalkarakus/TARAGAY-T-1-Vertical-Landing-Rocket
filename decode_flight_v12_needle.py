#!/usr/bin/env python3
from pathlib import Path
import struct, sys, csv

MAGIC = 0x54475931
FRAME_SIZE = 272
CRC_OFFSET = 270

def crc16_ccitt(data: bytes, init=0xFFFF):
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)) & 0xFFFF
    return crc

def u16(b,o): return struct.unpack_from("<H", b, o)[0]
def i16(b,o): return struct.unpack_from("<h", b, o)[0]
def u32(b,o): return struct.unpack_from("<I", b, o)[0]

def main():
    if len(sys.argv) != 2:
        print("Usage: python decode_flight_v12_needle.py flight.bin")
        raise SystemExit(2)

    p = Path(sys.argv[1])
    raw = p.read_bytes()
    n = len(raw) // FRAME_SIZE
    trailing = len(raw) % FRAME_SIZE
    rows = []
    crc_ok = 0
    gaps = 0
    prev = None

    for i in range(n):
        f = raw[i*FRAME_SIZE:(i+1)*FRAME_SIZE]
        seq = u32(f, 4)
        if prev is not None and seq != ((prev + 1) & 0xFFFFFFFF):
            gaps += 1
        prev = seq

        stored = u16(f, CRC_OFFSET)
        calc = crc16_ccitt(f[:CRC_OFFSET])
        ok = stored == calc
        crc_ok += int(ok)

        flags = f[268]
        row = {
            "index": i,
            "magic_ok": int(u32(f,0) == MAGIC),
            "sequence": seq,
            "time_s": u32(f,8)/1e6,
            "format_version": u16(f,36),
            "frame_size": u16(f,38),
            "crc_ok": int(ok),
            "valve_cmd": u16(f,254)/10000.0,
            "valve_cmd_limited": u16(f,256)/10000.0,
            "pot_raw": u16(f,258),
            "zero_adc": u16(f,260),
            "target_adc": u16(f,262),
            "error_adc": i16(f,264),
            "rpwm": f[266],
            "lpwm": f[267],
            "enabled": int(bool(flags & 1)),
            "zero_valid": int(bool(flags & 2)),
            "lock": int(bool(flags & 4)),
            "needle_fault": f[269],
        }
        rows.append(row)

    print(f"File bytes      : {len(raw)}")
    print(f"Frames          : {n}")
    print(f"Trailing bytes  : {trailing}")
    print(f"CRC OK          : {crc_ok}/{n}")
    print(f"Sequence gaps   : {gaps}")
    if rows:
        print(f"Format          : V{rows[0]['format_version']}")
        print(f"Frame size      : {rows[0]['frame_size']}")
        print(f"Peak Valve_Cmd  : {max(r['valve_cmd'] for r in rows):.4f}")
        print(f"Needle faults   : {sum(r['needle_fault'] != 0 for r in rows)} frames")

    out = p.with_name(p.stem + "_V12_NEEDLE.csv")
    if rows:
        with out.open("w", newline="", encoding="utf-8") as fp:
            w = csv.DictWriter(fp, fieldnames=rows[0].keys())
            w.writeheader()
            w.writerows(rows)
    print(f"CSV             : {out}")

if __name__ == "__main__":
    main()
