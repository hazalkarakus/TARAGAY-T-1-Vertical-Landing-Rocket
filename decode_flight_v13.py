#!/usr/bin/env python3
from pathlib import Path
import struct
import sys
import csv

MAGIC = 0x54475931
FRAME_SIZE = 288
CRC_OFFSET = 286

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
        print("Usage: python decode_flight_v13.py flight.bin")
        raise SystemExit(2)

    p = Path(sys.argv[1])
    raw = p.read_bytes()
    count = len(raw) // FRAME_SIZE
    trailing = len(raw) % FRAME_SIZE

    rows = []
    crc_ok_count = 0
    sequence_gaps = 0
    previous = None

    for i in range(count):
        f = raw[i*FRAME_SIZE:(i+1)*FRAME_SIZE]

        seq = u32(f,4)
        if previous is not None and seq != ((previous + 1) & 0xFFFFFFFF):
            sequence_gaps += 1
        previous = seq

        stored_crc = u16(f, CRC_OFFSET)
        calc_crc = crc16_ccitt(f[:CRC_OFFSET])
        crc_ok = stored_crc == calc_crc
        crc_ok_count += int(crc_ok)

        needle_flags = f[268]

        rows.append({
            "index": i,
            "magic_ok": int(u32(f,0) == MAGIC),
            "sequence": seq,
            "time_s": u32(f,8) / 1e6,
            "format_version": u16(f,36),
            "frame_size": u16(f,38),
            "crc_ok": int(crc_ok),

            "valve_cmd": u16(f,254) / 10000.0,
            "valve_cmd_limited": u16(f,256) / 10000.0,
            "pot_raw": u16(f,258),
            "zero_adc": u16(f,260),
            "target_adc": u16(f,262),
            "error_adc": i16(f,264),
            "rpwm": f[266],
            "lpwm": f[267],
            "needle_enabled": int(bool(needle_flags & 1)),
            "needle_zero_valid": int(bool(needle_flags & 2)),
            "needle_lock": int(bool(needle_flags & 4)),
            "needle_fault": f[269],

            "nrf_link_active": f[270],
            "nrf_command": f[271],
            "nrf_valid_packet_count": u32(f,272),
            "nrf_last_sequence": f[276],
            "servo_target_open": f[277],
            "servo_pulse_us": u16(f,278),
            "nrf_last_packet_age_ms": u16(f,280),
            "nrf_irq_count_low": u16(f,282),
            "nrf_rx_count_low": u16(f,284),
        })

    print(f"File bytes        : {len(raw)}")
    print(f"Frames            : {count}")
    print(f"Trailing bytes    : {trailing}")
    print(f"CRC OK            : {crc_ok_count}/{count}")
    print(f"Sequence gaps     : {sequence_gaps}")

    if rows:
        print(f"Format            : V{rows[0]['format_version']}")
        print(f"Frame size        : {rows[0]['frame_size']}")
        print(f"Peak Valve_Cmd    : {max(r['valve_cmd'] for r in rows):.4f}")
        print(f"Needle fault rows : {sum(r['needle_fault'] != 0 for r in rows)}")
        print(f"NRF linked rows   : {sum(r['nrf_link_active'] != 0 for r in rows)}")
        print(f"Servo OPEN rows   : {sum(r['servo_target_open'] != 0 for r in rows)}")

        intervals = [
            rows[i]["time_s"] - rows[i-1]["time_s"]
            for i in range(1, len(rows))
        ]
        if intervals:
            avg = sum(intervals) / len(intervals)
            print(f"Average SD rate   : {1.0/avg:.3f} Hz")

    out = p.with_name(p.stem + "_V13.csv")
    if rows:
        with out.open("w", newline="", encoding="utf-8") as fp:
            w = csv.DictWriter(fp, fieldnames=rows[0].keys())
            w.writeheader()
            w.writerows(rows)

    print(f"CSV               : {out}")

if __name__ == "__main__":
    main()
