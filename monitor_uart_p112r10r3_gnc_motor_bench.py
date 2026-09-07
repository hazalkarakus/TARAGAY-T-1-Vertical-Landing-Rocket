#!/usr/bin/env python3
"""
TARAGAY-T1 P112R10R3 GNC MOTOR BENCH / SETTLED TARGET HOLD UART MONITOR

READ ONLY:
- STM32'ye hicbir komut gondermez.
- PA0 kullanilmaz.
- $TGY68 / 493 field / CRC16-CCITT.

P112R10R3 beklenen tek cevrim:
  CLOSED -> PE9 separation -> authorization
  -> 1 s CLOSED
  -> GNC target ~789 (1023 reference icin)
  -> gercek P110/P111/P112 OPEN hareketi
  -> ~3 s sonra target CLOSED
  -> gercek CLOSE hareketi
  -> CLOSED latch (reset gelene kadar tekrar stimulus yok)

Kurulum:
    py -m pip install pyserial

Calistir:
    py monitor_uart_p112r10r3_gnc_motor_bench.py --port COM21 --duration 40

Suresiz:
    py monitor_uart_p112r10r3_gnc_motor_bench.py --port COM21 --duration 0

Offline:
    py monitor_uart_p112r10r3_gnc_motor_bench.py uart_p112r10r3_gnc_motor_bench.txt
"""

from __future__ import annotations

import argparse
import binascii
import json
import sys
import time
from pathlib import Path
from typing import Iterable

FRAME_PREFIX = "$TGY68,"
EXPECTED_FIELDS = 493
DEFAULT_BAUD = 115200

IDX = {
    "seq": 1, "time_ms": 2,
    "preflight_state": 3, "flight_active": 4,
    "pe9_raw_open": 5, "pe9_debounced_open": 6,
    "preflight_ready": 7, "preflight_fault": 8,
    "connector_seen": 10, "actuator_authorized": 11,
    "imu_valid": 12,
    "baro_valid": 89, "lidar_valid": 90,
    "eskf_init": 91, "eskf_ok": 92,

    "needle_adc": 198, "needle_target_adc": 199,
    "needle_enabled": 200, "needle_fault": 202,
    "needle_zero_adc": 203, "needle_error_adc": 204,
    "needle_zero_valid": 207,

    "system_fault": 322,

    "p83_filtered_adc": 343, "p83_feedback_valid": 344,
    "p83_confidence_pct": 345, "p83_mode": 350,

    "p110_state": 435, "p110_result": 436,
    "p110_direction": 437, "p110_start_adc": 438,
    "p110_target_adc": 439, "p110_current_adc": 440,
    "p110_error_adc": 441, "p110_active_pwm": 442,
    "p110_learned_breakaway_pwm": 443, "p110_sustain_pwm": 444,
    "p110_speed_adc_s": 445, "p110_stop_distance_adc": 446,
    "p110_brake_entry_adc": 447, "p110_coast_max_adc": 448,
    "p110_final_adc": 449, "p110_final_error_adc": 450,
    "p110_total_powered_ms": 454, "p110_correction_count": 455,
    "p110_breakaway_found": 456, "p110_abort_reason": 457,

    "p111_state": 458, "p111_result": 459,
    "p111_command_sequence": 460, "p111_moves_completed": 464,
    "p111_baseline_adc": 465, "p111_target_adc": 466,
    "p111_learned_open_pwm": 467, "p111_learned_close_pwm": 468,
    "p111_learned_open_coast": 469, "p111_learned_close_coast": 470,
    "p111_last_open_error_adc": 471, "p111_last_close_error_adc": 472,
    "p111_fault": 481,

    "p112_control_ticks": 482, "p112_adc_samples": 483,
    "p112_hard_off_count": 484,
    "p112_uart_suppressed_count": 485,
    "p112_sd_suppressed_count": 486,
    "p112_isr_last_us": 487, "p112_isr_max_us": 488,
    "p112_timing_critical": 489,
    "p112_decel_active": 490, "p112_decel_pwm": 491,
    "p112_initial_coast_adc": 492,
}

P111_STATE = {
    0:"WAIT_REFERENCE", 1:"READY", 2:"TRACKING",
    3:"HOLD", 4:"REVOKED", 5:"FAULT"
}
P111_FAULT = {
    0:"NONE", 1:"REFERENCE", 2:"FEEDBACK",
    3:"COMMAND_TIMEOUT", 4:"LOW_LEVEL", 5:"TARGET_RANGE"
}
P110_STATE = {
    0:"WAIT", 1:"SEARCH", 2:"DRIVE",
    3:"BRAKE", 4:"CORRECTION_DWELL", 5:"DONE"
}
P110_RESULT = {
    0:"RUNNING", 1:"PASS", 2:"ABORT",
    3:"OVERSHOOT", 4:"NO_BREAKAWAY", 5:"POWER_TIMEOUT"
}

def number(v: str):
    try:
        return int(v, 0)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return v

def iv(f: dict, key: str) -> int:
    try:
        return int(f.get(key, 0))
    except Exception:
        return 0

def decode(line: str):
    s = line.strip()
    if not s.startswith(FRAME_PREFIX):
        raise ValueError("TGY68 frame degil")
    if "*" not in s:
        raise ValueError("CRC ayirici yok")

    body, crc_txt = s.rsplit("*", 1)
    rx = int(crc_txt, 16)
    calc = binascii.crc_hqx(body.encode("ascii"), 0xFFFF)
    if rx != calc:
        raise ValueError(f"CRC mismatch rx={rx:04X} calc={calc:04X}")

    vals = body.split(",")
    if len(vals) != EXPECTED_FIELDS:
        raise ValueError(f"field={len(vals)}, beklenen={EXPECTED_FIELDS}")

    return {k:number(vals[i]) for k,i in IDX.items()}, s

def phase(f: dict, target_pulse_seen: bool, one_move_seen: bool) -> str:
    if iv(f,"needle_fault") or iv(f,"p111_fault"):
        return "FAULT"
    if not iv(f,"needle_zero_valid"):
        return "WAIT_REFERENCE"
    if not iv(f,"preflight_ready"):
        return "WAIT_PREFLIGHT"
    if not iv(f,"pe9_debounced_open"):
        return "READY_PE9_CONNECTED"
    if not iv(f,"flight_active"):
        return "WAIT_FLIGHT"
    if not iv(f,"actuator_authorized"):
        return "WAIT_AUTH"

    base = iv(f,"p111_baseline_adc")
    tgt = iv(f,"p111_target_adc")
    moves = iv(f,"p111_moves_completed")
    p110 = iv(f,"p110_state")

    if p110 in (1,2,3,4):
        if target_pulse_seen and base and abs(tgt-base) <= 8:
            return "CLOSING"
        return "OPENING"

    if base and abs(tgt-base) > 40:
        if moves >= 1 or one_move_seen:
            return "OPEN_DWELL"
        return "OPEN_TARGET"

    if target_pulse_seen and moves >= 2:
        return "CYCLE_DONE"

    if target_pulse_seen and one_move_seen:
        return "RETURN_CLOSED_WAIT"

    return "AUTHORIZED_HOLD"

def status_line(f: dict, pulse: bool, one_move: bool) -> str:
    t = iv(f,"time_ms") / 1000.0
    return (
        f"t={t:7.2f}s PHASE={phase(f,pulse,one_move):>18s} | "
        f"PE9={'OPEN' if iv(f,'pe9_debounced_open') else 'CONN'} "
        f"READY={iv(f,'preflight_ready')} FLT={iv(f,'flight_active')} "
        f"AUTH={iv(f,'actuator_authorized')} SYSF={iv(f,'system_fault')} | "
        f"ADC={iv(f,'needle_adc'):4d} TGT={iv(f,'needle_target_adc'):4d} "
        f"ZERO={iv(f,'needle_zero_adc'):4d} EN={iv(f,'needle_enabled')} "
        f"NF={iv(f,'needle_fault')} | "
        f"P83={iv(f,'p83_feedback_valid')}/M{iv(f,'p83_mode')} "
        f"C={iv(f,'p83_confidence_pct'):2d}% | "
        f"P111={P111_STATE.get(iv(f,'p111_state'),iv(f,'p111_state'))} "
        f"TGT={iv(f,'p111_target_adc'):4d} CMD={iv(f,'p111_command_sequence'):3d} "
        f"MOV={iv(f,'p111_moves_completed')} "
        f"F={P111_FAULT.get(iv(f,'p111_fault'),iv(f,'p111_fault'))} | "
        f"P110={P110_STATE.get(iv(f,'p110_state'),iv(f,'p110_state'))} "
        f"{P110_RESULT.get(iv(f,'p110_result'),iv(f,'p110_result'))} "
        f"PWM={iv(f,'p110_active_pwm'):3d} BRK={iv(f,'p110_breakaway_found')} "
        f"SPD={iv(f,'p110_speed_adc_s'):4d} STOP={iv(f,'p110_stop_distance_adc'):3d} "
        f"COAST={iv(f,'p110_coast_max_adc'):3d} CORR={iv(f,'p110_correction_count'):2d} | "
        f"DEC={iv(f,'p112_decel_active')}/{iv(f,'p112_decel_pwm'):3d} "
        f"HO={iv(f,'p112_hard_off_count')} "
        f"ISR={iv(f,'p112_isr_last_us'):3d}/{iv(f,'p112_isr_max_us'):3d}us"
    )

def serial_api():
    try:
        import serial
        from serial.tools import list_ports
        return serial, list_ports
    except ImportError:
        print("pyserial kurulu degil: py -m pip install pyserial")
        raise

def choose_port(requested: str | None) -> str:
    if requested:
        return requested.upper()

    _, list_ports = serial_api()
    ports = list(list_ports.comports())
    for i,p in enumerate(ports,1):
        print(f"{i}) {p.device} - {p.description}")

    if len(ports) == 1:
        return ports[0].device

    ans = input("COM port: ").strip().upper()
    if ans.isdigit() and 1 <= int(ans) <= len(ports):
        return ports[int(ans)-1].device
    return ans

def serial_lines(port: str, baud: int, duration: float) -> Iterable[str]:
    serial, _ = serial_api()
    start = time.monotonic()

    with serial.Serial(port, baudrate=baud, timeout=0.5) as s:
        try:
            s.reset_input_buffer()
        except Exception:
            pass

        print("="*78)
        print(" P112R10R3 GNC MOTOR BENCH / SETTLED TARGET HOLD - READ ONLY UART")
        print(f" {port} @ {baud}")
        print(" BASINCSIZ / INERT BENCH ONLY. RCS + VENT FIZIKSEL OLARAK AYRIK.")
        print(" PA0/UART motor komutu yok. Ctrl+C = durdur.")
        print("="*78)

        while duration <= 0 or (time.monotonic()-start) < duration:
            b = s.readline()
            if b:
                yield b.decode("ascii", errors="replace")

def file_lines(path: str) -> Iterable[str]:
    with Path(path).open("r", encoding="utf-8", errors="replace") as h:
        for line in h:
            s = line.strip()
            if s.startswith("{"):
                try:
                    d = json.loads(s)
                    if isinstance(d.get("raw_frame"), str):
                        yield d["raw_frame"]
                        continue
                except Exception:
                    pass
            yield line

def run(args) -> int:
    if args.capture:
        lines = file_lines(args.capture)
        log = None
    else:
        port = choose_port(args.port)
        lines = serial_lines(port, args.baud, args.duration)
        log = Path(args.log).open("w", encoding="utf-8", buffering=1)
        log.write(
            f"# P112R10R3 GNC motor bench settled-target hold UART log | "
            f"port={port} baud={args.baud} expected_fields={EXPECTED_FIELDS}\n"
        )

    valid = 0
    rejected = 0
    latest = None
    last_phase = None

    preflight_seen = False
    pe9_seen = False
    flight_seen = False
    auth_seen = False
    target_pulse_seen = False
    target_returned_seen = False
    p110_motion_seen = False
    breakaway_seen = False
    pwm_seen = False
    one_move_seen = False

    max_moves = 0
    max_moves_before_target_return = 0
    min_adc = 65535
    max_adc = 0
    min_target = 65535
    max_isr_us = 0
    max_hard_off = 0
    max_sd_supp = 0
    max_uart_supp = 0

    try:
        for raw in lines:
            s = raw.strip()
            if not s.startswith(FRAME_PREFIX):
                continue

            try:
                f, frame = decode(s)
            except Exception as e:
                rejected += 1
                print(f"REJECT: {e}", file=sys.stderr)
                continue

            valid += 1
            latest = f

            preflight_seen |= bool(iv(f,"preflight_ready"))
            pe9_seen |= bool(iv(f,"pe9_debounced_open"))
            flight_seen |= bool(iv(f,"flight_active"))
            auth_seen |= bool(iv(f,"actuator_authorized"))

            base = iv(f,"p111_baseline_adc")
            tgt = iv(f,"p111_target_adc")
            adc = iv(f,"needle_adc")
            moves = iv(f,"p111_moves_completed")

            if base and abs(tgt-base) > 40:
                target_pulse_seen = True
                max_moves_before_target_return = max(
                    max_moves_before_target_return, moves)
            if target_pulse_seen and base and abs(tgt-base) <= 8 and auth_seen:
                target_returned_seen = True

            p110_motion_seen |= iv(f,"p110_state") in (1,2,3,4)
            breakaway_seen |= bool(iv(f,"p110_breakaway_found"))
            pwm_seen |= iv(f,"p110_active_pwm") > 0
            one_move_seen |= moves >= 1

            max_moves = max(max_moves, moves)
            min_adc = min(min_adc, adc)
            max_adc = max(max_adc, adc)
            if tgt:
                min_target = min(min_target, tgt)

            max_isr_us = max(max_isr_us, iv(f,"p112_isr_max_us"))
            max_hard_off = max(max_hard_off, iv(f,"p112_hard_off_count"))
            max_sd_supp = max(max_sd_supp, iv(f,"p112_sd_suppressed_count"))
            max_uart_supp = max(max_uart_supp, iv(f,"p112_uart_suppressed_count"))

            ph = phase(f,target_pulse_seen,one_move_seen)
            if ph != last_phase:
                print()
                print(f">>> PHASE: {ph}")
                last_phase = ph

            print(status_line(f,target_pulse_seen,one_move_seen), flush=True)

            if log:
                d = dict(f)
                d["phase"] = ph
                d["raw_frame"] = frame
                log.write(json.dumps(d, sort_keys=True) + "\n")

    except KeyboardInterrupt:
        print("\nMonitor durduruldu.")
    finally:
        if log:
            log.close()

    print()
    print("======================== P112R10R3 OZET ========================")
    print(f"Valid frame             : {valid}")
    print(f"Rejected frame          : {rejected}")
    print(f"Preflight READY         : {'PASS' if preflight_seen else 'NO'}")
    print(f"PE9 OPEN                : {'PASS' if pe9_seen else 'NO'}")
    print(f"flight_active=1         : {'PASS' if flight_seen else 'NO'}")
    print(f"authorization=1         : {'PASS' if auth_seen else 'NO'}")
    print(f"GNC target OPEN pulse   : {'PASS' if target_pulse_seen else 'NO'}")
    print(f"Target returned CLOSED  : {'PASS' if target_returned_seen else 'NO'}")
    print(f"P110 motion observed    : {'PASS' if p110_motion_seen or max_moves >= 1 else 'NO'}")
    print(f"Breakaway observed      : {'YES' if breakaway_seen else 'not sampled'}")
    print(f"PWM nonzero sampled     : {'YES' if pwm_seen else 'not sampled'}")
    print(f"moves_completed max     : {max_moves}")
    print(f"moves before CLOSE tgt  : {max_moves_before_target_return} "
          f"({'PASS' if max_moves_before_target_return == 1 else 'CHECK'})")
    if min_target != 65535:
        print(f"Minimum P111 target     : {min_target}")
    if min_adc != 65535:
        print(f"Needle ADC range        : {min_adc} .. {max_adc}")
    print(f"P112 ISR max            : {max_isr_us} us")
    print(f"hard_off max            : {max_hard_off}")
    print(f"SD suppressed max       : {max_sd_supp}")
    print(f"UART suppressed max     : {max_uart_supp}")

    if latest is None:
        print("SONUC: FAIL - gecerli frame yok.")
        return 2

    base = iv(latest,"p111_baseline_adc")
    final_adc = iv(latest,"needle_adc")
    final_tgt = iv(latest,"p111_target_adc")
    final_p110_result = iv(latest,"p110_result")

    print()
    print("Son frame:")
    print(f"  baseline              = {base}")
    print(f"  needle_adc            = {final_adc}")
    print(f"  p111_target_adc       = {final_tgt}")
    print(f"  moves_completed       = {iv(latest,'p111_moves_completed')}")
    print(f"  p110_state            = {P110_STATE.get(iv(latest,'p110_state'),iv(latest,'p110_state'))}")
    print(f"  p110_result           = {P110_RESULT.get(final_p110_result,final_p110_result)}")
    open_err = iv(latest,'p111_last_open_error_adc')
    close_err = iv(latest,'p111_last_close_error_adc')
    print(f"  last open error       = {open_err}")
    print(f"  last close error      = {close_err}")
    print(f"  open accuracy         = {'NOMINAL <=8' if abs(open_err) <= 8 else ('SETTLED <=12' if abs(open_err) <= 12 else ('ADAPTIVE SETTLED <=20' if (open_err > 0 and open_err <= 20) else 'OUTSIDE ADAPTIVE BAND'))}")
    print(f"  close accuracy        = {'NOMINAL <=8' if abs(close_err) <= 8 else ('SETTLED GUARD <=12' if abs(close_err) <= 12 else 'OUTSIDE 12')}")
    print(f"  learned open PWM      = {iv(latest,'p111_learned_open_pwm')}")
    print(f"  learned close PWM     = {iv(latest,'p111_learned_close_pwm')}")
    print(f"  learned open coast    = {iv(latest,'p111_learned_open_coast')}")
    print(f"  learned close coast   = {iv(latest,'p111_learned_close_coast')}")
    print(f"  needle_fault          = {iv(latest,'needle_fault')}")
    print(f"  p111_fault            = {P111_FAULT.get(iv(latest,'p111_fault'),iv(latest,'p111_fault'))}")

    final_closed = bool(base) and abs(final_adc-base) <= 12 and abs(final_tgt-base) <= 8

    fail = (
        valid == 0
        or not preflight_seen
        or not pe9_seen
        or not flight_seen
        or not auth_seen
        or not target_pulse_seen
        or not target_returned_seen
        or max_moves != 2
        or max_moves_before_target_return != 1
        or not final_closed
        or iv(latest,"needle_fault") != 0
        or iv(latest,"p111_fault") != 0
        or max_hard_off != 0
        or iv(latest,"p110_active_pwm") != 0
    )

    print()
    if fail:
        print("SONUC: P112R10R3 TAM PASS DEGIL - logu bana gonder.")
        return 1

    print("SONUC: P112R10R3 SAME-TARGET HOLD + OPEN/CLOSE REAL MOTOR CYCLE PASS")
    print("Logu bana gonder; OPEN/CLOSE breakaway, coast, correction ve timing'i inceleyelim.")
    return 0

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("capture", nargs="?")
    p.add_argument("--port")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--duration", type=float, default=40.0)
    p.add_argument("--log", default="uart_p112r10r3_gnc_motor_bench.txt")
    return run(p.parse_args())

if __name__ == "__main__":
    raise SystemExit(main())
