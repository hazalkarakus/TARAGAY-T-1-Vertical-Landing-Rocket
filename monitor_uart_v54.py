#!/usr/bin/env python3
"""Validate and monitor TGY V54 USART2 diagnostic frames.

Examples:
  python monitor_uart_v54.py --port COM5
  python monitor_uart_v54.py --port /dev/ttyUSB0
  python monitor_uart_v54.py uart_capture.txt
  type uart_capture.txt | python monitor_uart_v54.py -
"""

from __future__ import annotations

import argparse
import binascii
import json
import sys
from pathlib import Path
from typing import Iterable


FIELDS = (
    "frame", "seq", "time_ms", "preflight_state", "flight_active",
    "pe9_raw_open", "pe9_debounced_open", "preflight_ready",
    "preflight_fault", "flight_time_ms", "imu_valid", "baro_valid",
    "lidar_valid", "eskf_init", "eskf_ok", "baro_fresh", "lidar_fresh",
    "vertical_source_mask", "vertical_degraded", "eskf_z_mm",
    "eskf_vz_mmps", "baro_alt_mm", "lidar_mm", "baro_pressure_pa",
    "baro_innov_mm", "lidar_innov_mm", "baro_updates", "lidar_updates",
    "baro_rejects", "lidar_rejects", "lidar_state", "lidar_age_ms",
    "lidar_reads", "lidar_errors", "roll_cdeg", "pitch_cdeg",
    "roll_rate_mdps", "pitch_rate_mdps", "rcs_state", "rcs_fault",
    "rcs_req_mask", "rcs_applied_mask", "rcs_roll_pd_us",
    "rcs_pitch_pd_us", "rcs_roll_pulse_rem_ms",
    "rcs_pitch_pulse_rem_ms", "rcs_roll_cooldown_rem_ms",
    "rcs_pitch_cooldown_rem_ms", "rcs_autodamp_mask", "rcs_dry_run",
    "rcs_event_count", "main_cmd_x10000", "main_target_force_cN",
    "main_est_thrust_cN", "main_burn_active", "main_output_valid",
    "main_physical_enabled", "needle_req_x10000",
    "needle_limited_x10000", "needle_adc", "needle_target_adc",
    "needle_enabled", "needle_lock", "needle_fault", "nrf_connected",
    "nrf_link", "nrf_cmd", "nrf_flags", "nrf_age_ms", "nrf_rx_count",
    "stop_latched", "sd_ready", "sd_logging", "cpu_x100", "system_ok",
    "system_fault", "uart_dma_errors", "uart_busy_skips",
)


def decode_frame(line: str) -> dict[str, str]:
    line = line.strip()
    if not line.startswith("$TGY54,"):
        raise ValueError("not a $TGY54 frame")
    if "*" not in line:
        raise ValueError("CRC separator is missing")

    body, crc_text = line.rsplit("*", 1)
    if len(crc_text) != 4:
        raise ValueError("CRC must contain four hex digits")

    received_crc = int(crc_text, 16)
    calculated_crc = binascii.crc_hqx(body.encode("ascii"), 0xFFFF)
    if received_crc != calculated_crc:
        raise ValueError(
            f"CRC mismatch: received={received_crc:04X} "
            f"calculated={calculated_crc:04X}"
        )

    values = body.split(",")
    if len(values) != len(FIELDS):
        raise ValueError(
            f"field count mismatch: received={len(values)} "
            f"expected={len(FIELDS)}"
        )

    return dict(zip(FIELDS, values))


def serial_lines(port: str, baud: int) -> Iterable[str]:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "Serial monitoring requires pyserial: pip install pyserial"
        ) from exc

    with serial.Serial(port, baudrate=baud, timeout=1.0) as connection:
        while True:
            raw = connection.readline()
            if raw:
                yield raw.decode("ascii", errors="replace")


def input_lines(path: str | None) -> Iterable[str]:
    if path in (None, "-"):
        yield from sys.stdin
        return

    with Path(path).open("r", encoding="ascii", errors="replace") as source:
        yield from source


def compact_status(frame: dict[str, str]) -> str:
    return (
        f"seq={frame['seq']} t={frame['time_ms']}ms "
        f"flight={frame['flight_active']} PE9={frame['pe9_debounced_open']} "
        f"src={frame['vertical_source_mask']} "
        f"fresh(B/L)={frame['baro_fresh']}/{frame['lidar_fresh']} "
        f"z={int(frame['eskf_z_mm']) / 1000.0:+.3f}m "
        f"vz={int(frame['eskf_vz_mmps']) / 1000.0:+.3f}m/s "
        f"RCS={frame['rcs_applied_mask']} events={frame['rcs_event_count']} "
        f"main={int(frame['main_cmd_x10000']) / 10000.0:.4f} "
        f"NRF={frame['nrf_link']} STOP={frame['stop_latched']} "
        f"UARTerr={frame['uart_dma_errors']} skip={frame['uart_busy_skips']}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", nargs="?", help="capture file or '-' for stdin")
    parser.add_argument("--port", help="serial port, for example COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--json", action="store_true", help="print every valid frame as JSON")
    args = parser.parse_args()

    lines = serial_lines(args.port, args.baud) if args.port else input_lines(args.capture)
    valid = 0
    rejected = 0

    try:
        for line in lines:
            if not line.startswith("$TGY54,"):
                if line.startswith("#"):
                    print(line.rstrip())
                continue
            try:
                frame = decode_frame(line)
            except (ValueError, UnicodeError) as exc:
                rejected += 1
                print(f"REJECT[{rejected}]: {exc}", file=sys.stderr)
                continue

            valid += 1
            print(json.dumps(frame, sort_keys=True) if args.json else compact_status(frame))
    except KeyboardInterrupt:
        pass

    print(f"valid={valid} rejected={rejected}", file=sys.stderr)
    return 0 if rejected == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
