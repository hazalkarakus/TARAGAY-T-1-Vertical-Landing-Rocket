# P41 — Scheduler Final / Long Soak Candidate

P41 is based on P40. No sensor driver, SD logger, ESKF fusion, actuator safety,
barometer filtering, IMU software-SPI timing or recovery logic is changed.

## Changes

1. **Scheduler execution priority only:**
   `IMU -> LIDAR -> ESKF -> BARO -> COV -> NRF -> MON`.
   Task-array indices remain unchanged, so SystemMonitor and UART field mappings
   are not affected.
2. **LiDAR before ESKF:** the 1 kHz LiDAR state-machine call is normally only
   tens of microseconds. It now executes immediately after the 1 kHz IMU owner,
   preventing rare 1–1.6 ms ESKF corrections from making LiDAR miss a full 1 ms
   release.
3. **Covariance before low-priority work:** the deterministic 25 Hz covariance
   task is serviced before NRF snapshot and 10 Hz integration monitor when due.
   Its existing IMU-slack guard remains unchanged.
4. UART schema stays **214 fields** and V14 SD format stays unchanged. Existing
   P40 CSV decoder remains compatible.

## Long soak test

Run 5–10 minutes with gas/pressure and physical actuators isolated. PE9 may be
toggled to exercise flight authorization without energizing pneumatic hardware.

PASS targets:
- SD state = 200 Hz, `sd_dropped=0`, `sd_ring_overruns=0`.
- `miss_imu=0`, `miss_eskf=0`, LiDAR miss/re-align near zero.
- LiDAR physical update remains roughly 170–200 Hz with no errors/recoveries.
- ESKF public output remains approximately 200 Hz and covariance approaches 25 Hz.
- IMU full-recovery rate stays near the P40 level or better.
- No frame may have (`fast_fault != 0` OR `system_fault != 0`) together with
  `rcs_applied_mask != 0`.
