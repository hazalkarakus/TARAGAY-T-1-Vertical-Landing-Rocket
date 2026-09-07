# P42 — Scheduler Rollback / Soak Candidate

P42 is intentionally a minimal change from P41.

## Change
Scheduler execution priority is restored to the P40 order:

`IMU -> ESKF -> LIDAR -> BARO -> NRF -> MON -> COV`

Task-array indices, periods, phases and UART field ordering are unchanged.

## Preserved from P40/P41
- IMU software-SPI Mode-3, 12-NOP edge timing and medium GPIO slew
- IMU pattern quick-retry and non-blocking recovery
- LiDAR non-blocking state machine/recovery and intentional-defer behavior
- ESKF public/correction architecture and divergence/reacquisition safety
- Barometer raw-spike guard
- SD 200 Hz logging, fast CRC path, 1-bit SDIO and runtime recovery
- Session-safe V14 decoder
- `fast_fault OR system_fault` physical actuator inhibit
- $TGY55 UART schema: 214 fields

## Test target
Run a 5–10 minute unpressurized soak test. Desired results: IMU/ESKF/baro misses zero, LiDAR misses/re-aligns near zero, SD dropped/overruns zero, ESKF public ~200 Hz, LiDAR physical ~170–200 Hz, covariance ~22–25 Hz.
