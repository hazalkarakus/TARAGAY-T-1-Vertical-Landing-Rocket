# P40 — Final Hardening / Soak Candidate

P40 is based on P39. The proven 200 Hz SD logger, non-blocking IMU/LiDAR recovery,
barometer raw-spike guard and bounded ESKF/LiDAR fusion are preserved.

## Changes

1. **Physical actuator inhibit = fast fault OR SystemMonitor fault.**
   The 1 kHz fast fault remains primary, but the slower 10 Hz fault snapshot also
   holds RCS/needle safe until it clears. This closes the ~100 ms restart window
   observed in the P39 UART log.
2. **IMU pattern quick retry.** A repeated-word triplet is discarded and one fresh
   3-burst triplet is attempted immediately. Only a second failure escalates to the
   existing ~45 ms non-blocking recovery.
3. **SW-SPI signal integrity.** Mode-3 remains; edge delay is 12 NOP and PA4/5/7 use
   medium GPIO slew to reduce ringing. Hardware SPI1 is still not re-enabled.
4. **LiDAR scheduler priority.** IMU -> ESKF -> LiDAR -> BARO -> NRF -> monitor ->
   covariance. The short 1 kHz state-machine service is no longer left behind
   lower-rate work; IMU remains protected by the same slack guard.
5. **flight.bin session boundary.** The V14 decoder stops when timestamps/counters
   roll back, so CRC-valid data left in a preallocated old tail is not exported as
   part of the new boot.
6. UART adds `imu_pattern_retries` and `imu_pattern_retry_success` (214 fields).

## First test

Keep gas/pressure and physical actuators isolated. Run 5–10 minutes if the first
60 s are clean. PASS targets: SD 200 Hz with zero drops; IMU misses 0; LiDAR misses
near zero; no frame with (`fast_fault != 0` OR `system_fault != 0`) and
`rcs_applied_mask != 0`; pattern retry success should absorb at least part of the
previous full IMU recoveries.
