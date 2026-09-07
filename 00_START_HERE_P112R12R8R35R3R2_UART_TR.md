# P112R12R8R35R3R2 UART PHASE RETRY HOTFIX

- Base: R8R35R3R1
- Control / ESKF / RCS / needle logic unchanged.
- SD R8R35R3 service closure unchanged.
- Only UART background admission behavior changed.

## Root cause
R3R1 updated `last_uart_attempt_us` even when the 600 us IMU-slack check failed.
That could lock every UART attempt to the same unfavorable phase of the 1 kHz scheduler.

## Fix
On a slack defer, `last_uart_attempt_us` is no longer advanced. The main loop retries until a safe >=600 us window exists. The UART telemetry implementation itself remains 10 Hz.

## Test
Use the same R8R35R3 SD monitor on COM21. UART should start producing frames without changing control behavior.
