# V49 ESKF signed-PD RCS controller

The former 5/10-degree landing-envelope phase-plane controller is no longer
the physical RCS path.

The current controller consumes Full-State ESKF roll/pitch at 200 Hz and uses
bias-corrected body gyro rate:

```text
signed_time_ms = 2.50 * angle_deg + 3.55 * angular_rate_dps
```

- The sign selects the positive-error or negative-error correcting valve.
- Absolute time below 20 ms produces no shot.
- Absolute time above 60 ms is saturated to 60 ms.
- TIM7 closes roll and pitch shots independently with 1 ms resolution.
- Each axis enters a 100 ms cooldown after a shot.
- A derivative-driven sign reversal provides active damping before zero-angle
  crossing.
- PE9, ESKF health/freshness, output interlocks and the NRF STOP latch override
  every physical RCS command.

See the root `README_V49_ESKF_RCS_SIGNED_PD_TR.md` for the full Turkish
integration and test procedure.
