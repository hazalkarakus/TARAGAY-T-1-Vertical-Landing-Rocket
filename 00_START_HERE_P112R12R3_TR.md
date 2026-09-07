# P112R12R3 - Clean Production Autonomous / 950+ / INERT

Bu surum P112R12R2/R2R1/R2R2 bench target-step ve breakaway-kick deneylerini production zincirinden tamamen cikartir.

## Bu surumde kalan production zinciri

`GeneratedFlightControl -> normal actuator authorization -> P111 autonomous supervisor -> P110/P112 adaptive actuator -> motor`

- R12R1 same-target anti-chatter guard korunur.
- CLOSED reference kabul alani 950..1023 ADC korunur.
- P110 normal adaptive breakaway/search/drive/brake/correction mantigi korunur.
- R10 sentetik GNC stimulus KAPALI.
- R11 PA0 commissioning KAPALI.
- R12R2 target-step KAPALI / kaynakta yok.
- R12R2R1 max-hold deneyi KAPALI / kaynakta yok.
- R12R2R2 255 PWM kick deneyi KAPALI / kaynakta yok.
- UART RX actuator komutu yok; UART yalniz diagnostik TX.
- RCS ve vent outputlari INERT validation icin hard-isolated.

## Ilk fiziksel testin amaci

Tamamen basincsiz/inert duzende sistemi ac, preflight READY'i bekle ve PE9'i ayir.
Gercek GeneratedFlightControl komutu CLOSED kalirsa beklenen davranis:

- `actuator_authorized=1`
- `main_output_valid=1`
- `needle_target_adc ~= p111_baseline_adc`
- `p111_moves_completed=0`
- `p110_state=WAIT`
- `p110_active_pwm=0`
- `needle_fault=0`, `p111_fault=0`
- `p112_hard_off_count=0`
- `rcs_applied_mask=0`

Gercek GNC target degistirirse P111/P110 normal production mantigi ile hareket edebilir. Bu build herhangi bir yapay OPEN step uretmez.

## Test

CubeIDE: Clean -> Build -> Flash

UART:

`py monitor_uart_p112r12r3_prod_auto_inert.py --port COM21 --duration 40`

Bu dosya DO NOT FLY'dir; Cortex-M4 target build ve fiziksel test ayri ayri dogrulanmadan flight qualification sayilmaz.
