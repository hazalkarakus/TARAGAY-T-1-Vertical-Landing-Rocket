# TARAGAY-T1 R16 STM32 — PE9 Mission Start

Bu revizyonda görev başlangıç otoritesi **PE9**'dur.

- PE9 LOW: connector takılı / GND. R16 state machine ilerlemez. Normal ana iğne motoru ve RCS uçuş otoritesi kapalıdır.
- PE9 HIGH (debounced) + preflight_ready: `flight_active` tek yönlü latch olur ve R16 görev zamanı başlar.
- İlk R16 kontrol adımı PE9 sonrasında `INIT` ile başlar; ardından `SELF_CHECK -> PREPOSITION -> READY -> ASCENT -> CAPTURE -> HOVER`.
- `release_event=1` mission epoch boyunca normaldir. Bu nedenle Simulink'teki pre-state-2 unexpected-release safety testi STM32 mimarisinde kullanılmaz; PE9'nin readiness öncesi erken ayrılması `PreflightTrigger` tarafından fault-latched edilerek görevin hiç başlamamasıyla ele alınır.
- RCS ve normal main-needle fiziksel handoff ayrıca `flight_active` ile hard-gatedir.
- Final vertical constants korunur: +3 m hedef, +/-4 m latched safety, b_hat DOWN=0.050 / UP=0.015, fast authority, 3-turn nonlinear Cv mapping.

## Build status
Bu ortamda `arm-none-eabi-gcc` mevcut değildir; gerçek STM32 target compile/link CubeIDE'de yapılmalıdır.
