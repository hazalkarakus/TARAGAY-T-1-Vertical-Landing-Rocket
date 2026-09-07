# P112R12R8R32 - REAL FLIGHT LOGIC -> PHYSICAL NEEDLE

Bu revizyonun tek yeni fiziksel authority'si ana iğne vanadır.

Zincir:
`real sensors -> Full-State ESKF -> fixed R8R30 IMU->rocket matrix -> Hover V19.6 -> Valve_Cmd/L_Hedef -> /0.30 normalization -> P111 -> P110/P112 -> motor`

- Hover V19.6 mission state PE9/`flight_active` yükselmeden ilerlemez.
- `L=0.30` mevcut 3-turn full-scale olarak `1.0` normalized command'e map edilir.
- P111/P110/P112 içindeki 585 ADC = 3 tur mekanik sınır korunur.
- RCS röle/solenoid fiziksel çıkışları bu revizyonda zorla SAFE/OFF'tur.
- Vent/servo inert-output isolation altında kalır.
- UART nominal flight logic için gerekli değildir.

## Basınçsız entegrasyon kontrolü
1. Sistemi kapalı needle konumunda aç.
2. `ready=1`, `system_ok=1`, `eskf_ok=1`, `cal_phase=12` bekle.
3. Basınçsız düzende PE9'u ayırarak `flight_active=1` oluştur.
4. `fl_state` 0->1 ilerlemeli; `fl_valve_x10000` ile `needle_cmd_x10000` ilişkisi yaklaşık `needle_cmd = fl_valve/0.30` olmalı.
5. Needle 585 ADC travel sınırını aşmamalı.
6. `rcs_mask=0` ve `fl_rcs_applied_mask=0` kalmalı.

Bu paket uçuş onayı değildir. 3.5 mm thrust map halen provisional ve absolute horizontal X/Y aiding halen yoktur.
