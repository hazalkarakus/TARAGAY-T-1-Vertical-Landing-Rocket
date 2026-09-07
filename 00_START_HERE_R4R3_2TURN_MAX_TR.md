# TARAGAY-T1 R4R3 — 2 TUR MAKSIMUM İĞNE VANA

Bu revizyon R4R2 göreli +3 m hedef / +4 m ana itki tavanı davranışını korur.

Değişiklik yalnız ana iğne vana maksimum fiziksel açıklık sınırıdır:
- Ölçülen kalibrasyon: 195 ADC / tur
- Eski maksimum: 585 ADC = 3 tur
- Yeni maksimum: 390 ADC = 2 tur

Uygulanan katmanlar:
- `App/Common/app_config.h`: `APP_R8R32_NEEDLE_MAX_TRAVEL_ADC = 390U`
- `App/Services/NeedleValveAutonomousControl/needle_valve_autonomous_control.c`: `P112R5_MAX_TRAVEL_ADC = 390U`
- `App/Modules/Control/NeedleValve/needle_valve_controller.c`: `NV_MAX_TURNS = 2.0f`, `NV_MAX_TRAVEL_ADC = 2 * 195`

Hover V19.6, RCS V7.13.4, PE9, ESKF, STOP/E-STOP, +3 m relative target ve +4 m ceiling mantığı değiştirilmemiştir.
