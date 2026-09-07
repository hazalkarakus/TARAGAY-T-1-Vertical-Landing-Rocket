# TARAGAY-T1 P71 — Karşılıklı RCS Tahliye

P71, P70'in konnektör-sonrası grounded tahliye güvenliğini aynen korur ve yalnızca manuel tahliye sırasını değiştirir.

## Yeni tahliye sırası

Switch-1 basılı tutulduğunda artık tek tek 1 -> 2 -> 4 -> 8 yerine karşılıklı çiftler açılır:

- X çifti: X+ + X- = mask `0x03` (decimal 3)
- 50 ms tümü kapalı deadtime
- Y çifti: Y+ + Y- = mask `0x0C` (decimal 12)
- 50 ms tümü kapalı deadtime
- Switch-1 basılı kaldığı sürece döngü tekrar eder.

Her çiftin ON süresi P70 ile aynı: 100 ms.

Beklenen UART `rcs_applied_mask` dizisi:

`3 -> 0 -> 12 -> 0 -> 3 -> 0 -> 12 -> 0 ...`

Bu sayede aynı eksendeki iki karşıt RCS tahliye sırasında birlikte açılır ve ideal olarak net yönlendirme momenti/kuvveti azaltılır.

## Güvenlik korunuyor

- Dört valf aynı anda açılamaz (`0x0F` reddedilir).
- Çapraz/yarım çoklu maskeler reddedilir.
- Normal uçuş RCS kontrolünde X+ ve X- veya Y+ ve Y- aynı anda açılmaya HALA izin verilmez.
- Karşılıklı çift izni sadece `SolenoidOutput_SetGroundVentMask()` adlı manuel tahliye yolunda vardır.
- P70 post-separation grounded qualifier aynen korunur: PE9 ayrıldıktan sonra tahliye ancak LiDAR/ESKF ile yerde ve durağan durum doğrulanırsa açılabilir.
- E-Stop latch, NRF freshness, preflight/system actuator fault ve fail-close davranışı değişmedi.
- P69/V12 ground firmware ve çift taraflı NRF protokolü değişmedi.

## Değişen işlevsel rocket dosyaları

- `App/app.c` — vent sequencer 2 karşılıklı çifte dönüştürüldü.
- `App/Services/SolenoidOutput/solenoid_output.c/.h` — vent-only pair mask izni eklendi.
- `App/Common/app_version.h` — P71 sürümü.

IMU, SensorManager, barometre, LiDAR, ESKF, SDLogger, NRF24 driver, RemoteControl parser, NRF telemetry, NeedleValve, scheduler/app_tasks, PreflightTrigger ve SystemMonitor P70 ile byte-byte aynıdır.

## Test

İlk test gaz/basınç olmadan veya güvenli dummy/bench koşulunda yapılmalı.

Rocket:

`python -u monitor_uart_p71.py --port COM21 --log uart_p71.txt`

Ground P69/V12:

`python -u monitor_ground_station_p69.py --port COM20`

Beklenen:

- Switch-1 OFF: `rcs_applied_mask=0`
- Switch-1 ON: `3 -> 0 -> 12 -> 0` tekrarı
- PE9 ayrıldıktan ve grounded qualifier oluştuğunda aynı sıra devam edebilmeli
- Uçuşta/yüksekte grounded qualifier yoksa tahliye fail-closed kalmalı
- E-Stop sonrası `stop_latched=1`, `actuator_authorized=0`, `rcs_applied_mask=0`, needle PWM `0/0`
