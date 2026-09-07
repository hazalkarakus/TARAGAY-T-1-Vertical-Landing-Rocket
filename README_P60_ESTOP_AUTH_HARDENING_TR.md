# P60 - E-STOP Actuator Authorization Hardening

## Amaç
P59 NRF/tahliye/E-STOP entegrasyonu RF ve fiziksel output katmanında çalıştı. P59 testinde E-STOP latch aktif olduktan sonra fiziksel RCS ve needle çıkışları güvenli kalmasına rağmen UART `actuator_authorized=1` gösterebildi. P60 yalnızca bu mimari tutarsızlığı kapatır.

## Değişiklik
Normal uçuş aktüatörleri için tek bir merkezi karar eklendi:

`App_IsActuatorAuthorized()` yalnız şu koşulların tamamında 1 döndürür:
- PE9 flight gate aktif,
- preflight fault yok,
- latched remote E-STOP yok,
- SystemMonitor actuator fault yok,
- Full-State ESKF output inhibit değil.

Bu karar 1 kHz RCS servisinde, 200 Hz RCS güncellemesinde, generated-main/needle fiziksel yolunda ve UART `actuator_authorized` alanında kullanılır.

## Savunma katmanları
P60 STOP davranışını tek gate'e indirmez. P59'daki son-output korumaları aynen kalır:
- `AttitudeControl_ForceSafe()`
- `SolenoidOutput_ForceSafe()`
- `NeedleValveController_Stop()`
- STOP latch reset/power-cycle olmadan temizlenmez.

Böylece STOP sonrası hem **yetki 0** olur, hem de fiziksel output katmanı ayrıca zorla safe tutulur.

## Değişmeyenler
NRF24 driver, RemoteControl packet parser, tahliye sıralaması, solenoid driver, IMU, ESKF, barometre, LiDAR, SDLogger ve yer istasyonu TX P59 ile aynıdır. RF adresi/kanal/payload değiştirilmemiştir.

## Kabul testi
İlk test gazsız ve mümkünse solenoid güç hattı kapalı yapılmalıdır.

1. TX açık, link=1, invalid=0 doğrula.
2. Switch-1 (PA1-GND) ile tahliye maskelerini 1->0->2->0->4->0->8->0 izle.
3. Switch-1 basılıyken Switch-2 (PA0-GND) bas.
4. En geç bir sonraki UART frame'de:
   - `stop_latched=1`
   - `actuator_authorized=0`
   - `rcs_req_mask=0`
   - `rcs_applied_mask=0`
   - `needle_rpwm=0`, `needle_lpwm=0`
5. Switch-2'yi bırak; latch 1 ve authorization 0 kalmalı.
6. STOP latched halde PE9 flight-active olsa bile `actuator_authorized` kesinlikle 0 kalmalı. Bu P60'ın ana kabul kriteridir.
7. Reset/power-cycle sonrası, STOP yok ve uçuş/ESKF sağlıklıysa authorization normal şekilde tekrar 1 olabilir.

Aynı testte `imu_recovery_count` da izlenmelidir. P59'da E-STOP yakınında bir full IMU recovery görüldü; tekrar ederse bobin/relay EMI veya besleme transienti ayrıca incelenmelidir.

## Güvenlik notu
NRF üzerinden STOP bir yazılım güvenlik katmanıdır; MCU veya güç zinciri arızasında tek başına gerçek donanımsal E-STOP yerine geçmez. Final sistemde aktüatör/solenoid enerjisini bağımsız kesebilen NC hardwired kill loop önerilir.
