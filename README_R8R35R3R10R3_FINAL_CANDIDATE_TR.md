# TARAGAY-T1 R8R35R3R10R3 — Final Candidate

Bu revizyon R3R10R2 tabanını korur ve son inert testte görülen iki problemi hedefler.

## 1. Uçuşta nRF command-priority RX
- PE9 / flight_active sonrasında roket yeni 32-byte nRF telemetry TX başlatmaz.
- Başlamış bir preflight TX yalnız non-blocking şekilde RX'e dönmeyi tamamlar.
- STOP / vent 4-byte heartbeat komutları için radyo PRX'te kalır.
- Link timeout olursa, async TX idle iken 250 ms aralıkla PRX tekrar assert edilir.
- 1 Hz mevcut configuration verifier/recovery korunur.

## 2. Needle HOLD-safe P83 quarantine
- Motor hareket etmiyorsa ve son doğrulanmış SAME-target HOLD konumu ölçülmüş hold cap içinde ise, geçici P83 quarantine kalıcı actuator fault üretmez.
- Motor ForceSafe/OFF kalır; feedback geri gelene kadar hareket yapılmaz.
- GNC gerçek target değiştirirse HOLD koruması kalkar; feedback invalid kalırsa mevcut 500 ms fault yolu çalışır.
- Aktif P110 hareketi sırasında feedback kaybı hâlâ anında abort/fault'tur.
- E-STOP safe-close live-feedback reacquire ve bounded override R3R10R2 ile aynıdır.

## Değişmeyenler
- TaragayFlightLogic, ESKF, RCS physical mapping, SD RAM-first 60 s logger, HAL_Delay runtime guard.
- Uçuş kontrol otoritesi ve safety interlockları değiştirilmedi.

## Final inert kabul
1. READY=1
2. PE9 ayır
3. 45–60 s çalıştır; nRF link/heartbeat devam etmeli
4. E-STOP ver
5. stop_latched=1, RCS=0, auth=0
6. Needle CLOSED reference civarına dönmeli
7. RAM->SD flush_complete=1
8. Flight boyunca yeni nRF telemetry TX start artmamalı
9. SD physical fault/drop/overflow/runtime HAL_Delay = 0

İlk doğrulama basınçsız/inert yapılmalıdır.
