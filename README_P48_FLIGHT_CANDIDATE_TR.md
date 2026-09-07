# TARAGAY-T1 P48 — Flight Candidate Hardening

P48, P47 uzun soak testinde kalan iki gerçek problemi ve uçuş interlock tutarsızlıklarını hedefler.

## 1. ESKF gravity düzeltmesi

P47'de tek covariance fault yine `GRAV -> BGY` olarak görülmüştü. P48'de gravity ölçümünün Kalman gain'i yalnızca ilgili iki attitude error state'ine (`THX/THY/THZ`) sınırlandı.

- Position / velocity gravity tarafından doğrudan inject edilmez.
- Accel bias / gyro bias gravity tarafından doğrudan inject edilmez.
- Gyro bias için mevcut stationary bias correction yolu korunur.
- Covariance update, iki non-zero K satırı için exact sparse Joseph biçimindedir.
- 2x2 attitude bloğu double precision hesaplanır.
- P46 state-preserving covariance rollback ve public z/vz guard aynen korunur.

Bu değişiklik hem `BGY` negatif variance kök nedenini ortadan kaldırmayı hem de P47 full-matrix Joseph CPU maliyetini düşürmeyi amaçlar.

## 2. SD stall buffering

P47'de SD hata vermeden 75 frame kaybı oluşmuş, ring high-water 127'ye ulaşmıştı. P48:

- CCM capture ring: 128 frame / 49,152 byte — değişmedi.
- DMA writer buffer: 2 -> 4 adet.
- Her writer buffer: 9,216 byte / 24 V14 frame.
- Writer SRAM toplamı: 36,864 byte.
- Ek writer toleransı: P47'ye göre yaklaşık +240 ms.
- Backpressure başlangıcı: 48 -> 32 frame.
- Critical threshold: 96 -> 72 frame.
- Critical drain: 12 frame / 760 us'a kadar.
- Guard yalnız ring <= 8 frame iken ve DATA kuyruğu yokken başlatılır.
- DATA her zaman GUARD'dan önceliklidir.

V14 wire format 384 byte / 200 Hz ve 128 MiB preallocation + 96 MiB fallback değişmedi.

## 3. Uçuş interlock düzeltmesi

V50 vertical policy zaten uçuşta LIDAR-only veya BARO-only degraded operation'a izin veriyordu; fakat SystemMonitor ayrı ayrı LIDAR/BARO kaybını global actuator fault yapıyordu. Bu P48'de düzeltildi.

Uçuş öncesi:
- IMU hazır olmalı.
- ESKF hazır olmalı.
- LIDAR referansı hazır/fresh olmalı.
- Barometer referansı hazır/fresh olmalı.
- Needle zero/fault kontrolü geçmeli.
- **SD logger ready + logging olmalı.**

Uçuş başladıktan sonra:
- IMU ve ESKF freshness/health kritik olmaya devam eder.
- Scheduler stall kritik olmaya devam eder.
- ESKF divergence kritik olmaya devam eder.
- Dikey aiding için fresh LIDAR **veya** fresh barometer yeterlidir.
- İki vertical aiding kaynağı da kaybolursa actuator fault oluşur.
- SD logger uçuşta hata verirse kayıt durabilir fakat RCS/needle sadece logger yüzünden kapatılmaz.

Bu davranış, recorder arızasının çalışan uçuş kontrolünü öldürmesini engellerken SD'yi preflight şartı yapar.

## 4. UART / SD araçları

P48 UART frame:

- `$TGY59`
- 253 data field
- 115200 8N1 USART2

Monitor:

```powershell
python -u monitor_uart_p48.py --port COM8
```

JSON görünümü:

```powershell
python -u monitor_uart_p48.py --port COM8 --json
```

Monitor otomatik `uart_p48.txt` kaydeder.

SD decoder:

```powershell
python decode_flight_v14_p48.py FLIGHT.BIN
```

## 5. Uçuş adayı kabul kriterleri

P48'in gerçek uçuş için kabul edilmesi için aşağıdaki bench/HIL testleri geçmelidir:

1. CubeIDE Clean + Build: 0 error, RAM overflow yok.
2. 5 dk kısa bench: covariance fault=0, SD drop=0.
3. En az 25 dk soak: covariance fault=0, SD drop=0, ring high-water <127.
4. IMU recovery testi: recovery failure=0 ve actuator gate stale süre boyunca SAFE.
5. LIDAR dropout testi: barometer fresh iken vertical control degraded olarak devam etmeli.
6. Barometer dropout testi: LIDAR fresh iken vertical control degraded olarak devam etmeli.
7. İki vertical source birlikte kayıp: main output SAFE/CLOSED ve actuator fault aktif olmalı.
8. Uçuş sırasında SD çıkarma/yazma fault simülasyonu: logging fault görünmeli fakat yalnız SD nedeniyle RCS/needle inhibit olmamalı.
9. PE9 / STOP / ESKF inhibit interlock testleri tekrar PASS olmalı.
10. Aktüatör gücü kapalı dry-run sonrası düşük enerjili yer testi yapılmalı.

Yazılım paketi bu testlerin yapılması için flight-candidate durumundadır; fiziksel uçuş hazır kararı ancak bu kapılar geçildikten sonra verilmelidir.
