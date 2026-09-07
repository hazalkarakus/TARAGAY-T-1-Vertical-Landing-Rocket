# TARAGAY-T1 P64 — Explicit TDD NRF Telemetri

## Amaç
P61/P62 ACK-Payload yaklaşımı kaldırıldı. P64, P60'ta kanıtlanmış statik 4-byte komut + normal Auto-ACK ayarlarını geri getirir ve roket -> yer istasyonu sensör telemetrisini ayrı kısa RF zaman diliminde gönderir.

## Güvenlik / değişmeyen yol
- Switch-1 PA1->GND: GROUND VENT / tahliye.
- Switch-2 PA0->GND: latched E-STOP.
- P60 `remote_control.c` byte-byte değişmeden korunmuştur.
- P60 scheduler/app_tasks, IMU, baro, LiDAR, AttitudeEstimator, Full-State ESKF, SDLogger ve NeedleValveController kritik kaynakları byte-byte aynıdır.
- Telemetri aktüatör komutu üretmez; yalnız read-only snapshot alır.
- E-STOP/tahliye parser'ı telemetriye bağlı değildir.

## RF ayarı
Her iki tarafta normal komut yolu:
- Address: `TGY01`
- Channel: 76
- 1 Mbps, 0 dBm
- 2-byte hardware CRC
- EN_AA pipe0 = 1
- FEATURE = 0x00
- DYNPD = 0x00
- Komut payload = 4 byte

ACK Payload / Dynamic Payload KULLANILMAZ.

## TDD akışı
1. Yer istasyonu PTX olarak 4-byte komutu gönderir.
2. Roket PRX normal hardware ACK verir ve komutu işler.
3. Yer istasyonu TX_DS sonrası 30 ms'lik PRX penceresine geçer, RX_PW_P0=32 yapar.
4. Roket geçerli komutu gördükten yaklaşık 4.5 ms sonra non-blocking şekilde PTX olur ve 32-byte telemetri sayfası yollar.
5. Yer istasyonu paketi alır, CRC16 doğrular, PA9 USART1 üzerinden PC'ye iletir ve tekrar PTX olur.
6. Roket telemetri TX tamamlanınca non-blocking şekilde tekrar 4-byte PRX komut moduna döner.

Roketin runtime TDD rol geçişinde `HAL_Delay()` yoktur.

## Telemetri
32-byte frame, CRC16-CCITT. 19 round-robin sayfa:
- system / flight / auth / stop / RCS / needle / SD
- IMU raw gyro/accel
- IMU scaled + filtered gyro
- IMU accel + norm
- filtered accel + norm
- baro pressure/filter/ground/altitude
- baro vertical speed/temp/median/D1/D2
- LiDAR raw/median/filtered/state
- attitude quaternion
- roll/pitch/yaw + world acceleration
- world specific force
- ESKF XYZ position + XY velocity
- ESKF Vz + world acceleration + vibration
- accel/gyro biases
- innovations + vertical consistency/reacq
- ESKF quaternion
- ESKF pitch/yaw/references/gravity weight
- freshness/update counts
- ESKF/radio diagnostic counters

P60 komut heartbeat'i 100 ms korunmuştur. Bu nedenle tüm 19 sayfanın bir turu yaklaşık 1.9 s sürer; her telemetri paketinin common header'ında system/IMU/baro/LiDAR/ESKF/flight/auth/stop bitleri bulunur ve bunlar 10 Hz yenilenir.

## Yer istasyonu pinleri
- PA1 -> Switch-1 -> GND : TAHLIYE
- PA0 -> Switch-2 -> GND : E-STOP
- PA2 = NRF IRQ
- PA3 = NRF CSN
- PA4 = NRF CE
- PA5 = SCK
- PA6 = MISO
- PA7 = MOSI
- PC13 = status LED
- PA9 = USART1 TX -> USB-TTL RX
- GND -> USB-TTL GND

USB-TTL 3.3 V logic kullanın. Kart ayrı besleniyorsa USB-TTL VCC bağlamak gerekmez.

## Kurulum
### Roket
`TGY_V8_19M_SMALLBOARD_FULLSYSTEM_P64_NRF_TDD_NOACK.zip`
- CubeIDE import
- Clean Project
- Build Project
- STM32F407'ye flash

### Yer istasyonu
`nrf_verici_2SWITCH_LED_V8_TDD_TELEMETRY.zip`
- CubeIDE import
- Clean Project
- Build Project
- BluePill'e flash

## Python
`monitor_ground_station_p63.py`

```bash
py -m pip install pyserial
python -u monitor_ground_station_p63.py --port COM8
```

Oluşan dosyalar:
- `ground_p63_packets.jsonl`
- `ground_p63_summary.txt`

Normal ilk kabul:
- `radio=1`
- `FEATURE=0x00`
- `DYNPD=0x00`
- `TX_OK` sürekli artar
- `TX_FAIL` 0 veya çok düşük
- `TLM_RX` sürekli artar
- `TLM_BAD=0`
- `pages=19/19`
- `TDD_TIMEOUT` idealde 0 veya çok düşük

## İlk test güvenliği
İlk test gazsız ve aktüatör güçleri güvenli/inhibe halde yapılmalıdır. Önce yalnız NRF + UART telemetri doğrulansın. Sonra tahliye ve E-STOP bench testine geçilsin.

## Doğrulama
- P64 validator: 9/9 PASS.
- Değiştirilen roket/yer istasyonu C dosyaları host-Clang syntax kontrolünden geçti.
- Bu ortamda ARM GNU/CubeIDE linker yok; gerçek final link/build doğrulaması CubeIDE `Clean + Build` ile yapılmalıdır.
