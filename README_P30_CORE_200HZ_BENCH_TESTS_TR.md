# P30 – Barometre Recovery, 200 Hz ESKF ve Bench Testleri

Varsayılan CORE profilinde röle testi, dört-tur motor testi, nRF ve SD kapalıdır. Böylece önce IMU + BMP585 + LiDAR + ESKF zamanlaması tek başına doğrulanır.

## Varsayılan ayarlar (`App/Common/app_config.h`)
```c
#define APP_RELAY_SEQUENCE_TEST_MODE        0U
#define APP_NEEDLE_FOUR_TURN_TEST_MODE      0U
#define APP_OPTIONAL_NRF24_ENABLED          0U
#define APP_OPTIONAL_SDLOGGER_ENABLED       0U
```

## 1. Barometre donma düzeltmesi
- SPI polling timeout 10 ms -> 2 ms, retry -> 1.
- 3 ardışık okuma hatasında recovery istenir.
- 100 ms yeni fiziksel örnek yoksa frozen-data recovery başlar.
- Runtime recovery `HAL_Delay()` kullanmaz; STANDBY -> register restore -> NORMAL durum makinesidir.
- Gerekirse SPI2 `Abort -> DeInit -> Init` yapılır.
- Son geçerli barometre örneği 50 ms'den eskiyse üst katman `pressure_valid=0`, `healthy=0` yapar.

Live Expressions: `bmp585_recovery_active`, `bmp585_recovery_reason`, `bmp585_recovery_request_count`, `bmp585_recovery_success_count`, `bmp585_recovery_failure_count`, `bmp585_spi_abort_count`, `bmp585_spi_reinit_count`.

## 2. Gerçek 200 Hz ESKF
- Nominal IMU propagation: 1000 Hz.
- Public ESKF commit: doğrudan scheduler 5000 us correction taskı = 200 Hz.
- 15x15 covariance: 25 Hz (`decimation=40`).
- Full numerical health ve büyük Live Expressions kopyası: 50 Hz.
- 10 saniyede initialized/healthy durumda yaklaşık `public_output_count += 2000` ve `v815_eskf_correction_task_counter += 2000` beklenir.

## 3. IN1–IN4 röle testi
**Solenoidler/gaz hattı bağlı değilken çalıştır.**
```c
#define APP_RELAY_SEQUENCE_TEST_MODE        1U
#define APP_NEEDLE_FOUR_TURN_TEST_MODE      0U
```
PA0'a basınca: IN1/PB15 500 ms -> IN2/PE15 500 ms -> IN3/PE11 500 ms -> IN4/PE7 500 ms -> hepsi OFF. Test sürerken tekrar PA0 = abort. PE9 flight/fault da testi kapatır.

Live: `relay_bench_test_active`, `relay_bench_test_channel`, `relay_bench_test_last_mask`, `relay_bench_test_start_count`, `relay_bench_test_complete_count`, `relay_bench_test_abort_count`.

## 4. Ana motor / iğne vana dört-tur testi
**Basınç/gaz hattı bağlı değilken ve mekanik hareket alanı serbestken çalıştır.**
```c
#define APP_RELAY_SEQUENCE_TEST_MODE        0U
#define APP_NEEDLE_FOUR_TURN_TEST_MODE      1U
```
PA0 sırası: 1) CLOSED ZERO yakala (ADC >= 1012), 2) driver enable, 3) `1.0` komutu ile nominal 4.0 tur / 780 ADC aç, 4) açık uç doğrulanınca `v819j_test_result=7`; PA0 ile `0.0` komutu verip ZERO'ya dön.

Hareket sırasında PA0 = anlık abort. Her bacak 6 s timeout. Açık uç 780 ±30 ADC, kapanış ZERO ±20 ADC doğrulanır. Mevcut stall, ADC validity, travel limit ve closed-loop korumaları korunur.

Sonuç: `2=PASS`, `3=fault`, `4=timeout`, `5=abort`, `6=command rejected`, `7=return confirmation`, `8=travel mismatch`, `9=close mismatch`, `10=preflight inhibit`.

## 5. nRF ve SD'yi tek tek geri açma
Önce yalnız nRF:
```c
#define APP_OPTIONAL_NRF24_ENABLED          1U
#define APP_OPTIONAL_SDLOGGER_ENABLED       0U
```
200 Hz/deadline stabilse sonra SD'yi de `1U` yap. Bu sıra sorunun SPI3 mü SDIO mu olduğunu ayırır.

## Doğrulama
Paket hazırlanırken:
- `python tools/validate_v55.py` -> PASS
- `python tools/validate_p30.py` -> PASS
- Host GCC syntax-check: CORE / RELAY / NEEDLE -> PASS

Hedef kart için CubeIDE'de **Project > Clean** ardından **Build Project** yap; gerçek 200 Hz sonucu kart üzerindeki sayaçlardan doğrula.
