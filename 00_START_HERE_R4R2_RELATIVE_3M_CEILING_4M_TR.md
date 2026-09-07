# TARAGAY-T1 R4R2 — PE9 SONRASI +3 m HEDEF / +4 m ANA İTKİ TAVANI

Bu sürüm R4R1 son test kodundan türetilmiştir.

## Uçuş geometrisi

1. PE9 ayrılır ve mevcut `flight_active` zinciri görev kontrolünü başlatır.
2. İlk geçerli ESKF/100 Hz kontrol örneğinin CG yüksekliği `z_sep` olarak dondurulur.
3. Ascent/hover hedefi `z_sep + 3.00 m` olur.
4. `z >= z_sep + 4.00 m` olursa ana itki komutu zorla `0` yapılır (`target_force_n=0`, `valve_cmd=0`).
5. Tavan guard aktifken RCS kontrolü aynen devam eder.
6. Roket `z <= z_sep + 3.80 m` seviyesine geri indiğinde tavan guard bırakılır ve normal descent/landing kontrolü ana itkiyi tekrar kullanabilir.

Örnek: PE9 sonrası ilk geçerli yükseklik 1.00 m ise hedef 4.00 m, tavan 5.00 m, tavan release 4.80 m'dir.

## Korunan yollar

- Hover V19.6 kazançları ve genel state machine
- RCS V7.13.4 ve fiziksel R8R33 handoff
- R8R32 needle physical handoff ve P111/P110/P112 actuator katmanı
- Full-State ESKF / fixed IMU->rocket transform
- PE9 authorization, STOP/E-STOP, SystemMonitor ve freshness gate'leri
- SD RAM-first/postflight replay ve nRF TDD

## Doğrulama

Host GCC ile `taragay_flight_logic.c` object compile PASS (yalnız CMSIS host pointer-size ve dormant legacy-function warningleri).
Host davranış testi:

- separation=1.00 m -> target=4.000 m: PASS
- z=5.00 m (+4.00 m) -> target_force=0, valve=0: PASS
- z=4.90 m -> ceiling hysteresis hâlâ aktif, valve=0: PASS
- z=4.79 m -> guard release, normal controller tekrar force üretir: PASS

CubeIDE/ARM target build bu ortamda yapılmadı; karta atmadan önce Clean + Build ile 0 error doğrulaması gerekir.
