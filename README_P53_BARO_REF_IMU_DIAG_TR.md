# TARAGAY-T1 P53 — Baro Reference Tracking + IMU Pattern Diagnostics

P53, P52'nin uzun-soak covariance/SD/scheduler davranisini korur. Bu surum iki hedefe odaklanir:

1. Preflight beklemesinde MS5611 sicaklik/ortam driftinin barometrik sifiri metrelerce kaydirmasini engellemek.
2. IMU repeated-word recovery olayinda uc ham redundant SPI burstunu ve karantinaya alinmis corrected tripleti yakalamak.

## Barometre referans politikasi

Ilk 400 kabul edilmis sample (~2 s) ile klasik ground-pressure kalibrasyonu aynen kalir. Sonrasinda ground-pressure datum sadece su kosullarda yavasca takip edilir:

- flight_active = 0
- Full-State ESKF initialized + healthy
- stationary_detected = 1
- |vz| <= 0.15 m/s
- LIDAR valid ve sample age <= 50 ms

Takip IIR alpha=0.0025 ve sample basina maksimum 0.05 Pa adimla yapilir. Referans hareketi barometrik vertical-speed olarak yorumlanmaz. Full-State ESKF barometrik reference da tracking sirasinda ayni datum ile senkron tutulur; boylece sensor sifiri duzelirken ESKF eski pre-warmup baro originini cikarmaya devam etmez.

PE9 confirmed/debounced separation veya flight_active goruldugu anda `Barometer_FreezeGroundReference()` tek yonlu latch olur. Ayni boot boyunca datum tekrar takip edilemez. Bu, ucusta gercek basinc/yukseklik degisimini korur.

## IMU pattern diagnostigi

Repeated-word pattern gorulurse P45/P51 recovery davranisi degismez. P53 sadece son olayin diagnostigini saklar:

- event count + timestamp
- repeated magnitude + sign mask
- burst0/1/2: GX,GY,GZ,AX,AY,AZ raw
- corrected: GX,GY,GZ,AX,AY,AZ raw

Bu alanlar bench UART'a eklenmistir; actuator authorization veya flight logic bunlara bagli degildir.

## UART

- frame: `$TGY64`
- banner: `TGY UART FLIGHT DIAGNOSTICS V64`
- firmware: `8.19M-P53-BARO-REF-TRACK-IMU-PATTERN-DIAG`
- 297 field + CRC16

## SD

V14 / 384-byte binary formati degismedi. P52 decoder mantigi byte-format olarak aynidir.

## Ilk kabul testi

1. Roket sabit, PE9 flight-active yapilmayacak. 10 dakika beklet.
2. `baro_ref_track_active=1` ESKF stationary olduktan sonra gorulmeli.
3. Sicaklik artarken `baro_alt_mm` sifir cevresinde kalmali. Hedef ±0.10-0.20 m.
4. `eskf_cov_faults=0`, `sd_dropped=0`, `miss_imu=0`.
5. IMU pattern olayi olursa `imu_pat_event_count` artmali ve 3 burst raw alanlari dolmali.
6. Sonra flight_active yap: `baro_ref_frozen=1`, `baro_ref_track_active=0`.
7. Roketi kontrollu +0.5 / +1.0 m kaldir: ground pressure sabit kalmali, baro gercek delta-yuksekligi vermeli.

Not: UART gercek ucusta fiziksel olarak bagli olmak zorunda degildir; P53 UART diagnostigi sadece bench analizi icindir.
