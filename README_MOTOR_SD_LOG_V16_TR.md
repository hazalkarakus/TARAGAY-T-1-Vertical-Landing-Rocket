# TARAGAY-T1 Motor SD Log V16

Bu revizyon V2 tahliye duzeltmesini korur ve sadece SD logger telemetrisini genisletir.
Motor/RCS/NRF/sensor kontrol algoritmalarinin karar veya cikis mantigi degistirilmedi.

## SD'ye kaydedilen ana motor / igne-vana verileri

- Flight-control ana vana komutu (`valve_cmd`)
- Istenen ve limitlenmis igne-vana komutu
- Gercek ADC, kapali/sifir ADC, hedef ADC ve ADC hata
- Gercek konum (tur), hedef konum (tur), hareket edilen ADC ve yuzde konum
- RPWM, LPWM, aktif PWM ve OPEN/CLOSE/STOP yonu
- Enable, zero-valid, lock, homing-active, homing-complete, motor-active ve fault
- Maks tur, ADC/tur, maksimum hareket ADC ve maksimum acik ADC
- Stall suresi, homing suresi, homing baslangic ADC
- Kontrol tick sayaci ve ADC invalid sayaci
- Donanim ADC raw12 ve ADC mV
- P83 robust feedback: ADC1, ADC2, pair diff/candidate, median7, filtered ADC,
  valid/confidence/mode/acquisition ve reject/reacquire/timeout sayaclari
- P110 adaptive actuator: start/target/current/error ADC, hiz, durma mesafesi,
  brake-entry, final ADC/final error, powered ms, state/result/direction/active PWM/abort
- P111 supervisor state/result

## Format

- Normal/preflight kayit: V14, 384 byte frame korunmustur.
- Ucus RAM kaydi: V16 compact kayit 192 byte olmustur.
- CCM toplam ayirma degismemistir: 49,152 byte.
- V16 kapasite: 256 frame. 7.5 Hz'de yaklasik 34.1 saniye.
- Ucus sirasinda SDIO/FatFS yazma yapilmama prensibi korunmustur; kayit RAM'de tutulup post-flight replay edilir.

## CSV

`sd_flight_reader_motor_v16.py` veya guncel `sd_okuma` scripti `flight.bin` dosyasindan su ciktilari uretir:

- `flight_v14_state.csv`
- `flight_v14_fast_imu.csv`
- `flight_v15_replay.csv` (eski dosya uyumlulugu)
- `flight_v16_replay.csv`
- `flight_motor.csv` (motor odakli tek CSV)
- `flight_summary.txt`

`flight_motor.csv`, motor incelemesi icin kullanilmasi onerilen dosyadir.
