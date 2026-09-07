# P112R12R8R35R1 — DIAGNOSTIC CLOSURE

Amaç: R8R35 final dry-run kontrol mantığını DEĞİŞTİRMEDEN iki kalan anomalinin kök nedenini görünür yapmak.

## Değişmeyenler
- Hover V19.6 ana needle authority
- RCS V7.13.4 physical authority
- R8R34 quiet-upright anti-chatter
- R8R30 fixed IMU->rocket matrix
- PE9 / authorization / STOP / ESKF safety interlock'ları
- 585 ADC / 3 tur needle travel sözleşmesi
- Ground vent ve servo fiziksel izolasyonu

## R35R1'de yalnız eklenen read-only telemetry
P83 feedback: flags, ADC1/ADC2 raw, pair diff/candidate, median/filter, feedback_valid, confidence, reject/quarantine/reacquire/mode/timeout/window değerleri.

SD runtime: init/mount/file/logging, result/disk status, error/write/drop/overrun/timeout/ring/backpressure/guard, runtime reinit/recovery sayaçları ve hata nedeni.

## Test
BASINÇSIZ / DEPRESSURIZED.

```bash
py monitor_uart_p112r12r8r35r1_diagnostic_closure.py --port COM21 --duration 35
```

1. PE9 bağlı: READY=1 bekle. Needle ve RCS fiziksel çıkış 0 olmalı.
2. PE9 ayır.
3. 4-5 s dik/hareketsiz tut; RCS sessiz kalmalı.
4. Sonra 4-6° kontrollü eğ; RCS cevap vermeli.
5. TXT dosyasını inceleme için gönder.

Bu revizyon uçuşa serbest bırakma değildir. Diagnostik kapanış adaydır.
