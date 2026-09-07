# P112R12R8R35R2 — Closure Fixes

Bu revizyon R8R35R1 diagnostik logunda kanitlanan iki kapanis problemini hedefler.

## Degisenler

1. **SD servis onceligi**
   - `SDLogger_Update()` artik freshness/preflight'ten sonra, nRF/remote servisinden once calisir.
   - 1 kHz hard scheduler otoritesi ve mevcut IMU slack gate aynen korunur.
   - Amaç R8R35R1'de 127/127 dolan capture ring ve binlerce overrun/drop olayini ortadan kaldirmaktir.

2. **Needle same-target anti-overshoot guard**
   - 12 ADC hard overshoot fault esigi GEVSETILMEDI.
   - Gercek GNC target degisimi yine aninda P110'a gider.
   - Ayni target HOLD durumunda yeni breakaway ancak son basarili stroktan ogrenilen stop mesafesi + 4 ADC'yi asan hata 200 ms surerse baslar.
   - Gate 20..48 ADC arasinda sinirlidir. R8R35R1 logunda ogrenilen stop mesafesi 32 ADC oldugu icin pratik HOLD gate ~36 ADC olur.

3. **P83 HOLD-only transient recovery**
   - Needle hareket ederken feedback kaybi halen aninda P110 fail-safe/abort'tur.
   - Needle zaten HOLD ise kisa P83 quarantine sirasinda motor hareket etmez; en fazla 500 ms toparlanma penceresi verilir.
   - 500 ms sonunda feedback hala gecersizse ayni FEEDBACK fault latched olur.

## Test

Basinçsiz / depressurized full-system dry-run:

```bash
py monitor_uart_p112r12r8r35r2_closure_fixes.py --port COM21 --duration 35
```

Beklenen kritik sonuclar:
- preflight needle/RCS fiziksel cikis = 0
- PE9 sonrasi needle target travel <= 585 ADC
- needle fault = 0
- RCS pulse ~40 ms
- SD drop/overrun DELTA = 0
- SD ready/logging flight boyunca 1
- scheduler miss/realign DELTA = 0
- P83 kisa quarantine olabilir; final valid/mode NORMAL olmali ve invalid sure <=500 ms

Bu revizyon hala basinçsiz dry-run adayidir; basincli test/atis qualification degildir.
