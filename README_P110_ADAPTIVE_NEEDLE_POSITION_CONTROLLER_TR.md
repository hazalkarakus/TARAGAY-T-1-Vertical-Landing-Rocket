# P110 Adaptive Needle Position Controller — BENCH ONLY / DO NOT FLY

P110, P106-P109'daki sabit PWM ve sabit sure karakterizasyonlarini tek tek tekrar etmek yerine, yuk altinda her hareket icin kendini sinirli olarak adapte eden kapali cevrim bir iğne-vana konum kontrolcusu ekler.

## Bu bench image ne yapiyor?

MCU resetinden yaklasik 5 s sonra, P83 feedback `valid + NORMAL` ise otomatik olarak **-50 ADC OPEN** hedefi ister. UART RX komutlari kapali kalir. Bu otomatik hareket sadece yeni adaptif state machine'i guvenli ve tekrarlanabilir sekilde bench'te komisyona almak icindir.

Ayni kod daha sonra kullanilmak uzere iki API sunar:

```c
NeedleValveIntegrationTest_RequestRelativeAdaptive(int16_t delta_adc);
NeedleValveIntegrationTest_RequestAbsoluteAdaptive(uint16_t target_adc);
```

- `delta_adc < 0`: OPEN
- `delta_adc > 0`: CLOSE
- absolute request: dogrudan ADC hedefi

## Adaptif algoritma

1. **SEARCH / breakaway tanima**
   - PWM 112'den baslar.
   - Her 12 ms'de +8 artar.
   - Maksimum 192/255 ile sinirlidir.
   - Filtreli feedback komut yonunde >=4 ADC hareket edince breakaway bulunmus kabul edilir.

2. **DRIVE / sustain**
   - Sustain PWM, bulunan breakaway PWM'den otomatik turetilir (`breakaway - 16`, alt sinir 112).
   - Yuk hareket sirasinda tekrar artarsa ve ADC hizi 20 ms boyunca cok dusuk kalirsa sustain PWM otomatik +8 artar; yine 192 ustune cikamaz.
   - Filtreli ADC hizindan `ADC/s` hareket hizi hesaplanir.

3. **Dinamik fren mesafesi**
   - Sabit `10 ADC once frenle` mantigi kullanilmaz.
   - Stop distance; o anki hiz + onceki brake/coast bilgisi ile hesaplanir.
   - Fren mesafesi 8..24 ADC arasinda sert sinirlidir.

4. **ACTIVE BRAKE + coast learning**
   - Hedefe tahmini durma mesafesi kadar kalinca drive kesilir ve dynamic brake uygulanir.
   - Brake sonrasi gercek devam hareketi (`coast`) olculur.
   - Bu deger ayni boot icindeki sonraki hareketlerde stop-distance tahminine katilir.

5. **Automatic undershoot correction**
   - 300 ms brake sonrasinda hedef +/-8 ADC icinde degilse ve hedef gecilmemisse sistem yeni bir kisa adaptif yaklasma yapabilir.
   - Maksimum 2 correction vardir.
   - Overshoot sonrasinda otomatik ters yon correction YOKTUR; guvenlik icin durur.

## Sert guvenlik sinirlari

- PWM: 112..192/255
- Hedef toleransi: +/-8 ADC
- Hard overshoot guard: 12 ADC
- Stop-distance: 8..24 ADC
- Maksimum powered sure: 260 ms / request
- Maksimum correction: 2
- Maksimum sequence: 4 s
- Feedback invalid / quarantine / sequence timeout / power timeout -> bridge OFF
- P83/P98 command-aware feedback korunur
- RCS safe tutulur

## P110 UART telemetry

Yeni alanlar:

```text
p110_state
p110_result
p110_direction
p110_start_adc
p110_target_adc
p110_current_adc
p110_error_adc
p110_active_pwm
p110_learned_breakaway_pwm
p110_sustain_pwm
p110_speed_adc_s
p110_stop_distance_adc
p110_brake_entry_adc
p110_coast_max_adc
p110_final_adc
p110_final_error_adc
p110_search_ms
p110_drive_ms
p110_brake_ms
p110_total_powered_ms
p110_correction_count
p110_breakaway_found
p110_abort_reason
```

State:
- 0 WAIT
- 1 SEARCH
- 2 DRIVE
- 3 BRAKE
- 4 CORRECTION_DWELL
- 5 DONE

Result:
- 0 RUNNING
- 1 PASS
- 2 ABORT
- 3 OVERSHOOT
- 4 NO_BREAKAWAY
- 5 POWER_TIMEOUT

## Monitor

```bash
python -u monitor_uart_p110_adaptive_needle.py --port COM21 --log uart_p110_adaptive_needle.txt
```

Beklenen saglikli bitis:

```text
p110_state = 5
p110_result = 1
p110_breakaway_found = 1
p110_final_error_adc = -8..+8
p110_abort_reason = 0
```

## Kritik not

Bu surum **ucus yazilimi degildir**. Tam vana takildiginda manuel PWM karakterizasyon zincirini tekrar etmemek icin adaptif bench kontrol mimarisini komisyona almak amaciyla hazirlanmistir. Gaz/basinc baglanmadan test edilmelidir. Mekanik sikisma, kaplin kaymasi veya sert stall gorulurse fiziksel motor gucu derhal kesilmelidir.
