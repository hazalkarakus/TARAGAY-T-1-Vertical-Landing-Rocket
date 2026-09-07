# P109 - 50 ADC Loaded Pre-Brake / Coast Characterization (120 ms sustain)

**BENCH ONLY - DO NOT FLY - GAZ/BASINC YOK.**

Amaç: P108'de 60 ms sustain süresi pre-brake eşiğine ulaşmadan doldu. P109 yalnız sustain süresini 120 ms'ye çıkarır; PWM seviyeleri artırılmaz. Hedef, gerçek pre-brake eşiğine ulaştıktan sonra aktif fren sonrası 100/250/500 ms coast değerlerini ölçmektir.

## Sabitler

- START kabul: 850..1023 ADC
- Hedef: START - 50 ADC
- Pre-brake: TARGET + 20 ADC = START - 30 ADC
- Boost: PWM 144/255 (~56.5%), tam 20 ms
- Sustain: PWM 128/255 (~50.2%), en fazla 120 ms
- Toplam powered OPEN: en fazla 140 ms
- Active brake: 500 ms
- Coast örnekleri: brake entry, +100 ms, +250 ms, +500 ms
- Otomatik geri dönüş: YOK
- Hard powered overshoot guard: TARGET - 12 ADC altı
- Sequence timeout: 3000 ms

## Beklenen başarılı akış

RESET -> 5 s -> P83 valid/NORMAL -> BOOST 20 ms -> SUSTAIN -> START-30 ADC pre-brake -> active brake -> 100/250/500 ms örnekleri -> bridge OFF -> CHARACTERIZED.

Başarılı karakterizasyon için `p109_result=1` ve `p109_brake_cause=1` beklenir. `p109_result=2` sustain bütçesinde pre-brake'e ulaşılamadığını gösterir. `p109_result=4` overshoot guard'dır.

Monitor:

```bash
python -u monitor_uart_p109_prebrake_coast_120ms_50adc.py --port COM21 --log uart_p109_prebrake_coast_120ms_50adc.txt
```

Mekanik sert sıkışma, kaplin kayması veya motor stall sesi olursa testi beklemeden motor beslemesini kesin.
