# P111 — 5 Çevrim Adaptif İğne Vana Öğrenme Testi

**BENCH ONLY / DO NOT FLY / GAZ VE BASINÇ YOK**

P111, P110 adaptif konum kontrolcüsünü değiştirmeden onun üstüne bir commissioning supervisor ekler. Amaç aynı firmware ile OPEN ve CLOSE yönlerinde breakaway PWM, sustain davranışı, hız, dinamik fren mesafesi ve coast değerlerinin çevrimden çevrime öğrenilip öğrenilmediğini görmek.

## Otomatik test akışı

Resetten sonra 5 saniye beklenir ve P83 feedback `VALID + NORMAL` olunca o anki konum `baseline_adc` olarak kaydedilir.

Her çevrim:

1. `OPEN target = baseline - 50 ADC`
2. P110 adaptif kontrolcü breakaway PWM'i online arar.
3. Hareket başlayınca sustain PWM otomatik seçilir.
4. ADC hızı ölçülür ve dinamik stop-distance hesaplanır.
5. Active brake uygulanır, coast öğrenilir.
6. OPEN PASS ise bridge tamamen kapatılır ve 1 s beklenir.
7. `CLOSE target = baseline`
8. CLOSE yönünde aynı adaptif öğrenme yapılır.
9. CLOSE PASS ise çevrim PASS sayılır, bridge kapalı 1.2 s beklenir.
10. Toplam 5 çevrim / 10 hareket tamamlanır.

Bir hareket P110 tarafından PASS edilmezse P111 kalan çevrimleri **iptal eder**. Yani bir fault sonrası otomatik tekrar zorlaması yoktur.

## Güvenlik limitleri

P110 limitleri aynen korunur:

- Breakaway search: `PWM 112..192 / 255`
- Search step: `+8 PWM`, 12 ms adımlar
- Maksimum powered süre: `260 ms / hareket`
- Target toleransı: `±8 ADC`
- Hard overshoot guard: `12 ADC`
- Dynamic stop distance: `8..24 ADC`
- Active brake: `300 ms`
- Maksimum 2 undershoot correction
- Tek hareket sequence timeout: `4 s`

P111 supervisor:

- 5 tam OPEN/CLOSE çevrimi
- OPEN/CLOSE arası bridge-off dwell: `1000 ms`
- Çevrimler arası bridge-off dwell: `1200 ms`
- Supervisor toplam timeout: `25 s`
- Bir hareket FAIL olursa kalan test iptal

## Önemli

Bu testte vana/aktüatör mekanizması bağlı olabilir fakat **gaz veya basınç bağlı olmamalı**. Kaplin kayması, belirgin hizasızlık, sert mekanik stop veya motorun hard-stall sesi görülürse yazılımın bitmesini beklemeden motor beslemesini kesin.

## UART monitor

```bash
python -u monitor_uart_p111_5cycle_adaptive_learning.py --port COM21 --log uart_p111_5cycle_adaptive_learning.txt
```

Başarılı final beklenen ana alanlar:

```text
p111_state      = 5   DONE
p111_result     = 1   PASS
p111_completed  = 5
p111_moves_completed = 10
p111_pass_mask  = 31  (0x1F)
p111_abort_reason = 0
```

Öğrenme için özellikle şu alanları takip edin:

```text
p111_learned_open_pwm
p111_learned_close_pwm
p111_learned_open_coast
p111_learned_close_coast

p111_last_open_breakaway_pwm
p111_last_close_breakaway_pwm
p111_last_open_stop_adc
p111_last_close_stop_adc
p111_last_open_coast_adc
p111_last_close_coast_adc
p111_last_open_error_adc
p111_last_close_error_adc
```

Ayrıca P110'un canlı alanları her hareketin içinde mevcut kalır: `p110_speed_adc_s`, `p110_stop_distance_adc`, `p110_active_pwm`, `p110_sustain_pwm`, `p110_correction_count`, `p110_total_powered_ms`.

## Firmware banner

```text
8.19M-P111-5CYCLE-ADAPTIVE-LEARNING
```
