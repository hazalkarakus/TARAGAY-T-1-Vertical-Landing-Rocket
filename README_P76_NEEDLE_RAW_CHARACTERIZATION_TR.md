# P76 — Needle Raw Pulse Characterization (DO NOT FLY)

Amaç: P74'te görülen aşırı salınımın ardından motor + 1:10 redüktör + vana + potansiyometre zincirinin gerçek açık çevrim tepkisini ölçmek. P76 kapalı çevrim pozisyon kontrolü kullanmaz.

## İzolasyon
- P71 ana baseline ayrı kalır.
- NRF compile-time kapalıdır.
- UART RX komutları kapalıdır; USART2 sadece telemetri gönderir.
- RCS `ForceSafe` tutulur. Testte RCS güç hattını fiziksel olarak inhibit et.
- Gaz/basınç BAĞLAMA.

## P76 test seti
Mavi USER (PA0) butonu her basışta yalnız bir adım ilerletir:

1. Referans ADC'yi yakala — hareket yok.
2. OPEN: PWM 35, tam 10 ms.
3. CLOSE: PWM 35, tam 10 ms.
4. OPEN: PWM 45, tam 10 ms.
5. CLOSE: PWM 45, tam 10 ms.
6. OPEN: PWM 55, tam 10 ms.
7. CLOSE: PWM 55, tam 10 ms.

Her pulse sonunda TIM7 1 kHz ISR motoru tam sürede brake eder. Enable 500 ms boyunca açık / PWM=0 tutularak dinamik fren sonrası sürüklenme ölçülür, sonra H-köprü disable edilir.

Her adım için kaydedilir:
- start ADC
- pulse sonu ADC
- +100 ms ADC
- +250 ms ADC
- +500 ms ADC
- signed hareket (`start - ADC500`; OPEN pozitif, CLOSE negatif)
- referanstan maksimum sapma

## Güvenlik
- Referans yakalanırken ADC >= 1012 şart.
- Referanstan >120 ADC sapma olursa hard abort.
- Flight active / preflight fault / actuator fault / system_ok=0 durumunda pulse başlamaz; aktif pulse varsa abort.
- Pulse veya 500 ms settle sırasında USER butonuna basmak local abort + driver disable yapar.
- OPEN ters yönde >=3 ADC veya CLOSE ters yönde >=3 ADC giderse test abort olur.

## UART

```bat
python -u monitor_uart_p76_characterization.py --port COM21 --log uart_p76_characterization.txt
```

İlk testte yalnız **1. basış (referans) + 2. basış (PWM35 OPEN)** yapıp sonucu kontrol etmek en güvenli yoldur. 500 ms settle bitince monitör `NEXT` durumuna döner. Sonucu paylaşmadan daha yüksek PWM adımlarına geçmek zorunlu değildir.
