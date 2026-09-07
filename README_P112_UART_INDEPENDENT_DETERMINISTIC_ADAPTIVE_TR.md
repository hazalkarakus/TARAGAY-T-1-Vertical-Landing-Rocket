# P112 UART-Independent Deterministic Adaptive Needle Controller

BENCH ONLY - DO NOT FLY - NO GAS / NO PRESSURE.

Amaç: P111 adaptif OPEN/CLOSE öğrenmesini ana döngü, UART formatting ve SD servis zamanlamasından ayırmak.

## P112 değişiklikleri
- TIM7 1 kHz ISR, P110/P111 state machine kararlarını deterministik çalıştırır.
- P83 dual-ADC robust feedback TIM7 içinde 500 Hz örneklenir.
- Motor SEARCH/DRIVE sırasında 260 ms hard powered-time deadline ISR tarafından zorlanır.
- Timing-critical SEARCH/DRIVE/BRAKE sırasında UART frame formatting ve SD background update ertelenir; dwell/DONE sırasında telemetry devam eder.
- İlk coast tahmini 20 ADC konservatif başlar. Öğrenilen coast yukarı hızlı, aşağı yavaş adapte olur.
- Stop-distance üst sınırı 48 ADC'ye çıkarıldı.
- Hedefe yaklaşırken sustain PWM'den 24 count düşürülmüş bounded decel bölgesi uygulanır; minimum 96/255.
- P111'in 5 OPEN/CLOSE çevrimi korunur.

## Test sırası
1. Mevcut kısmi mekanik bağlantı, gaz/basınç yok.
2. Python monitorü önce aç, sonra RESET: `python -u monitor_uart_p112_deterministic_adaptive.py --port COM21 --log uart_p112_uart_open.txt`
3. Ayrı bir RESET'te monitor kapalıyken 5 çevrimi fiziksel olarak tamamlat. Test bittikten sonra monitorü açıp final snapshot'ı `uart_p112_uart_closed_then_attach.txt` olarak al.
4. İki koşuda da 5/5 PASS, hard_off_count=0 ve benzer öğrenilmiş OPEN/CLOSE değerleri beklenir.

P112 geçmeden iğne vanayı tam mekanik bağlantıya alma. P112 iki koşulda da geçerse bir sonraki aşama P113 FULL-VALVE COMMISSIONING'dir.
