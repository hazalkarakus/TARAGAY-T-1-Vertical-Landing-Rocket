# P108 erken fren + coast karakterizasyonu

Bu test yalnız bench içindir. Gaz/basınç bağlama.

1. Kaplin/mil hizasını ve mekanik sıkışma olmadığını gözle kontrol et.
2. P107 sonrası mekanizma hâlâ açık tarafta olabilir; P108 başlangıcı 850..1023 ADC aralığında kabul eder. Sert mekanik stopa yakın olduğunu düşünüyorsan testi çalıştırma.
3. Fiziksel motor güç kesicisi erişilebilir olsun; mekanizmadan elini çek.
4. CubeIDE: **Clean -> Build -> Flash**.
5. UART:
   `python -u monitor_uart_p108_prebrake_coast_50adc.py --port COM21 --log uart_p108_prebrake_coast_50adc.txt`
6. Reset sonrası yaklaşık 5 s bekle.
7. P108: PWM144/20 ms boost -> PWM128 sustain. Hedef toplam 50 ADC'dir fakat drive yaklaşık 30 ADC OPEN'da erken kesilir.
8. Active brake başladıktan sonra 100, 250 ve 500 ms konumları otomatik kaydedilir.
9. 500 ms sonunda bridge tamamen kapanır. Otomatik geri dönüş YOKTUR.

Ana telemetri:
- p108_start_adc / p108_target_adc / p108_prebrake_adc
- p108_brake_entry_adc
- p108_adc_100ms / p108_adc_250ms / p108_adc_500ms
- p108_coast_100_adc / p108_coast_250_adc / p108_coast_500_adc
- p108_final_error_adc

Coast işareti:
`coast = brake_entry_adc - sample_adc`
Pozitif değer OPEN yönünde fren sonrası devam hareketidir.
