# P112R12R8R18 — GNC -> P111 -> P110/P112 gerçek motor bench testi (INERT)

Bu rev yalnız BASINÇSIZ / GAZSIZ bench doğrulaması içindir. Uçuş build'i değildir.

Amaç: R8R17'de elle vana açılırken görülen LOW_LEVEL/STALL olayını normal GNC motor yolu ile karıştırmadan doğrulamak.

Değişiklikler R8R17'ye göre:
- `APP_P112R10R3_GNC_MOTOR_BENCH_MODE = 1`
- sanal GNC girişleri mevcut üretim 200 Hz vertical outer-loop matematiğini kullanır.
- ilk test komut cap'i `%15` (`0.15`) ile sınırlandırıldı.
- PE9 ayrıldıktan sonra 1 s CLOSED hold, 3 s aktif stimulus, sonra command=0 CLOSED.
- RCS/vent fiziksel çıktıları INERT izolasyon nedeniyle OFF kalır.
- UART RX komutları kapalıdır.
- E-STOP emergency close mantığı aynen aktiftir.
- ESKF/sensör/scheduler/P110/P111 algoritmaları değiştirilmemiştir.

## Test
1. Gaz/basınç yok.
2. Vana CLOSED; motor/redüktör bağlı; eller hareketli parçalardan uzak.
3. Ground station açık, Switch-1 ve Switch-2 OFF.
4. Rocket açılır; UART'ta `ready=1`, `needle_adc≈closed_ref`, `fault=0` beklenir.
5. PE9'u ayır. Bundan sonra vanaya ELLE DOKUNMA.
6. Yaklaşık 1 s CLOSED hold.
7. Sonraki ~3 s GNC komutu en fazla 0.15 olur; hedef CLOSED referanstan yaklaşık 117 ADC aşağıya gider ve motor OPEN yönüne hareket eder.
8. Stimulus bitince command=0; motor CLOSED referansa geri dönmelidir.
9. Beklenen final: ADC≈closed_ref ±12, `p111_fault=0`, `needle_fault=0`, `p110_pwm=0`.
10. İstersen aktif 3 s penceresinde Switch-2 E-STOP uygulanabilir; o durumda normal GNC iptal olup emergency close CLOSED'a dönmelidir.

Monitor:
`py monitor_uart_p112r12r8r18_gnc_motor_bench.py --port COM21 --duration 45`
