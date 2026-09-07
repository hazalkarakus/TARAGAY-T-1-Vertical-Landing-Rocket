# R8R35R3R10R1 — 60 S ENDURANCE BENCH

Bu varyant yalnız uzun süreli basınçsız/inert bench doğrulaması içindir.

- Ground/preflight fiziksel SD logging: 30 Hz, R3R10 ile aynı.
- PE9 sonrası yeni SDIO komutu: 0.
- Flight RAM compact kayıt: 7.5 Hz (30 Hz TIM5 örneklerinin 1/4'ü).
- Compact frame: 96 byte.
- RAM: 512 frame = yaklaşık 68.27 s.
- Kontrol/sensör/ESKF/RCS/needle scheduler hızları değiştirilmedi.
- SD health, flight RAM modunda 7.5 Hz frame sayısına değil 30 Hz TIM5 heartbeat'ine bakar; bu yüzden düşük test-log hızını yanlış SD fault saymaz.

## 60 saniyelik test
1. Basınç kullanma; motor güç hattı bağlı olsun.
2. READY=1 bekle, PE9 ayır.
3. PE9 sonrası yaklaşık 60 s çalıştır.
4. 60 s civarında latched E-STOP ver; actuatorlar safe olduktan sonra 1 s quiet ve RAM->SD replay bekle.
5. `flush_complete=1` görmeden gücü kesme.

```powershell
python monitor_uart_p112r12r8r35r3r10r1_60s_endurance_LIVE.py --port COM21 --duration 80
```

Beklenen: flight sırasında `sd_dma_start_total` artmaz, `ram_frames` yaklaşık 7.5/s artar, 60 s sonunda yaklaşık 450 frame olur, `ram_overflow=0`, `runtime_hal_delay_violations=0`.

Not: Bu endurance profili final uçuş log çözünürlüğü kararı değildir; uzun soak testi için oluşturulmuştur.
