# R8R35R3R10 — RAM-FIRST FLIGHT LOGGER

## Amaç
R3R9 donanım testinde 30 Hz ve gerçek 750 kHz SDIO uygulanmasına rağmen, actuator quiet-window bittikten sonraki ilk SD komutunda yine COM_CRC_FAILED -> CMD_RSP_TIMEOUT görüldü. R3R10 bu problemi flight-control döngüsünden tamamen ayırır.

## Yeni çalışma kuralı
- Ground / preflight: `flight.bin` normal V14 384-byte frame ile 30 Hz kaydolur.
- PE9 flight-active latch olur olmaz: **yeni SDIO komutu başlatılmaz.**
- Flight: 30 Hz compact record yalnız CCM RAM'e yazılır.
- Compact frame: 96 byte.
- Aynı mevcut 49,152-byte CCM alanı overlay edilir; ek CCM RAM kullanılmaz.
- Kapasite: `512 frame / 30 Hz = 17.07 s`.
- Touchdown veya latched E-STOP + actuatorların 1 s quiet olması: compact RAM kayıtları `flight.bin` içine V15 replay frame olarak yazılır.
- Postflight replay tamamlandıktan sonra flight-session RAM mode latched kalır; aynı PE9 oturumunda tekrar SD yazımı açılmaz.

## SD tarafı
- Capture: 30 Hz.
- SDIO quiet-side clock: `ClockDiv=10` => yaklaşık 4 MHz.
- Writer buffer: 1536 byte = 4 x 384-byte frame = 3 sector.
- Bunun nedeni PE9'dan hemen önce başlamış olabilecek preflight DMA'nın uçuşa sarkma süresini küçültmektir.

## Uçuşta değişmeyenler
TaragayFlightLogic, Full-State ESKF, needle autonomous/low-level controller, RCS/SolenoidOutput ve scheduler kontrol otoriteleri değiştirilmedi.

## İlk bench testi — basınçsız / inert
1. CubeIDE ile Clean + Build + Flash.
2. SD kart takılı, needle motor güç hattı bağlı olsun; basınç kullanma.
3. UART monitor:

```powershell
python monitor_uart_p112r12r8r35r3r10_ram_first_flight_logger_LIVE.py --port COM21 --duration 35
```

4. `READY=1` bekle.
5. PE9 ayır.
6. Needle/RCS hareket etsin.

### Beklenen canlı sonuç
- `RAM=1`
- `frames` yaklaşık 30/s artar.
- `NEW SD DMA starts during flight = 0`
- `sd_r10_physical_sd_faults = 0`
- `sd_r10_ram_overflow = 0`
- `runtime_hal_delay_violations = 0`
- `div=10`

`tail_dma_at_entry=1` olabilir. Bu yalnız PE9'dan önce başlamış bir DMA'nın hâlâ tamamlanıyor olduğunu gösterir; yeni flight DMA start değildir.

## Postflight replay'i bench'te doğrulama
Normal bench koşullarında gerçek touchdown oluşmayabilir. Replay yolunu ayrıca test etmek için, PE9 sonrası birkaç saniye RAM capture aldıktan sonra **latched E-STOP** ver. Kontrol katmanı actuatorları safe yaptıktan ve 1 s quiet geçtikten sonra:

- `capture_frozen=1`
- `postflight_reason=2`
- `flush_active=1` görülebilir
- en sonunda `flush_complete=1`

Sonra karttaki `flight.bin` dosyasını PC'ye al ve:

```powershell
python decode_r3r10_flight_bin.py flight.bin
```

V15 + `0xF10A` marker frame'leri RAM flight replay kayıtlarıdır.

## Gerçek touchdown
TaragayFlightLogic `TARAGAY_MISSION_TOUCHDOWN` durumuna ulaştığında aynı süreç otomatik olarak `postflight_reason=1` ile çalışır.

## Önemli sınır
RAM flight kapasitesi yaklaşık 17.07 saniyedir. Bu süre aşılırsa `sd_r10_ram_overflow` artar ve capture freeze olur. Bu durumda uçuş kontrolü devam eder fakat log kapasitesi dolmuştur.
