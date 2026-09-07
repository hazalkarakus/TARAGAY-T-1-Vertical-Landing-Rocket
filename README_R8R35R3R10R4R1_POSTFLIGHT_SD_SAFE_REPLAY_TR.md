# TARAGAY-T1 R8R35R3R10R4R1 — nRF Stable TDD + Postflight SD Safe Replay

## Taban
Bu revizyon doğrudan **R8R35R3R10R4 nRF Stable TDD** tabanıdır. R4 ile gelen explicit-TDD / tek FAST cevap davranışı değiştirilmemiştir. Overshoot, P110/P111, TaragayFlightLogic, ESKF ve RCS kontrol otoritesine dokunulmamıştır.

## R4R1'de düzeltilen SD akışı
E-STOP sonrası SD replay artık yalnız şu sırayla başlayabilir:

1. `STOP` latch olur.
2. Needle one-shot E-STOP safe-close tamamlanır.
3. `needle_lpwm=0`, `needle_rpwm=0`, RCS requested/applied mask = 0 doğrulanır.
4. **Tam bu anda yeni quiet timer başlar.**
5. 3000 ms boyunca hiçbir gerçek motor/RCS aktivitesi olmaz; olursa timer sıfırlanır.
6. SDIO/DMA/HAL command path `BSP_SD_RuntimeSoftRecover()` ile temizlenir. Bu adım kart re-enumeration yapmaz ve `HAL_Delay()` kullanmaz.
7. RAM flight frame'leri SD'ye replay edilir.
8. Replay tamamlanınca `sd_r10_flush_complete=1` olur.

E-STOP close FAILED ise replay başlamaz. RAM görüntüsü korunur.

## Neden full runtime re-init değil?
Projede runtime `HAL_Delay()` guard aktiftir. `BSP_SD_RuntimeReinit()` -> `BSP_SD_Init()` yolu kart reset beklemeleri için `HAL_Delay()` içerir. Bunu postflight sırasında açmak nRF/scheduler döngüsünü gereksiz yere bloklayabilirdi. R4R1 bu nedenle motor EMI sonrası ihtiyaç duyulan **host/DMA command-path temizliğini** non-blocking soft-recovery ile yapar ve mevcut no-runtime-HAL_Delay kuralını korur.

## Önemli davranış
R3R10'da PE9 sınırına sarkan eski bir DMA hata verdiyse `sd_r10_physical_sd_faults` artmış olabilir. R4'te bu sayaç replay'i tamamen engelliyordu. R4R1'de bu tarihsel tail-fault, 3 s quiet + command-path recovery sonrasında ilk replay denemesini engellemez. Recovery SONRASI yeni bir SD write fault oluşursa replay durdurulur ve hata görünür kalır.

## Bench testi — basınçsız / inert
```powershell
python monitor_uart_p112r12r8r35r3r10r4r1_postflight_sd_safe_replay_LIVE.py --port COM21 --duration 85
```

Beklenenler:
- Flight boyunca `NEW SD DMA starts during flight = 0`.
- `nrf_link=1`, accepted command rate tercihen ~20 Hz; R4 davranışı korunur.
- STOP sonrası `estop_close_complete=1`, `estop_close_failed=0`.
- İlk `flush_active=1` gözlemi, ilk `estop_close_complete=1` gözleminden **en az ~3000 ms sonra** olmalıdır (UART örnekleme kuantizasyonu nedeniyle küçük pozitif fark beklenir).
- `sd_r10_flush_complete=1`.
- `runtime_hal_delay_violations=0` kalır.
- RAM overflow=0.

## Uçuş notu
Bu paket kaynak düzeyinde hazırlanmış ve host/static kontrollerden geçirilmiştir; CubeIDE ARM target build + basınçsız inert donanım testi yapılmadan flight-qualified kabul edilmemelidir.
