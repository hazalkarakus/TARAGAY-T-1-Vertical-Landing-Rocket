# R8R35R3R9 — SDIO 750 kHz + Actuator Quiet Window

Bu revizyon R3R8 donanım logundaki iki somut problemi hedefler:

1. **R3R8 ClockDiv=30 gerçekte runtime'da korunmuyordu.** `Core/Src/sdio.c` değeri BSP host reset/init sırasında `ClockDiv=10` ile eziliyordu. R3R9'da iki yol da ortak `APP_SDIO_RUNTIME_CLOCK_DIV=62` kullanır. STM32F4 SDIOCLK=48 MHz kabulüyle SDIO_CK = 48 MHz / (62+2) = **750 kHz**.
2. **R3R8 flight-entry hold yanlışlıkla kısalabiliyordu.** PE9 anındaki motor-active yolu 2500 ms deadline'ı 500 ms ile overwrite edebiliyordu. R3R9 deadline'ı yalnız ileri uzatan MAX-deadline mantığına çevrildi.

Ek olarak, fiziksel/istenen RCS aktivitesinde ve tamamlanan her RCS pulse sonrasında yeni SDIO command launch **350 ms** bekletilir. 30 Hz RAM capture devam eder; control/ESKF/RCS/needle beklemez.

## Değişmeyen uçuş kontrol kaynakları

- TaragayFlightLogic: değişmedi
- Needle autonomous controller: değişmedi
- Needle low-level controller: değişmedi
- SolenoidOutput/RCS output: değişmedi
- Full-State ESKF: değişmedi
- Scheduler: değişmedi
- `App/app.c`: değişmedi
- `app_tasks.c`: değişmedi

## Yeni TGY73 son alanlar

Toplam alan: **329**

- `sd_hold_active_r9`
- `sd_hold_remaining_ms_r9`
- `sd_rcs_hold_events_r9`
- `sd_sdio_clkcr_r9`

`sd_sdio_clkcr_r9 & 0xFF` değeri **62** olmalı.

## Basınçsız / inert test

Needle motor güç hattı bağlı kalsın. READY=1 görüldükten sonra PE9 ayır.

```powershell
python monitor_uart_p112r12r8r35r3r9_sdio_750khz_actuator_quiet_LIVE.py --port COM21 --duration 35
```

Beklenen kapanış kriterleri:

- `SD final ready/log = 1/1`
- `sd_r7_fail_count = 0`
- `sd_dma_start_errors_r4 = 0`
- `write_errors = 0`
- `drop = 0`, `overrun = 0`
- capture yaklaşık 30 Hz
- `CLKDIV = 62` (~750 kHz)
- `runtime_hal_delay_violations = 0`

## Önemli

RAM buffering sınırsız değildir. 30 Hz/384 B ile mevcut 128-frame ring + 4x9216 B writer buffer toplamı kabaca birkaç saniyelik actuator-noise penceresini taşır; uzun süre hiç SD write yapılamaması hâlâ overflow yaratabilir. Bu nedenle bu revizyon donanım testi geçmeden uçuşa hazır kabul edilmez.
