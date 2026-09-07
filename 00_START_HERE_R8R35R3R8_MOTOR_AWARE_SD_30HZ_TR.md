# R8R35R3R8 — Motor-Aware SD 30 Hz

Amaç: powered needle motor yolu aktifken gözlenen SDIO CMD CRC/timeout arızasını, kontrol döngüsünü bloklamadan SD yazma zamanlamasını motor aktivitesinden ayırarak containment etmek.

## Değişiklikler
- TIM5 SD capture: 200 Hz -> ~30 Hz (33.333 ms).
- SDIO runtime clock divider: 10 -> 30 (~4 MHz -> ~1.5 MHz) for additional command/data noise margin.
- PE9/flight-active yükselen kenarında yeni SDIO write launch 2.5 s tutulur.
- P110 SEARCH/DRIVE/BRAKE/CORRECTION_DWELL veya LPWM/RPWM aktifken yeni SDIO write launch yapılmaz.
- Motor aktivitesi bittikten sonra 500 ms cooldown uygulanır.
- Capture durmaz: frame'ler RAM ring + writer buffer'larda birikir.
- Aktif DMA varsa completion servisi devam eder; abort/re-init yapılmaz.
- R3R6 bounded start retry ve R3R7 düşük seviye diagnostik korunur.
- Runtime HAL_Delay guard korunur.
- Kontrol/ESKF/RCS/needle authority değiştirilmedi.

## Telemetri
Mevcut `sd_suppressed_total` alanı motor-aware write-hold nedeniyle ertelenen write-launch girişimlerini de sayar. Yeni TGY73 alanı eklenmedi; field count değişmez.

## Test
Basınçsız/inert, needle motor güç hattı bağlı. READY sonrası PE9 ayır. 35 s çalıştır.
Beklenen: `SD final ready/log=1/1`, write/drop/overrun=0, R7 fail=0, runtime HAL_Delay=0. `sd_suppressed_total` > 0 olması beklenebilir.

Not: ARM/CubeIDE target build bu paket hazırlanırken çalıştırılmadı; CubeIDE Clean + Build ile doğrula.
