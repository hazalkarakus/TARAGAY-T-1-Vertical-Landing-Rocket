# P112R12R8R11 — Bounded Background INERT

Bu revizyon R8R10 fiziksel logundaki scheduler root-cause bulgusunu hedefler.

## R8R10 fiziksel bulgu
- Pure ESKF correction max: ~476 us
- ESKF task max: ~550 us
- Covariance max: ~138 us
- IMU miss: 0
- Ancak ESKF defer sayısı çok yüksek ve ESKF/BARO release miss devam ediyor.
- SD background update max: ~974 us
- TGY71 UART background max: ~741 us

Bu iki background iş mevcut admission rezervlerinden daha uzun sürdüğü için bir sonraki 1 kHz IMU release'ine taşabiliyordu. IMU geç başlayınca aynı fazdaki 200 Hz ESKF/BARO servisleri IMU guard tarafından bir veya daha fazla nesil erteleniyordu.

## R8R11 değişikliği
- ESKF matematiği, covariance kernel, scheduler priority/periyot/faz ve sensör sürücüleri değiştirilmedi.
- SD ring drain bounded:
  - normal: max 2 frame / 220 us budget
  - high: max 3 frame / 330 us budget
  - critical: max 3 frame / 420 us budget
- SD CRC16 ve ring-drain fonksiyonları GCC O2 (fonksiyon bazlı), fast-math yok.
- SD admission slack: 520 us.
- TGY71 builder/helper/CRC yolu GCC O2 (fonksiyon bazlı), fast-math yok.
- UART admission slack: 600 us.
- ESKF slot reserve 620 us + 40 us IMU guard AYNI.
- BARO reserve 360 us AYNI.
- Covariance reserve 650 us AYNI.

## Test
CubeIDE: Clean -> Build -> Flash -> Reset

```bash
py monitor_uart_p112r12r8r11_bounded_background.py --port COM21 --duration 120
```

Çıktı: `uart_p112r12r8r11_bounded_background.txt`

## Hedefler
- miss_imu = 0
- miss_eskf <= 5 (tercihen 0)
- miss_baro <= 5 (tercihen 0)
- SD update max < 520 us
- UART background max < 600 us
- pure ESKF correction < 700 us
- covariance < 650 us
- public ESKF ~200 Hz
- BARO/LiDAR/ESKF freshness dropout = 0
- actuator/RCS hareket = 0

**UÇUŞ İÇİN DEĞİLDİR. INERT bench qualification.**
