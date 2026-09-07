# TARAGAY-T1 P34 — Deterministik 1 kHz IMU / 200 Hz ESKF / SD

P34, P33 UART kaydinda gorulen yaklasik 58/s IMU deadline miss ve scheduler realign davranisini hedefler.

## Korunan frekanslar
- IMU nominal propagation: 1000 Hz
- Full-State ESKF public correction/output: 200 Hz
- BMP585: 200 Hz scheduler service
- LIDAR: 200 Hz scheduler service
- nRF monitor: 200 Hz
- SD capture/state frame: 200 Hz
- ESKF 15x15 covariance propagation: 25 Hz

## P34 degisiklikleri
1. Scheduler App_Run icinde ilk servis edilen katmandir. Background servis gruplarinin arasinda Scheduler_Run tekrar cagrilir.
2. 5 ms yavas gorevler 1 ms slotlara dagitildi:
   - slot 0: covariance/background icin bos
   - ESKF: slot 1
   - BARO: slot 2
   - LIDAR: slot 3
   - NRF + monitor: slot 4 (monitor 10 Hz)
3. Yavas cooperative task, bir sonraki IMU release'ine yeterli sure yoksa pending kalir; gecmis periyotlari kovalamaz.
4. IMU software-SPI Mode-3, 3-burst redundant validation korunur. GPIO edge delay 12 NOP -> 8 NOP ve her burst sonundaki gereksiz Live-Debug kopyasi kaldirildi.
5. 25 Hz 15x15 covariance propagation 200 Hz ESKF correction task'indan ayrildi. 40 ms accumulator doldugunda bos slot-0 slack penceresinde calisir; bir sonraki 200 Hz ESKF correction bundan sonra gelir. Boylece covariance-predict -> measurement-correct sirasi korunurken ayni 1 ms slotta iki agir is ust uste binmez.
6. SD ring drain bir main-loop gecisinde en fazla 2 frame ve 180 us butce kullanir. SD capture 200 Hz ve SDIO 1-bit korunur.
7. `cpu_x100` artik scheduler task surelerine ek olarak olculen main-context/background servis surelerini de kapsar. `cpu_bg_x100` background payini verir.
8. UART 167 alanlidir ve P34 timing/SD/covariance diagnostiklerini ekler.
9. V55 preflight actuator hard gate ve STOP latch aynen korunur.

## UART yeni alanlar
- cpu_bg_x100
- sd_update_us / sd_update_max_us
- sd_drain_us / sd_drain_max_us / sd_drain_yields
- cov_us / cov_max_us / cov_count
- sched_slow_defers
- bg_fresh_us / bg_remote_us / bg_control_us
- bg_uart_us / bg_uart_max_us
- sd_defers / control_defers / uart_defers

## Ilk kart testi
```
python monitor_uart_p34.py --port COM8 --json > uart_p34.txt
```
20-30 saniye kayit alin.

Hedefler:
- `eskf_public_count`: 10 saniyede yaklasik +2000
- `sd_frames`: 10 saniyede yaklasik +2000
- `miss_imu`: tercihen sabit, en fazla cok seyrek artis
- `scheduler_realigns`: tercihen sabit, en fazla cok seyrek artis
- `task_imu_us`: P33'teki ~270-320 us degerinden daha dusuk
- `sd_ready=1`, `sd_logging=1`
- `sd_errors=0`, `sd_write_errors=0`, `sd_dropped=0`, `sd_ring_overruns=0`
- `system_fault=0`, `system_ok=1`

Not: `sched_slow_defers` sayacinin artmasi tek basina hata degildir. P34'te bu sayaç, yavas bir task'in IMU deadline'ini calmak yerine bilerek bir sonraki uygun slack slotuna ertelendigini gosterir.
