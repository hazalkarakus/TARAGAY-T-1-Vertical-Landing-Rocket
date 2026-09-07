# P112R12R8R7 — SENSOR/SCHED RECOVERY INERT

Bu paket P112R12R8R6 fiziksel UART testinin iki regresyonunu hedefler:

- R8R6: `miss_imu=0`, `miss_eskf≈0`, CPU ≈ %45.6 -> KORUNDU.
- R8R6: BARO/LiDAR scheduler reserve 650/600 us -> sensör servis starvation -> DÜZELTİLDİ.
- R8R6: LiDAR runtime recovery delay 0 ms -> recovery storm -> R8R5 kanıtlanmış sürücüye GERİ ALINDI.

R8R7 değişiklikleri:
1. `App/Modules/Sensors/Lidar/Lidar.c` R8R5 ile byte-for-byte aynıdır.
2. BARO slot reserve: 650 -> 360 us.
3. LiDAR slot reserve: 600 -> 360 us.
4. Compact UART min slack: 650 -> 450 us (R8R6 physical max ~379 us).
5. R8R6 incremental ESKF integrity, compact TGY69 ve inert safety korunur.
6. Aktüatör/RCS fiziksel çıkışları INERT kalır. Uçuşa uygun değildir.

Beklenen 120 s bench kriterleri:
- `miss_imu` <= 5, tercihen 0.
- `miss_eskf` <= 3.
- BARO valid/fresh runtime dropout = 0.
- `miss_baro` hızlı büyümemeli; final tercihen <= 10.
- LiDAR fresh büyük çoğunlukta 1; recovery storm olmamalı.
- `task_lidar_max_us` < 1000 us.
- CPU < %60.
- PWM/RCS/hardoff davranışı inert ve güvenli kalmalı.
