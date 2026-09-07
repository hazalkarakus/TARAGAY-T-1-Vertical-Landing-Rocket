# P39 — Fusion / IMU / RCS Hardening

P39 is based on P38 and preserves the proven 200 Hz SD logger, non-blocking
IMU/LiDAR recovery, 1 kHz IMU scheduler and same $TGY55 UART/CRC protocol.

Changes:

1. **RCS hard fail-safe at 1 kHz**
   - A current fast fault is evaluated directly from IMU, barometer, LiDAR,
     ESKF and SD state.
   - Any critical fault forces the physical RCS mask to zero.
   - TIM7 also aborts the internal pulse state so a stale pulse cannot reopen
     when a transient fault clears.

2. **ESKF–LiDAR dead-zone recovery**
   - Normal LiDAR innovation gate remains 0.25 m.
   - A stable LiDAR stream outside that gate but within 1.0 m must persist for
     6 samples before bounded reacquisition is allowed.
   - Reacquisition still limits one correction to 5 cm position and 0.5 m/s
     velocity. It exits only after 8 samples within 0.12 m.

3. **IMU exact-repeat stale limit: 100 ms -> 20 ms**
   - Six-axis bit-identical frozen samples are rejected/recovered much sooner.

4. **Barometer one-frame raw spike guard**
   - >50 Pa single-sample jumps are held out of the published raw state.
   - A real persistent pressure step is accepted after two consistent samples.

5. **LiDAR service cadence**
   - An IMU-protected LiDAR defer is carried to the next IMU slot instead of
     being discarded, while intentional carries do not create false deadline
     faults.

6. **Covariance readiness**
   - Sample threshold 40 -> 32; the task period remains exactly 25 Hz.
   - Covariance always integrates the actual accumulated IMU dt.

UART is **212 fields** in P39. Six focused diagnostics were added:
`fast_fault`, `baro_raw_spike_rejects`, `baro_raw_step_confirms`,
`eskf_lidar_soft_reacq_active`, `eskf_lidar_soft_reacq_count`, and
`eskf_lidar_soft_reacq_success`. Use `monitor_uart_p39.py`.

## İlk P39 bench testi

İlk test gaz/basınç ve fiziksel aktüatörler bağlı olmadan yapılmalıdır.
Önerilen süre en az 180 saniyedir.

Özellikle kontrol edilecekler:

- `scheduler_realigns` yaklaşık 0, `miss_imu` 0 olmalı.
- `sd_dropped` ve `sd_ring_overruns` 0 kalmalı; `sd_frames` yaklaşık 200 Hz artmalı.
- `fast_fault != 0` görülen hiçbir frame'de `rcs_applied_mask` sıfırdan farklı olmamalı.
- IMU exact-repeat donması yaklaşık 20 ms içinde invalid/recovery yoluna alınmalı.
- `baro_raw_spike_rejects` tek-frame basınç sıçraması olduğunda artmalı.
- `eskf_lidar_soft_reacq_count` 0.25..1.0 m dead-zone oluşursa artmalı;
  başarılı yakınsamada `eskf_lidar_soft_reacq_success` artmalı.
- LiDAR fiziksel okuma hızı P38'deki yaklaşık 67 Hz seviyesinden yükselmelidir.
- Covariance sayacı 25 Hz hedefine yaklaşmalıdır.
