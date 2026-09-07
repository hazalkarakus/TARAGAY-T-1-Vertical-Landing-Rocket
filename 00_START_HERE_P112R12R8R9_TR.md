# P112R12R8R9 — ESKF covariance deterministic timing — INERT

R8R8 fiziksel logu saf ESKF correction yolunun bounded oldugunu gosterdi:
- pure correction max ~553 us
- task-4 max ~628 us
- IMU miss = 0

Kalan kok neden 25 Hz covariance propagation servisiydi: Debug build'de ~980 us goruldu,
ama scheduler bu servisi 650 us reserve ile kabul ediyordu. Bu servis bir sonraki 1 kHz
IMU slotuna tasarak ESKF/BARO release zincirini geciktirebiliyordu.

R8R9 degisikligi DAR kapsamli:
- Sadece Full-State ESKF covariance propagation + covariance sanitization kernel'leri
  GCC function-specific O2 ile derlenir.
- -ffast-math YOK.
- ESKF gate/noise/denklem/frekanslari degismedi.
- 200 Hz correction, 25 Hz covariance, 50 Hz gravity batch korunur.
- Scheduler, IMU, BARO, LiDAR, SD, control, actuator/RCS kaynaklari degismedi.

## Test
CubeIDE: Clean -> Build -> Flash -> Reset

Sonra:
`py monitor_uart_p112r12r8r9_eskf_cov_determinism.py --port COM21 --duration 120`

Hedef:
- cov_max_us < 650 us (tercihen <500 us)
- miss_imu = 0
- miss_eskf delta <= 5 / 120 s, tercihen 0
- miss_baro delta <= 5 / 120 s, tercihen 0
- pure ESKF correction <700 us
- ESKF public output >=195 Hz
- BARO/LiDAR/ESKF freshness kaybi yok
- system_ok=1 warmup sonrasi
- actuator/RCS tamamen inert

BU SURUM UCUŞ ICIN DEGILDIR. INERT BENCH VALIDATION ICINDIR.
