# P112R12R8R8 — ESKF deterministic aiding — INERT

Bu revizyon R8R7 fiziksel logundaki ESKF/task-4 timing tepesini izole eder ve gravity aiding burst'ünü deterministik hale getirir.

## Değişenler
- Sensör sürücüleri değişmedi: IMU, BMP585 ve LiDAR R8R7 ile aynıdır.
- Scheduler değişmedi.
- Actuator/RCS/control/SD kaynakları değişmedi.
- Full-State ESKF gravity aiding: 50 Hz üç-eksen batch korunur; X/Y/Z sparse Joseph update'leri üç ardışık 200 Hz correction slotuna bölünür.
- Gravity measurement direction / predicted direction / R değeri batch başlangıcında snapshot alınır.
- Gate, noise, covariance ve measurement sabitleri değiştirilmedi.
- Live-debug commit 50 Hz'den 20 Hz'e düşürüldü; public ESKF output hâlâ 200 Hz'dir.
- UART yalnız READ-ONLY compact `$TGY70`; pure ESKF correction/predict ve alt-aiding sürelerini task-4 toplamından ayrı gösterir.

## Test
CubeIDE: Clean -> Build -> Flash -> Reset.
Tamamen basınçsız/INERT bench kullan.

```text
py monitor_uart_p112r12r8r8_eskf_determinism.py --port COM21 --duration 120
```

Çıktı: `uart_p112r12r8r8_eskf_determinism.txt`

Beklenen ana hedefler:
- BARO/LiDAR runtime dropout = 0
- miss_imu = 0'a yakın
- pure `eskf_corr_max_us` < 700 us
- `eskf_public_rate` >= 195 Hz, hedef ~200 Hz
- gravity_joseph_faults = 0
- covariance max < 1000 us
- system_ok = 1
- actuator/RCS hareketi = 0

Bu revizyon uçuş için değildir; actuator çıkışları INERT kalır.
