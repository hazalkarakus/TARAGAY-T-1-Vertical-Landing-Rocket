# P50 – Fast ZUPT Joseph + Scheduler Fix

P49R1 365 s bench testinde ESKF/SD/FIFO ana hedefleri geçti. Ancak UART + SD zamanlaması şu deterministik ilişkiyi gösterdi:

- `zupt_joseph_updates = 4020`
- `miss_imu = 1340`
- `4020 / 3 = 1340`

Yani her 3-eksen ZUPT çevrimi tam olarak bir 1 kHz IMU release kaçırıyordu. Kök neden P49 masked Joseph hesabının Cortex-M4F üzerinde software-double ile 15x15 çalışmasıydı.

P50 aynı masked Joseph matematiğini cebirsel olarak sadeleştirir. BGX/BGY/BGZ gainleri yine sıfırdır; BGxBG covariance bloğu korunur. P+ = P - p_k p_k^T/S eşdeğer rank-one formu hardware float ile hesaplanır ve mevcut covariance guard/rollback aynen korunur.

UART: `$TGY61`, 258 alan.

İlk bench hedefleri:
- `zupt_joseph_faults = 0`
- `eskf_cov_faults = 0`
- ZUPT artarken `miss_imu` artık artmamalı veya pratikte sıfıra yakın kalmalı
- `sd_dropped = 0`, `sd_fifo_order_faults = 0`
