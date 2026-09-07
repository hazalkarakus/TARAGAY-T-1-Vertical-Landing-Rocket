# P33 – Fast IMU Software-SPI + SDIO 1-bit

P32 hedef kart UART logundan iki kök neden çıkarıldı.

## IMU / CPU
- P32: `task_imu_us` yaklaşık 2.6 ms, `cpu_imu_x100` yaklaşık 8100, CPU toplam yaklaşık %96.5.
- Neden: ISM330DLC için üç redundant 13-byte burst, her clock edgeinde HAL GPIO çağrıları + 40-loop delay ile bit-bang ediliyordu.
- P33: aynı Mode-3 software-SPI ve aynı üçlü redundant doğrulama korunur.
- SCK/MOSI/CS doğrudan GPIOA BSRR, MISO doğrudan IDR üzerinden erişilir.
- Edge delay sabit 12 NOP.
- Hedef: `task_imu_us < 700 us`, tercihen 300-500 us; `miss_imu` ve `scheduler_realigns` steady-state artmamalı; CPU < %60.
- ESKF public/correction 200 Hz ve covariance 25 Hz korunur.

## SD
- P32: `sd_hal_init=0` (HAL_OK), `sd_wide=0` (4-bit command HAL_OK) fakat `sd_last_result=1` (FR_DISK_ERR), mount yok.
- P33: HAL_SD_Init sonrası 4-bit ACMD6 geçişi yapılmaz; host/card 1-bit SDIO'da tutulur.
- 1-bit SDIO mevcut logger veri hızı için yeterlidir ve D1-D3 fiziksel güvenilirliğine bağımlılığı kaldırır.
- Hedef: `sd_mount_ok=1`, `sd_file_open=1`, `sd_ready=1`, `sd_logging=1`, `sd_frames` artmalı.

## UART doğrulama
Mevcut P32 149-alan `monitor_uart_p32.py` ile uyumludur; field sırası değişmedi.
