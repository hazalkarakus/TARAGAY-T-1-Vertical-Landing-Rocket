# P31 - nRF24 + SD Logger Aktif Full-System Profili

Bu profil P30 çekirdeğini korur ve iki opsiyonel çevrebirimi varsayılan olarak açar:

- `APP_OPTIONAL_NRF24_ENABLED = 1U`
- `APP_OPTIONAL_SDLOGGER_ENABLED = 1U`

## nRF24

- SPI3 üzerinden çalışır.
- `RemoteControl_Update()` ana döngüde sürekli servis edilir.
- Scheduler içindeki `Task_NRFMonitor_200Hz()` 5 ms / 200 Hz teşhis snapshot görevidir.
- RF link kaybı tek başına ana motor STOP latch oluşturmaz. STOP latch yalnız geçerli uzak komuttaki Switch-2 biti ile oluşur.
- Tahliye servo fiziksel kontrolü `APP_VENT_SERVO_ENABLED = 0U` olarak kalmıştır.

## SD Logger

- SDIO + DMA aktiftir.
- Uçuş/state frame capture: TIM5 ile 5 ms / 200 Hz.
- Her frame son beş 1 kHz IMU örneğini de taşır.
- Runtime yazma yolu RAM ring + çift buffer + çok-sektör SDIO DMA kullanır.
- FatFS mount ve preallocation yalnız startup aşamasında bloklayabilir; scheduler başlamadan önce yapılır.

## P30 çekirdeğinden korunanlar

- IMU propagation: 1 kHz
- ESKF correction/public output: 200 Hz
- Covariance propagation: 25 Hz
- ESKF ağır health/live debug: 50 Hz
- Barometre SPI kısa-timeout + otomatik recovery
- Röle bench modu ve 4-tur iğne vana bench modu varsayılan kapalı

## İlk kart testi için Live Expressions

### ESKF
- `full_eskf_public_output_count` veya ilgili public-output counter
- 10 saniyede yaklaşık +2000 beklenir.

### nRF24
- `v89_nrf_initialized`
- `v89_nrf_connected`
- `v89_nrf_task_counter`
- `v89_remote_link_active`
- `v89_remote_valid_packet_count`
- `v89_remote_timeout_count`

### SD
- `sd_logger_initialized`
- `sd_logger_mount_ok`
- `sd_logger_ready`
- `sd_logger_logging_active`
- `sd_logger_frame_count`
- `sd_logger_dropped_frame_count`
- `sd_logger_ring_overrun_count`
- `sd_logger_error_count`
- `v87_sd_update_max_us`

Normal durumda `sd_logger_frame_count` artmalı; `dropped_frame_count`, `ring_overrun_count` ve hata sayıları 0 kalmalıdır.
