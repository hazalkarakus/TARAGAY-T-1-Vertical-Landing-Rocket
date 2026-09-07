# P112R12R8R12 — NRF COEXISTENCE INERT

Taban: fiziksel 10 dakikaya yakın soak testini 0 scheduler miss ile geçen P112R12R8R11.

## Bu revizyonda değişen runtime davranış
- `APP_OPTIONAL_NRF24_ENABLED = 1`: mevcut/proven SPI3 nRF24 + RemoteControl + explicit-TDD downlink yolu tekrar aktif.
- Sensör, Full-State ESKF, scheduler, covariance, SD bounded-background, actuator ve RCS algoritmaları değiştirilmedi.
- `APP_P112R12_INERT_OUTPUT_ISOLATION_MODE = 1` korunur: RCS/ground-vent fiziksel çıkışları hard-safe kalır; vent servo zaten disabled.
- İlk testte iki ground switch de OFF bırakılmalıdır. Switch-2 E-STOP mantıksal olarak latch olabilir ve reset gerektirir.
- UART compact observer `$TGY72` nRF RX/TX/task sayaçlarını ekler. UART hâlâ TX-only/read-only bench diagnostics.

## RF eşleşmesi
- Address: `TGY01`
- Channel: 76
- 1 Mbps, 0 dBm
- 2-byte HW CRC
- EN_AA=0 / SETUP_RETR=0
- uplink command: static 4 byte
- downlink telemetry: static 32 byte, explicit TDD
- mevcut P69/V12 fast-PRX ground station protokolü ile uyumlu.

## İlk test
1. Gaz/basınç YOK. RCS/solenoid ve motor güç yolunu güvenli/inhibit tut.
2. Ground switch-1 ve switch-2 OFF/açık.
3. Rocket R8R12 flash. Ground tarafında P69/V12 fast-PRX uyumlu firmware.
4. Rocket UART: `py monitor_uart_p112r12r8r12_nrf_coexistence.py --port COM21 --duration 120`
5. İlk 120 s switchlere dokunma.

## Kabul
- nrf_connected=1, nrf_link=1
- nrf channel=76, RF setup=6
- RX/valid yaklaşık ground heartbeat hızında artar (20 Hz ground ise yaklaşık 20 Hz)
- invalid/errors=0
- TDD tx_success artar; tx_fail ideal 0
- miss_imu/baro/lidar/nrf/eskf=0 hedef
- scheduler_realigns=0 hedef
- BARO/LiDAR/ESKF freshness dropout=0
- p110_pwm=0, p111_moves=0, rcs_mask=0

Bu hâlâ INERT/DEPRESSURIZED qualification paketidir; uçuş firmware'i değildir.
