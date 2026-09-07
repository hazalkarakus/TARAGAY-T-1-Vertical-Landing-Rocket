# P112R12R8R13 — P73 FAST/REDUNDANT NRF + R8R11 FROZEN CORE

Bu revizyon P112R12R8R12 tabanidir. Sensör/ESKF/scheduler katmani R8R11 frozen kalir.

## Alinan nRF sistemi
Kaynak: kullanicinin TARAGAY_P73_P71_REDUNDANT_RF_BUNDLE.zip içindeki
`TARAGAY_ROCKET_P73_REDUNDANT_FAST_TLM.zip`.

P73'ten **yalniz** NRFTelemetry fast/redundant state-machine tasindi:
- 4-byte Ground->Rocket command: 20 Hz / 50 ms heartbeat korunur.
- RF: TGY01, kanal 76, 1 Mbps, 0 dBm, 2-byte CRC.
- FAST packet type: 0xFC.
- FAST:DETAIL = 7:1.
- FAST payload: AGL, ESKF Vz, pitch/yaw, pitch/yaw rate, RCS req/applied,
  needle ADC, main valve command, estimated thrust.
- Her mantiksal 32-byte telemetri frame'i ayni sequence ile 5 ms sonra bir kez daha gonderilir.
- Yeni command gelirse baslamamis duplicate iptal edilir; command her zaman onceliklidir.

## Bilerek degistirilmeyenler
- NRF24 driver
- RemoteControl command parser
- scheduler ve task periyotlari
- IMU / BARO / LiDAR
- Full-State ESKF / covariance / gravity aiding
- SD logger
- GeneratedFlightControl
- needle / solenoid / preflight / SystemMonitor

R8R11 INERT cikis izolasyonu korunur. Ilk test gazsiz/basincsiz ve aktüatör enerji yolu inhibit durumda yapilmalidir.

## Ground tarafi
Bu rocket firmware P73 bundle'daki redundant/fast ground ile eslestirilmelidir:
- TARAGAY_GROUND_P71_REDUNDANT_RX.zip
- TARAGAY_GROUND_STATION_V3_2_REDUNDANT_FAST.zip

## Rocket UART coexistence testi
```
py monitor_uart_p112r12r8r13_p73_fast_nrf.py --port COM21 --duration 120
```

Beklenen frozen-core kabul:
- nrf_connected=1, nrf_link=1
- nrf valid RX ~20 Hz
- primary TDD TX success ~20 Hz
- nrf_errors=0, nrf_invalid=0
- miss_imu/baro/lidar/nrf/eskf = 0
- scheduler_realigns=0
- system_ok=1 ve sensor/ESKF freshness kaybi yok

Ground UI'da FAST frame akisinin 10 Hz'nin belirgin ustunde olmasi beklenir; bundle hedefi ~14-15 Hz usable FAST ekrana gelisidir.
