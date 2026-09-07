# TARAGAY T1 - nRF RX-ONLY / 100 Hz Ground Command Build

Bu varyantta nRF RF yönü bilinçli olarak **tek yönlüdür**:

- Yer istasyonu -> Roket: AKTIF
- Roket -> Yer istasyonu nRF downlink: KAPALI
- nRF hardware Auto-ACK: KAPALI (roket RF ACK de göndermez)
- RF kanal: 76
- RF air data rate: 1 Mbps
- Komut paketi: 4 byte (`0xA5, flags, sequence, checksum`)
- Roket komut RX poll: 5 ms = 200 Hz
- Önerilen eş yer istasyonu heartbeat: 10 ms = 100 paket/s
- Roket remote link timeout: 500 ms (değiştirilmedi)

## Kaynak değişiklikleri

`App/Common/app_config.h`:
- `APP_NRF_ROCKET_DOWNLINK_ENABLED = 0`
- `APP_NRF_FLIGHT_COMMAND_PRIORITY_RX_ONLY = 1`
- `APP_NRF_FLIGHT_MINIMAL_FAST_TDD = 0`

`App/app.c`:
- `NRFTelemetry_Init()` ve `NRFTelemetry_Service()` yalnızca
  `APP_NRF_ROCKET_DOWNLINK_ENABLED != 0` ise çağrılır. Bu build'de çağrılmaz.
- `RemoteControl_Update()` aktif kalır; roket sürekli PRX/receive tarafındadır.

## Önemli

ZIP içindeki eski `Debug` build çıktıları bilinçli olarak kaldırılmıştır.
STM32CubeIDE içinde **Clean + Build** yapıp yeni firmware'i üretin.
Önce basınçsız/aktüatör güvenli bench testinde `remote_rx_valid_packet_count` ve
`remote_rx_last_packet_age_ms` değerlerini doğrulayın.
