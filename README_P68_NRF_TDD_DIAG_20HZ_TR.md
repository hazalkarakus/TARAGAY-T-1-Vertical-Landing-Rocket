# TARAGAY-T1 P68 — NRF TDD Diagnostic + Gerçek 20 Hz Ground Test

## Amaç
P68 tek bir bench testiyle iki soruyu ayırmak için hazırlanmıştır:
1. BluePill yer istasyonu gerçekten 50 ms / 20 Hz komut gönderiyor mu?
2. Roket telemetriyi gerçekten TX ediyor mu, yoksa kayıp ground RX tarafında mı oluşuyor?

## Ana sistem koruması
P68 roket tarafı P67 tabanıdır. Flight/control/sensor/ESKF/SD/actuator/remote parser/NRF state-machine kaynakları değiştirilmemiştir.
Rokette kasıtlı kaynak değişiklikleri yalnız:
- `App/Common/app_version.h`
- `App/Services/UARTTelemetry/uart_telemetry.c`

UARTTelemetry değişikliği yalnız mevcut read-only NRF telemetry sayaçlarını bench UART'a ekler:
- nrf_tlm_schedule
- nrf_tlm_tx_start
- nrf_tlm_tx_success
- nrf_tlm_tx_fail
- nrf_tlm_pending_replace
- nrf_tlm_last_tx_us
- nrf_tlm_max_tx_us

`P68_MAIN_SYSTEM_UNCHANGED_SHA256.txt` dosyasında 25/25 kritik kaynak P67 ile byte-identical doğrulanmıştır.

## Ground V11 değişikliği
- heartbeat yapılandırması 50 ms / 20 Hz
- `next_heartbeat_tick` ile deterministik cadence
- RF protokol/payload/adres/kanal/CRC/Auto-ACK ayarları değiştirilmedi
- local UART'a yeni 0xFD timing diagnostic sayfası eklendi; RF üzerinden gönderilmez

0xFD sayfası şunları bildirir:
- build_id = 0x68 (P68/V11)
- HB_CFG = 50 ms
- gerçek TX interval last/min/max
- TX_START
- TX_OK
- TLM_RX
- TDD_TIMEOUT

## Tek test prosedürü
**Gaz/pressurized sistem bağlı olmasın. Aktüatör gücü güvenli/inhibit durumda olsun. SD kart takılı olsun.**

1. Her iki ground switch'i de OFF/açık konumda başlat. Özellikle E-STOP PA0 GND'ye çekili başlamasın.
2. Rokete P68 firmware'i flashla.
3. BluePill'e V11 P68 firmware'i flashla.
4. Ground UART monitor:
   `python -u monitor_ground_station_p68.py --port COM20`
5. Rocket UART monitor:
   `python -u monitor_uart_p68.py --port COM21 --log uart_p68.txt`
6. 60 saniye hiçbir switch'e dokunmadan çalıştır.
7. İstersen 60. saniyeden sonra Switch-1'i ~1 s tutup bırak (gaz ve aktüatör enerji yolu güvenli/inhibit durumda).
8. Testin en sonunda Switch-2 E-STOP'u ~1 s aktif et, sonra bırak. STOP_LATCH reset/power-cycle'a kadar 1 kalmalıdır.
9. Toplam 90–120 saniye sonra iki Python'u Ctrl+C ile kapat.

## İlk 60 saniyede beklenen
Ground ekranda:
- `GROUND_FW=P68/V11`
- `HB_CFG=50ms`
- `TX_INTERVAL last` yaklaşık 50 ms
- `CMD_RATE` yaklaşık 20 Hz

Rocket UART'ta:
- `nrf_rx_count` yaklaşık 20 Hz artmalı
- `nrf_tlm_schedule` aynı mertebede artmalı
- `nrf_tlm_tx_start` schedule'ı takip etmeli
- `nrf_tlm_tx_success` start'ı takip etmeli
- `nrf_tlm_tx_fail=0` ideal

## Tek testte teşhis matrisi
- Ground `GROUND_FW` görünmüyor veya HB_CFG != 50 ms -> yanlış/eski BluePill binary.
- Ground TX interval ~100 ms -> 20 Hz ground hâlâ uygulanmamış.
- Roket `schedule` ~20 Hz ama `tx_start` düşük -> rocket TDD pending/busy sorunu.
- Roket `tx_start` artıyor ama `tx_fail` artıyor -> rocket TX/role-switch sorunu.
- Roket `tx_success` ~20 Hz artıyor fakat Ground TLM_RX düşük -> problem ground RX/TDD capture tarafı.
- Roket `nrf_rx_count` ~20 Hz değil -> uplink cadence/ground TX tarafı.

## Kabul hedefleri
- GROUND_FW=P68/V11
- HB_CFG=50 ms
- CMD_RATE 19–21 Hz
- Rocket nrf_rx_count artış hızı 19–21 Hz
- nrf_errors=0
- nrf_invalid=0
- miss_nrf=0
- nrf_tlm_tx_fail=0
- 19/19 page
- telemetry CRC/format error=0
- SD ready/logging=1
- ESKF div=0, cov fault=0
- E-STOP sonunda latch=1 ve actuator_authorized=0
