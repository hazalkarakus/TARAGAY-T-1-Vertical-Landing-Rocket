# TARAGAY-T1 P64 NRF explicit TDD / NO-AUTOACK

## Neden P64?
P63 testinde yer istasyonu `TX_OK=0`, `TX_FAIL` sürekli artarken roket UART tarafında `nrf_rx_count` artıyordu. Bu, 4-byte komutun fiziksel olarak rokete ulaştığını fakat yer istasyonunun hardware Auto-ACK alamadığını gösterdi.

P63 tasarım hatası: yer istasyonu 32-byte telemetri dinleme penceresini yalnız `TX_DS` (hardware ACK başarısı) sonrasında açıyordu. ACK gelmediği için yer istasyonu hiç PRX telemetri penceresine giremiyordu.

P64 bu bağımlılığı kaldırır:
- `EN_AA = 0x00` iki tarafta
- `SETUP_RETR = 0x00` iki tarafta
- FEATURE=0, DYNPD=0
- Yer istasyonu 4-byte komutu bir kez yayınlar. Auto-ACK kapalı olduğundan TX_DS paket havaya çıktığında oluşur.
- Yer istasyonu hemen 32-byte PRX penceresine geçer.
- Roket geçerli komut alınca yaklaşık 4.5 ms sonra 32-byte telemetri sayfasını explicit PTX olarak yollar.
- Telemetri header içindeki `cmd_seq_echo`, uygulama seviyesinde komut alındı teyididir.
- Roket tekrar PRX olur; yer istasyonu tekrar PTX olur.

## Güvenlik
Tahliye ve latched E-STOP parser/mantığı değiştirilmedi. Telemetri alınmasa bile komut heartbeat'i devam eder. E-STOP tek pakete güvenmez; switch basılı kaldığı sürece aynı flag yeni sequence ile periyodik gönderilir. İlk test gazsız/aktüatör gücü güvenli durumda yapılmalıdır.

## Beklenen ilk test
Yer istasyonu:
- radio=1
- FEATURE=0x00
- DYNPD=0x00
- `CMD_TX` sürekli artar
- `CMD_TX_FAIL` 0 veya çok düşük
- `TLM_RX` sürekli artar
- `APP_LINK=1`
- pages 19/19
- TLM_BAD=0

Roket normal UART:
- nrf_connected=1
- nrf_link=1
- nrf_rx_count sürekli artar
- nrf_errors=0
- nrf_invalid=0
- miss_nrf=0

## Pinler
Yer istasyonu BluePill:
- PA1 -> Switch-1 -> GND : TAHLIYE
- PA0 -> Switch-2 -> GND : E-STOP
- PA2 IRQ, PA3 CSN, PA4 CE, PA5 SCK, PA6 MISO, PA7 MOSI
- PA9 USART1 TX -> USB-TTL RX
- GND -> USB-TTL GND

## Kurulum
Roket: `TGY_V8_19M_SMALLBOARD_FULLSYSTEM_P64_NRF_TDD_NOACK.zip`
Yer istasyonu: `nrf_verici_2SWITCH_LED_V9_TDD_NOACK.zip`

CubeIDE: Clean Project -> Build Project -> Flash.

Python:
```bash
python -u monitor_ground_station_p64.py --port COM21
python -u monitor_uart_p64.py --port COM20 --log uart_p64.txt
```
