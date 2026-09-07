# TARAGAY-T1 P67 - P64 Single-Slot TDD + 20 Hz Ground Cadence

## Neden P67?
P64 testinde downlink teslim orani yaklasik %77 idi. P65 iki yakin kopyada yaklasik %39'a, P66 8/20 ms iki slotta yaklasik %29'a dustu. Bu nedenle P67, roket tarafinda P64'te kanitlanan tek-slot 4.5 ms downlink yoluna geri doner. Guvenilirligi ayni TDD penceresine ek kopya sikistirarak degil, yer istasyonunun komut/telemetri firsatlarini 100 ms'den 50 ms'ye cikartarak artirir.

## Mimari
- Ground -> Rocket: 4 byte komut, 20 Hz (50 ms heartbeat)
- Rocket -> Ground: her kabul edilen komuttan ~4.5 ms sonra tek 32 byte telemetry frame
- Ground RX window: 30 ms
- Auto-ACK: kapali
- Dynamic payload: kapali
- ACK payload: kapali
- RF: kanal 76, 1 Mbps, 0 dBm, 2 byte RF CRC

## Guvenlik
Komut paketi ve parser P60/P64 ile aynidir.
- PA1 -> GND: Tahliye
- PA0 -> GND: latched E-Stop
- E-Stop: stop_latched=1, actuator_authorized=0, RCS safe, needle stop
- Telemetri kaybi komut guvenlik mantigini acmaz veya bypass etmez.
- RF E-Stop fiziksel hardwired E-Stop yerine kullanilmamalidir.

## P66 testinde gorulen ayri konu: SD
P66 UART'ta `system_fault=17`, `sd_mount_ok=0`, `sd_ready=0`, `sd_logging=0`, `sd_errors=1` goruldu. Fault 17 `SYS_FAULT_SD_LOGGING`'dir. Bu P67 ile bypass edilmez. SD kart testte bilerek yoksa haberlesme testi yapilabilir; fakat tam sistem/aktuator kabul testinde SD takili ve logging saglikli olmalidir.

## Python monitor
`monitor_ground_station_p67.py` tum 19 telemetry sayfasini canli dashboard olarak gosterir ve ayrica:
- DL(interval)
- CMD_RATE [Hz]
- TLM_RATE [Hz]
- TLM_AGE [ms]
- MAX_PAGE_AGE [ms]
- PAGES 19/19
- IMU raw/scaled/filtered
- Baro
- Lidar
- Attitude/quaternion
- ESKF position/velocity/accel/bias/innovation
- RCS/needle/E-Stop/system/SD
alanlarini gosterir.

Komut:
```
python -u monitor_ground_station_p67.py --port COMxx
```

Normal rocket UART:
```
python -u monitor_uart_p67.py --port COMyy --log uart_p67.txt
```

## Ilk kabul hedefi
P67'de asil hedef tek paket basari yuzdesini zorla %95 yapmak degil, kritik uplink'i bozmadan kullanisli telemetri throughput'u elde etmektir.
- CMD_RATE ~20 Hz
- nrf_errors=0
- nrf_invalid=0
- miss_nrf=0
- TLM_RATE tercihen >10 Hz
- PAGES=19/19
- MAX_PAGE_AGE tercihen <2.5 s
- CRC_BAD=0, TLM_BAD cok dusuk/0

Aktuator/gaz olmadan once haberlesme bench testi yapin.
