V38 - UART HEARTBEAT ONLY TEST
==============================
Taban: kullanicinin calistigini belirttigi V26.
Degisen tek kaynak dosya: App/app.c

Amaç:
- Sensor, SD, nRF, RCS, needle veya scheduler verisi okumadan UART DMA tasimasini test etmek.
- USART2/PA2 uzerinden her 1 saniyede sadece "ALIVE\r\n" gonderir.
- UARTTelemetry servisi CAGIRILMAZ.
- Snapshot, formatlama, SDLogger_IsReady/IsLogging ve global IRQ maskesi YOKTUR.

Baglanti:
STM32F407 PA2 (USART2_TX) -> CH340 RX
STM32F407 GND -> CH340 GND
115200 8N1
CH340 5V baglanmaz; kart kendi beslemesinden calisir.

Test:
1) V38'i Clean/Build/Flash et.
2) Sistemin V26'daki sensor ve kontrol davranisinin aynen devam edip etmedigini kontrol et.
3) Python/terminalde saniyede bir ALIVE gor.
4) Sistem calisiyor + ALIVE geliyorsa UART DMA tasimasi saglam; sorun tam telemetri snapshot/paketleme katmanindadir.
