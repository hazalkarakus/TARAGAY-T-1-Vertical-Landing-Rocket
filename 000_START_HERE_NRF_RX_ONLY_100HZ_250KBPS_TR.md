# TARAGAY T1 - NRF RX ONLY / GROUND 100 Hz / RF 250 kbps

Bu varyant, onceki `NRF_RX_ONLY_GROUND100HZ` projesinin sadece nRF hava veri hizini degistirir.

## Aktif nRF yapisi
- Haberlesme yonu: YER ISTASYONU -> ROKET
- Roket -> yer nRF downlink: KAPALI
- Ground heartbeat: 10 ms = nominal 100 komut paketi/s
- Roket RX polling: 5 ms = 200 Hz
- Payload: 4 byte
- Kanal: mevcut proje ayari korunur
- Auto-ACK: kapali
- Hardware retransmit: kapali
- RF data rate: **250 kbps**
- RF power: 0 dBm
- Beklenen RF_SETUP register: **0x26**

## Degisen kaynak
`App/Modules/NRF24/nrf24.c`
- RF_SETUP 0x06 (1 Mbps) -> 0x26 (250 kbps)
- Konfigurasyon readback dogrulamasi da 0x26 bekleyecek sekilde guncellendi.

## Onemli
Yer istasyonu da ayni anda 250 kbps firmware kullanmalidir. Bir taraf 1 Mbps, diger taraf 250 kbps olursa haberlesemezler.

CubeIDE'de Clean Project + Build Project yapip yeni firmware'i olusturun. Ilk testi basincsiz/aktuatorler guvenli durumda yapin.
