TARAGAY-T1 P59 - P57 TABANLI nRF TAHLIYE + ACIL STOP ENTEGRASYONU
================================================================

TABAN
Bu proje P57_IMU_STALE_STATUS_WARMUP tabanindan uretilmistir. P57'nin IMU,
ESKF, barometre, LIDAR, SDIO/SD logger, scheduler ve needle kontrol algoritmalari
NRF entegrasyonu icin yeniden tasarlanmadi. IMU surucu dosyalari aynen korunur.

RF ESLESMESI
- Adres: TGY01
- Kanal: 76
- Data rate: 1 Mbps
- RF power: 0 dBm
- CRC: 2 byte
- Pipe 0 Auto-ACK
- Payload: 4 byte
  Byte0=0xA5
  Byte1 flags: bit0 TAHLIYE, bit1 ACIL STOP
  Byte2 sequence
  Byte3=0xA5 XOR flags XOR sequence XOR 0x5A
- Uygulama paketi hataliysa komut kabul edilmez; bad_magic/bad_flags/bad_checksum
  sayaclari artar.

F407 nRF PINLERI (SPI3)
- CSN PD0
- CE  PD1
- IRQ PD3
- SCK PB3
- MISO PB4
- MOSI PB5
- VCC 3.3 V, GND ortak

YER ISTASYONU F103 PINLERI
- CE PA4, CSN PA3, IRQ PA2, SCK PA5, MISO PA6, MOSI PA7
- Switch-1 / TAHLIYE: PA1 -> switch -> GND
- Switch-2 / ACIL STOP: PA0 -> switch -> GND

F407 LED ANLAMLARI
- PD12 YESIL: gercekte bir tahliye solenoidi enerjili
- PD13 TURUNCU: gecerli NRF linki var
- PD14 KIRMIZI: ACIL STOP latch olmus

SWITCH-1 / TAHLIYE
- Sadece ground durumunda calisir; PE9 flight-active olduktan sonra tahliye reddedilir.
- Gecerli ve taze paket gerekir. Genel RF link timeout 500 ms, fiziksel tahliye icin
  daha siki packet-age limiti 250 ms'dir. Paketler kaybolursa solenoidler fail-closed olur.
- Yanlis dokunmayi azaltmak icin switch 250 ms surekli tutulmadan fiziksel cikis baslamaz.
- Mevcut 4 RCS solenoidi SIRALI kullanilir; ayni anda yalnizca bir vana acar:
  IN1/X+ -> IN2/X- -> IN3/Y+ -> IN4/Y- -> tekrar.
- Her vana 100 ms ON, ardindan 50 ms break-before-make OFF araligi vardir.
- Switch birakilinca, RF tazeligi kaybolunca, flight active/fault olunca veya STOP gelince
  tum solenoidler derhal safe/off komutuna cekilir.
- Opposing-pair interlock korunur; yeni yol all-valves bypass yapmaz.

SWITCH-2 / ACIL STOP
- Gecerli pakette bit1 gorulur gorulmez v30_stop_latched=1 olur.
- STOP, tahliyeden daha yuksek onceliklidir.
- Attitude/RCS komutlari ForceSafe edilir; fiziksel solenoid maskesi 0 olur.
- NeedleValveController_Stop() cagrilir; motor surucu komutu kapatilir.
- 1 kHz ve 200 Hz kontrol yollarinda stop latch gate olarak kalir; normal kontrol STOP'u
  sonraki tickte yeniden acamaz.
- Switch birakmak STOP'u temizlemez. Temizlemek icin kart reset/power-cycle gerekir.

GUVENLIK / TEST SIRASI
1) Ilk testi BASINCSIZ yap. Gaz/solenoid gucunu ayir veya dummy LED/relay yukleri kullan.
2) NRF linki: PD13 yanmali, rx/valid sayaclari artmali, invalid=0 olmali.
3) Switch-1'i >250 ms tut: PD12 pulse etmeli; applied mask sirayla 1,2,4,8 olmali
   ve hicbir anda birden fazla bit set olmamali. Birakinca mask 0 olmali.
4) TX'i kapatirken tahliye acik olsun: en gec yaklasik 250 ms packet-age penceresinden sonra
   fiziksel tahliye kapanmali.
5) Switch-2'yi EN SON dene: PD14 kalici yanmali, solenoid mask 0 ve needle output 0 olmali.
6) STOP sonrasi normal komutla tekrar hareket edemedigini dogrula; sonra resetle.

KRITIK UYARILAR
- RCS tahliyesi gaz cikisi nedeniyle kuvvet/tork uretir. Basincli testte roket mekanik olarak
  sabitlenmeden tahliye testi YAPMA.
- MCU GPIO'sundan solenoid direkt surulmez; mevcut relay/driver, uygun sigorta ve flyback/TVS
  korumasi kullanilmalidir. Cikislar active-LOW'dur.
- RF tabanli STOP tek hata noktasidir ve 'gercek' E-stop yerine gecmez. Final sahada aktüator
  beslemesini bagimsiz olarak kesen, tercihen normally-closed ve kablo kopmasinda da guvenli
  duruma giden donanimsal kill/E-stop hatti kullan.
- nRF24 3.3 V beslenmeli; modül dibinde 10 uF + 100 nF decoupling onerilir.
- Basincli tahliye icin 100 ms pulse/50 ms deadtime degerleri ilk guvenli commissioning
  degerleridir; gaz debisi ve mekanik tepki gorulmeden uzatilmamalidir.

UART
P57'nin $TGY68/V68 telemetri formati bilerek degistirilmedi. Mevcut monitor/SD analiz
araclarini bozmamak icin yeni tanilar CubeIDE Live Expressions ile izlenir.
