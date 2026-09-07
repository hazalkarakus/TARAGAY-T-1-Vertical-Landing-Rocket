STM32F407 TGY PROJESİNE nRF24L01 ALICI EKLEMESİ
==============================================

AMAÇ
- STM32F103 vericiden gelen ON/OFF paketini almak.
- Geçerli ve güncel ON komutunda STM32F4-Discovery üzerindeki yeşil PD12 LED'i
  250 ms aralıkla yakıp söndürmek.
- OFF komutunda veya 500 ms boyunca geçerli paket gelmezse LED'i söndürmek.

BAĞLANTILAR (STM32F407-DISCOVERY)
--------------------------------
nRF24L01 VCC  -> 3.3V (KESİNLİKLE 5V DEĞİL)
nRF24L01 GND  -> GND
nRF24L01 CE   -> PD1
nRF24L01 CSN  -> PD0
nRF24L01 SCK  -> PB3  (SPI3_SCK)
nRF24L01 MISO -> PB4  (SPI3_MISO)
nRF24L01 MOSI -> PB5  (SPI3_MOSI)
nRF24L01 IRQ  -> PD3
Durum LED'i   -> kart üzerindeki PD12 yeşil LED

RADYO AYARLARI
--------------
Adres       : TGY01 (5 byte)
Kanal       : 76
Veri hızı   : 1 Mbps
RF gücü     : 0 dBm
CRC         : 2 byte
Payload     : 4 byte
Auto-ACK    : Açık, pipe 0

YAZILIMA EKLENENLER
-------------------
- App/Modules/NRF24 modülü derlemeye dahil edildi ve pin isimleri düzeltildi.
- App/Modules/RemoteControl eklendi.
- App_Init içinde nRF alıcı başlatılıyor.
- App_Run içinde nRF alıcı kesintisiz ama bloklamadan servis ediliyor.
- PD0/PD1/PD3 ve PD12 GPIO ayarları eklendi.
- EXTI3 kesmesi eklendi; ayrıca 5 ms polling yedeği vardır.
- 500 ms haberleşme zaman aşımında güvenli OFF uygulanır.

CUBEIDE LIVE EXPRESSIONS İÇİN DEĞİŞKENLER
----------------------------------------
remote_rx_command             0=OFF, 1=ON
remote_rx_link_active         1 ise son 500 ms içinde geçerli paket var
remote_rx_last_sequence       son paket sıra numarası
remote_rx_valid_packet_count  geçerli paket sayısı
remote_rx_invalid_packet_count hatalı paket sayısı
remote_rx_timeout_count       bağlantı zaman aşımı sayısı
nrf24_connected               sadece SPI/register erişiminin çalıştığını gösterir
nrf24_rx_count                alınan payload sayısı

STM32CUBEIDE'DE DERLEME
-----------------------
1. File > Import > Existing Projects into Workspace
2. Bu klasörü seçin: tgy
3. Project > Clean
4. Çekiç simgesine basıp Debug derleyin.
5. Başarılı derlemeden sonra beklenen dosya:
   Debug/tgy.hex

TEST SIRASI
-----------
1. İki nRF modülünü de 3.3V ile besleyin ve GND'leri ortaklayın.
2. Önce F407 alıcıyı, sonra F103 vericiyi çalıştırın.
3. F103 anahtarı OFF konumundayken F407 PD12 LED sönük kalmalı.
4. Anahtarı ON yapınca F407 PD12 LED yanıp sönmeli.
5. F103'ün gücünü kesince F407 LED en geç 500 ms içinde sönmeli.

DONANIM NOTU
------------
nRF24L01 besleme pinlerinin yakınına 10 uF elektrolitik ve 100 nF seramik
kondansatör paralel bağlamak, ani akım nedeniyle oluşan kararsızlığı azaltır.
