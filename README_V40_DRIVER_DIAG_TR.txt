V40 DRIVER-LEVEL UART DIAGNOSTIC
================================
Taban: V39 / kullanicinin V26 (6)(1) kaynaklari.
Kaynak kodda sadece App/app.c degistirildi.
Sensor, SD, NRF, scheduler ve kontrol suruculerine dokunulmadi.

USART2: PA2 TX -> CH340 RX, GND ortak, 115200 8N1.
DMA: DMA1 Stream6 Channel4 (mevcut transport aynen kullanilir).

Her 2 saniyede bir:
TASK ...
IMU ...
BARO ...
LIDAR ...
SD ... NRF ...
yazdirir.

Amac: valid=0 olmasinin init, baglanti, DMA, sample veya kalibrasyon
katmanlarindan hangisinde kaldigini ayirmak.
