V39 UART STATUS DIAGNOSTIK
==========================
Taban: V38 / kullanicinin calisan V26 kaynaklari.

Kaynak kodda sadece App/app.c degistirildi.
Sensor suruculeri, SensorManager, scheduler, SD, NRF, Lidar, barometre, IMU,
needle, solenoid ve servo dosyalari degistirilmedi.

USART2 TX: PA2 -> CH340 RX
GND -> CH340 GND
115200 8N1
DMA: mevcut USART2 TX DMA altyapisi kullanilir.

Saniyede bir satir:
STAT I=0 B=0 L=0 SD=0 LOG=0 N=0 LINK=0

I    = sensor_imu_valid
B    = sensor_baro_valid
L    = Lidar_IsDistanceValid()
SD   = sd_logger_ready
LOG  = sd_logger_logging_active
N    = NRF24_IsConnected()
LINK = RemoteControl_IsLinkActive()

Bu surum sensorleri UART icin init etmez veya guncellemez; yalniz mevcut durum
bayraklarini okur. snprintf/float formatlama/global IRQ kapatma yoktur.
