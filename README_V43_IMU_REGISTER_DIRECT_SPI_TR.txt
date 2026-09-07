TGY V8.19M V43 - IMU REGISTER + DIRECT SPI1 TEST

TABAN: V42 IMU physical line test.

DEGISEN KAYNAKLAR:
- App/Modules/Sensors/IMU/imu.c
- App/Modules/Sensors/IMU/imu.h (yalniz yorum)
- App/app.c (yalniz IMU diagnostik/retry + UART diagnostik satirlari)

DOKUNULMAYANLAR:
- LiDAR driver/task
- BMP585/barometre driver/task
- SDIO/SD logger
- nRF24
- RCS/role/STOP/tahliye
- needle/motor
- scheduler periyotlari

V43 AMACI:
1) IMU register polling icin HAL_SPI_TransmitReceive yerine SPI1 DR/SR tabanli
   kisa, blocking register transfer yolu kullanir. 1 kHz raw veri yolu DMA olarak kalir.
2) Normal init basarisizsa V42 bit-bang testi WHO_AM_I, CTRL1_XL, CTRL2_G,
   CTRL3_C, CTRL4_C ve STATUS registerlarini okur.
3) AF5 geri yuklendikten sonra SPI1 peripheral uzerinden Mode0 ve Mode3 WHO_AM_I
   dogrudan test edilir.
4) Bit-bang cevabi varsa IMU_Init sadece bir kez yeniden denenir.

UART EK SATIRLARI:
IMUREG MODE=... WHO=... C1=... C2=... C3=... C4=... ST=...
IMUDIR M0=... M3=... OK=... TX=... DE=... RETRY=...

HEDEF:
IMU INIT=1 CON=1 ve WHO=0x6A veya mevcut projede kabul edilen alternatif 0x6B.

NOT:
0x6A bu projenin ISM330DLC beklenen kimligidir. 0x6B ham bir alternatif kimlik
olarak raporlanir; kart uzerindeki kesin parca adini yalniz bu byte'a bakarak
bu diagnostik surum isimlendirmez.
