# TGY V8.19M V55 P29 — scheduler ve sensör donma koruması

Bu yama PE9 uçuş kilidini, RCS pinlerini, röle polaritesini ve ana motor
güvenlik kapılarını değiştirmez. Amaç, RCS gücü verildiğinde görülen CPU/I²C
gecikmelerinin geçmiş görevleri yığmasını ve sensör donmalarının geçerli veri
gibi görünmesini önlemektir.

> İlk doğrulama ana motor, solenoid, basınç ve ateşleme gücü fiziksel olarak
> kapalıyken yapılmalıdır. Host/statik test sonucu uçuş yeterliliği değildir.

## P29 değişiklikleri

1. Bir görev en az bir tam periyot gecikmişse scheduler geçmiş periyotları art
   arda çalıştırmaz; sonraki zamanı `şimdi + periyot` olarak yeniden hizalar.
   Toplam olay sayısı UART'ta `scheduler_realigns` alanındadır.
2. LiDAR `WAIT_MEASUREMENT` durumunda DMA okuması `HAL_BUSY` kalırsa 30 ms
   sonunda işlem iptal edilir, I2C2 task bağlamında yeniden başlatılır ve 50 ms
   sonra ölçüm yeniden denenir. 100 ms yeni örnek gelmemesi de aynı kurtarmayı
   başlatır. Sürücü ölçümü kalıcı olarak bırakmaz.
3. IMU'nun gerçek DMA örnek zaman damgası barometreden bağımsız tutulur.
   Örnek 10 ms'den yaşlıysa `imu_valid` ana döngüde zorla sıfırlanır.
4. ESKF public çıkışının son üretim zamanı kaydedilir. Nominal IMU propagasyonu
   1000 Hz, public ESKF çıkışı 200 Hz olarak kalır.
5. SystemMonitor IMU örnek yaşını, LiDAR örnek yaşını ve ESKF public çıkış yaşını
   kontrol eder. İlk 15 saniyelik başlangıç payından sonra hata kodları:

   - `13`: IMU verisi bayat/donmuş
   - `14`: LiDAR verisi bayat/donmuş
   - `15`: ESKF public çıkışı donmuş
   - `6`: izlenen scheduler görevlerinden biri ilerlemiyor

6. Ağır 15×15 kovaryans propagasyonu 50 Hz'den 25 Hz'ye indirilmiştir. ESKF
   public çıkış ve RCS kontrol hızı 200 Hz olarak korunur.

## Yeni UART alanları

V55 çerçevesi CRC dahil olmadan 101 alandır. P29 ile eklenen alanlar:

- `imu_age_ms`
- `eskf_public_age_ms`, `eskf_public_count`
- `lidar_timeouts`, `lidar_wait_timeouts`, `lidar_recoveries`
- `system_imu_fresh`, `system_lidar_fresh`, `system_eskf_fresh`
- `scheduler_realigns`

`imu_age_ms`, `lidar_age_ms` veya `eskf_public_age_ms` değeri `4294967295`
ise henüz geçerli bir zaman damgası üretilmemiştir.

## Güç kapalı masa testi

1. Projeyi STM32CubeIDE'de temiz derleyip karta yükleyin.
2. Solenoid/röle yük beslemesini, BTS7960 motor beslemesini ve basınç hattını
   kapalı bırakın; yalnız kontrol kartını besleyin.
3. USART2 PA2-TX ve GND üzerinden `115200 8N1` okuyun:

   `python monitor_uart_v55.py --port COMx`

4. PE9 GND'ye bağlıyken `actuator_authorized=0`, `rcs_applied_mask=0` ve iğne
   PWM'leri sıfır kalmalıdır.
5. Normal akışta yaşlar küçük ve `system_*_fresh=1` olmalıdır. LiDAR hattı kısa
   süre kesilip geri getirildiğinde `lidar_wait_timeouts` veya
   `lidar_timeouts`, ardından `lidar_recoveries` artmalı; yeni örnek gelince
   `lidar_age_ms` tekrar küçülmelidir.
6. Sensör hattı arızası sırasında aktüatör çıkışlarının güvenli kaldığını UART
   ve multimetre/lojik analizörle doğrulamadan yük gücü vermeyin.
