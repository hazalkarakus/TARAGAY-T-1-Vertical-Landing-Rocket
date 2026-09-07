# P44 — SD kapasitesi, 25 Hz kovaryans ve IMU SPI yeniden senkronizasyonu

P44, P43 test kayıtlarında doğrulanan üç dar kapsamlı sorunu düzeltir. PE9
interlock, RCS pin/polarite eşlemesi, motor güvenlik kilidi, 200 Hz ESKF public
çıkışı ve LiDAR best-effort carry/skip mantığı değiştirilmemiştir.

## Değişiklikler

- SD ön ayırma isteği 64 MiB, asgari fallback 48 MiB oldu. V14 384 bayt x
  200 Hz akışta 48 MiB yaklaşık 10,9 dakika, 64 MiB yaklaşık 14,6 dakika taşır.
- 25 Hz kovaryans görevi, her 40 ms'deki çakışmada best-effort LiDAR'dan önce
  çalışır. Kabul rezervi P43'te ölçülen 450 us maksimum süreye karşı 500 us ve
  ayrıca mevcut 40 us IMU guard olarak ayarlanmıştır.
- İlk repeated-word IMU hatasında CS/SCK/MOSI yazılım-SPI sınırı, sensör reseti
  ve HAL_Delay olmadan yeniden senkronize edilir. Doğrulama okuması yine bir
  sonraki 1 ms IMU görevinde yapılır; ikinci hata mevcut tam non-blocking
  recovery akışına gider.

## Protokoller

- UART: `$TGY55`, 214 alan, 115200 baud — değişmedi.
- SD: V14, 384 bayt state frame @ 200 Hz — değişmedi.
- Başlangıç sürüm satırı: `FW 8.19M-P44-SD-COV-IMU-RESYNC`

## İlk gazsız test

1. Solenoid, motor ve basınç gücünü kapalı tut.
2. CubeIDE'de Clean Project ve Build Project yap, ardından karta yükle.
3. PE9 bağlı başlat; `state=2` görülmeden ayırma.
4. PE9 ayrıldıktan sonra sistemi en az 10 dakika 30 saniye açık bırak.
5. UART ve `flight.bin` dosyasını P44 Python araçlarıyla kaydet/çöz.

## PASS ölçütleri

- `sd_logging=1`, `sd_last_result=0` ve `system_fault!=17` test sonuna kadar.
- `cov_count` uçuşa hazır olduktan sonra yaklaşık 25/s artmalı.
- `miss_imu=0`; LiDAR hard miss yine 0 kalmalı.
- IMU pattern hatası oluşursa `imu_pattern_retry_success` artışı veya recovery
  sayısında belirgin azalma beklenir. Retry başarısı yine 0 kalırsa sonraki adım
  yazılım zamanlaması değil PA4/PA5/PA6/PA7 sinyal bütünlüğü ölçümüdür.
- Herhangi bir hızlı/sistem hatasında RCS ve motor çıkışları sıfır kalmalı.
