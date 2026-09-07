# TARAGAY-T1 P55 — IMU Fast Configuration Repair

P55 yalnızca P54'te register seviyesinde kanıtlanan IMU repeated-word/config kaybı problemine odaklanır. P54 barometre rate guard, ground-reference takibi, SD V14 logging, ESKF korumaları, scheduler ve mevcut tam IMU recovery güvenlik ağı aynen korunur.

## P55'te değişen akış

Repeated-word triplet ilk kez yakalandığında:

1. Bozuk triplet karantinaya alınır; uçuş stack'ine verilmez.
2. P44'ten gelen soft SPI resync uygulanır.
3. Bir sonraki 1 kHz IMU servisinde, deferred retry'dan önce şu registerlar okunur ve **pre-repair snapshot** olarak saklanır:
   - WHO_AM_I
   - CTRL1_XL
   - CTRL2_G
   - CTRL3_C
   - CTRL4_C
4. WHO_AM_I geçerliyse fakat kritik config bozuksa yalnızca hızlı register onarımı yapılır:
   - CTRL3_C = 0x44
   - CTRL4_C |= 0x04
   - CTRL1_XL = 0x8C
   - CTRL2_G = 0x88
5. Aynı registerlar tekrar okunup doğrulanır.
6. Doğrulama başarılıysa sensör resetlenmez; mevcut tek deferred 3-burst retry hemen devam eder.
7. WHO_AM_I geçersizse veya repair verify başarısızsa mevcut bounded full-recovery state machine'e doğrudan escalate edilir.

Bu kısa yol `HAL_Delay()` kullanmaz ve full reset güvenlik ağını kaldırmaz.

## Yeni diagnostikler

UART/Live Expressions için:

- `imu_fast_config_check_count`
- `imu_fast_config_repair_attempt_count`
- `imu_fast_config_repair_success_count`
- `imu_fast_config_repair_failure_count`
- `imu_fast_config_repair_last_duration_us`
- `imu_fast_config_repair_max_duration_us`

UART alan adları:

- `imu_fast_cfg_checks`
- `imu_fast_cfg_attempts`
- `imu_fast_cfg_success`
- `imu_fast_cfg_failures`
- `imu_fast_cfg_last_us`
- `imu_fast_cfg_max_us`

`imu_pat_whoami/ctrl*` alanları **repair öncesi snapshot** göstermeye devam eder. Böylece örneğin `CTRL3_C=0x00` görülüp aynı anda `imu_fast_cfg_success` artarsa, P55'in config kaybını full reset yapmadan düzelttiği doğrudan kanıtlanır.

## Wire format

- UART: `$TGY66`, V66, 303 field + CRC16
- SD: V14 / 384 byte / 200 Hz — format değişmedi

UART sadece bench/test diagnostiktir; IMU güvenliği, ESKF, preflight/actuator authorization ve SD logging UART bağlantısına bağlı değildir.

## İlk P55 bench testi

1. Aktüatör/pnömatik enerji kapalı.
2. PE9 kapalı.
3. Kartı 10 dakika açık bırak.
4. UART:
   `python -u monitor_uart_p55.py --port COM8`
5. Pattern oluştuğunda şu üç sınıfa bak:
   - `regs=`: pre-repair register snapshot
   - `fastCfg=success/attempts`
   - `IMUrec=` ve `cfgFail=`
6. Beklenen ana sonuç:
   - repeated pattern oluşabilir,
   - config kaybı varsa `imu_fast_cfg_attempts` ve `imu_fast_cfg_success` artar,
   - `imu_fast_cfg_failures = 0` hedeflenir,
   - `imu_recovery_count` P54'e göre ciddi düşmelidir,
   - `imu_recovery_failures = 0` kalmalıdır,
   - `miss_imu` mümkün olduğunca 0'a yakın kalmalıdır,
   - `imu_fast_cfg_max_us` birkaç ms sınırının altında kalmalıdır.

Örnek kabul hedefi: 10 dakikada 100 repeated-pattern görülürse en az 95 olayın full recovery'ye gitmeden soft-resync + fast config repair/deferred retry hattında toparlanması.

## Önemli not

P55 repeated-word olayının kendisini "valid sample" saymaz. İlk bozuk sample yine karantinaya alınır. Ama config kaybı register yazımıyla hızlıca düzeltilebiliyorsa eski yaklaşık 40–50 ms reset/settle boşluğuna girmeden bir sonraki örnekleme çevriminde geri dönmeye çalışır.
