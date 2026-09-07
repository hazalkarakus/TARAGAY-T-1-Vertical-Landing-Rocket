# TARAGAY-T1 P51 — Staggered Stationary Maintenance + Flight-Safe ZUPT Gate

P50 bench test özeti:
- 337.315 s / 67,464 state frame / tam 200.00 Hz
- BIN CRC error = 0, sequence gap = 0
- ESKF covariance fault = 0
- gravity Joseph fault = 0
- ZUPT Joseph fault = 0
- SD dropped/overrun/FIFO-order fault = 0
- IMU recovery 19/19 başarılı
- Ancak scheduler: miss_imu=72, miss_baro=3, miss_eskf=1, realign=76
- 72 IMU miss'in tamamı UART örneklemesinde ZUPT update artışı bulunan aralıklarda oluştu.
- P50 ESKF task max = 1770 us.

P51 değişiklikleri:

1. Stationary maintenance artık non-blocking / staggered.
   Eski davranış 10 Hz tetikte tek 200 Hz ESKF çağrısında:
     3x ZUPT + 3x gyro-bias + opsiyonel 3x accel-bias
   çalıştırıyordu.

   P51 aynı batch'i 200 Hz çağrılar arasında fazlara böler:
     ZUPT X -> ZUPT Y -> ZUPT Z ->
     BG X -> BG Y -> BG Z ->
     BA X -> BA Y -> BA Z

   Her correction çağrısında en fazla bir stationary scalar correction yapılır.
   Batch başlangıç periyodu yine yaklaşık 10 Hz'dir; estimator ownership/değerleri
   değiştirilmez, yalnız cooperative scheduler burst'ü kaldırılır.

2. Flight-safe stationary gate eklendi.
   Startup origin oluşmadan önce eski bootstrap davranışı korunur.
   Origin oluştuktan sonra stationary detector ayrıca:
     |vz| <= 0.15 m/s
   ister ve eşik aşılırsa stationary/ZUPT fazını beklemeden anında bırakır.

   Amaç: düzgün/az açısal hareketli powered descent sırasında accel norm ~1g
   olabildiği için aracı yanlışlıkla stationary sayıp ZUPT ile uçuş hızını
   sıfıra çekmeyi engellemek.

3. P50 güvenlikleri aynen korunur:
   - gravity attitude-only sparse Joseph
   - masked fast ZUPT Joseph / BGxBG korunumu
   - covariance state-preserving rollback
   - public z/vz guard
   - P49 vertical recovery
   - P49 SD FIFO ordering
   - SD backpressure / 4 writer buffer

UART:
- $TGY62
- V62
- 258 field
- SD V14 format değişmedi.

İlk bench hedefleri:
- eskf_cov_faults = 0
- grav_joseph_faults = 0
- zupt_joseph_faults = 0
- sd_dropped = 0
- sd_ring_overruns = 0
- sd_fifo_order_faults = 0
- imu_recovery_failures = 0
- miss_imu belirgin şekilde P50'nin 72 değerinin altında
- scheduler_realigns belirgin şekilde P50'nin 76 değerinin altında
- task_eskf_max_us tercihen < 1000 us

Not: Bu bir uçuş-adayı yazılım revizyonudur; gerçek uçuş onayı için ayrıca
uzun soak, sensör fault injection, PE9/STOP interlock ve enerjili-gazsız
aktüatör dry-run testleri gerekir.
