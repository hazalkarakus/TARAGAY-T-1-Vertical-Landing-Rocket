# TARAGAY-T1 P49 — ZUPT Joseph + Vertical Recovery + SD FIFO

P49, P48 bench/soak verisinde kalan üç uçuş-kritik problemi hedefleyen dar kapsamlı uçuş-adayı revizyondur.

## P48 verisinde bulunanlar

P48'in gravity attitude-only sparse Joseph değişikliği başarılı oldu:

- `grav_joseph_faults = 0`
- GRAV aşamasında covariance fault görülmedi.
- Public `z/vz` guard reject sayacı sıfır kaldı.
- Full ESKF reset yerine covariance-only rollback çalıştı.

Ancak yaklaşık 8.8–14.5 s başlangıç aralığında 52 covariance rollback görüldü. Tamamı:

- stage = `ZUPT` (3)
- state = `BGX` (12) veya `BGY` (13)

idi. Bu, klasik full-state ZUPT scalar covariance update'inin velocity residual'ı cross-covariance üzerinden gyro-bias varyansına taşıdığını gösterdi. Sistem zaten stationary durumda ayrı constrained gyro-bias update kullandığı için ZUPT'nin BG state'lerini ayrıca düzeltmesi gerekli değil.

İkinci uçuş-kritik problem: P48 `VerticalSensorPolicy`, canlı sağlık yerine kümülatif `numerical_error_count == 0` şartı kullanıyordu. Başarıyla recover edilmiş tek bir covariance fault bile bu sayacı kalıcı artırdığı için vertical source mask kalıcı olarak `0` oluyordu. Uçuş aktif olduğunda bu `SYS_FAULT_LIDAR_STALE`/vertical-aiding-lost fault'una ve RCS estimator fault zincirine dönüşebiliyordu.

Üçüncü problem SD tarafındaydı. P48 ring tarafı çok iyi çalıştı (`drop=0`, `overrun=0`), fakat 4 x 9216 B writer-buffer mimarisinde READY buffer seçimi buffer indeksine göre yapılıyordu. Uzun SD stall sırasında daha yeni düşük indeksli buffer daha eski yüksek indeksli buffer'dan önce yazılabiliyordu. P48 state CSV'sindeki tek boşluk tam 24 frame = 9216 B idi; bu bir writer buffer büyüklüğüyle birebir aynıydı. Decoder timestamp rollback gördüğünde eski preallocation tail sanıp dosyayı erken sonlandırabiliyordu.

## P49 değişiklikleri

### 1. ZUPT masked Joseph

ZUPT için yeni özel update yolu eklendi:

- H hâlâ velocity unit-state measurement.
- Kalman gain `PX..BAZ` (0..11) için korunur.
- `BGX/BGY/BGZ` gain'i zorla `0` yapılır.
- Gyro bias yalnız mevcut dedicated stationary gyro-bias update tarafından düzeltilir.
- 15x15 covariance upper triangle double precision Joseph formuyla güncellenir:

```
P+ = P - K p_k' - p_k K' + K S K'
```

- P46 state-preserving covariance rollback ve public output guard yedek koruma olarak aynen kalır.

Yeni diagnostikler:

- `zupt_joseph_update_count`
- `zupt_joseph_fault_count`

### 2. Vertical policy: historical hata sayacı canlı inhibit değildir

`numerical_error_count` P49'da vertical policy gate'inden çıkarıldı.

Canlı gate artık:

- ESKF enabled
- initialized
- healthy
- origin zeroed
- vertical position valid
- covariance integrity OK
- output not inhibited

şartlarına bakar.

Bu sayede başarıyla recover edilmiş eski bir covariance olayı uçuş boyunca LIDAR/BARO vertical aiding'i kalıcı olarak kapatmaz.

### 3. SD 4-buffer chronological FIFO

Her READY writer buffer'a monoton bir `ready_order` ticket verilir. DMA writer artık en düşük buffer indeksini değil, **en eski READY ticket'ı** seçer.

Yeni diagnostikler:

- `sd_logger_fifo_order_fault_count`
- `sd_logger_fifo_last_started_order`
- `sd_logger_fifo_next_ready_order`

P48'in şu iyileştirmeleri korunur:

- 128-frame CCM capture ring
- 4 x 9216 B SRAM writer buffer
- adaptive backpressure
- DATA-first, deferred guard
- 128 MiB preallocation / 96 MiB fallback

SD binary format değişmedi:

- V14
- 384 byte/frame
- 200 Hz state capture

### 4. UART

P49 UART:

- Prefix: `$TGY60`
- Banner: `TGY UART FLIGHT DIAGNOSTICS V60`
- Firmware: `8.19M-P49-ZUPT-JOSEPH-VERT-REC-SD-FIFO`
- 258 data field

P49 UART'a yeni ZUPT ve SD FIFO sayaçları eklendi.

## P49 uçuş-adayı kabul kriteri

P49 henüz yalnız kaynak/host doğrulaması geçmiş bir **flight candidate**'dır. Uçuş onayı için gerçek kart testleri gerekir.

İlk 5 dk bench'te:

```
eskf_cov_faults = 0
eskf_cov_rollbacks = 0
zupt_joseph_faults = 0
grav_joseph_faults = 0
eskf_public_rejects = 0

sd_dropped = 0
sd_ring_overruns = 0
sd_fifo_order_faults = 0

imu_recovery_failures = 0
uart_dma_errors = 0
```

Uçuş simülasyonunda PE9 sonrası, iki vertical sensör sağlıklıyken:

```
vertical_source_mask = 3
vertical_degraded = 0
system_fault = 0
rcs_fault = 0
```

Transient IMU recovery oluşursa safe inhibit kabul edilir; recovery sonrası ESKF/RCS'nin otomatik olarak sağlıklı duruma dönmesi gerekir.

Uzun soak sonunda SD CSV bütün test süresini kapsamalı; timestamp/sequence yön değiştirmemeli ve 24-frame / 9216-byte blok sıralama boşluğu olmamalıdır.

## Test sırası

1. CubeIDE Clean + Build: 0 error ve memory overflow yok.
2. Aktüatör/pnömatik enerji kapalı 5 dk bench.
3. 25–30 dk soak.
4. PE9 ile flight-state dry-run; vertical mask ve RCS recovery kontrolü.
5. LIDAR kes → BARO-only degraded devam.
6. BARO kes → LIDAR-only degraded devam.
7. İkisini kes → SAFE inhibit.
8. IMU recovery/stale fault injection → inhibit, recovery sonrası yeniden çalışma.
9. SD fault/latency davranışı → aktif uçuş kontrolü sırf recorder yüzünden ölmemeli.
10. STOP/PE9/physical output interlock tekrar testi.

Bu testler geçmeden P49 gerçek uçuş firmware'i olarak dondurulmamalıdır.
