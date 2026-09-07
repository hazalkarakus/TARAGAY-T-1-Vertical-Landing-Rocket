# TARAGAY-T1 P57 — IMU Stale STATUS_REG Warmup

## Neden P57?
P56 testinde 84 stale olayının 84'ünde de WHO_AM_I geçerli kalırken CTRL1_XL/CTRL2_G/CTRL3_C sıfırlanmıştı. Register-only repair 84/84 başarılıydı; ancak P56 sensörü sadece yaklaşık 1 ms sonra yeniden örneklediği için yalnız 13/84 olay fresh sample ile kurtuldu ve 71 olay full reset'e eskale oldu.

P57 bu tek-tick varsayımını kaldırır. Register repair başarılı olduğunda sensör resetlenmez; 1 kHz görev her çağrıda yalnız bir STATUS_REG okur ve hem XLDA hem GDA hazır olana kadar non-blocking warmup durumunda kalır.

## P57 akışı
1. Exact-six-axis stale (20 ms) tespit edilir; frozen sample flight stack'e verilmez.
2. SPI soft-resync yapılır.
3. WHO_AM_I + CTRL1/2/3/4 snapshot alınır.
4. WHO_AM_I geçerli ve config bozuksa P55/P56 register-only repair uygulanır ve readback doğrulanır.
5. Repair başarılıysa `imu_stale_warmup_active=1` olur.
6. Her 1 kHz çağrıda bir kez STATUS_REG okunur.
7. `(STATUS_REG & 0x03) == 0x03` (XLDA + GDA) olduğunda 3-burst validated sample denenir.
8. Raw hâlâ değişmemişse timeout dolana kadar sonraki scheduler tick'lerinde tekrar denenebilir.
9. Fresh validated triplet gelirse warmup başarıyla biter; full reset yoktur.
10. 40 ms timeout, WHO_AM_I kaybı, DMA/bus hatası veya repeated-pattern corruption olursa mevcut full recovery safety-net devreye girer.

Bu kısa yolun hiçbir yerinde `HAL_Delay()` veya busy-wait yoktur.

## Yeni UART
- Prefix: `$TGY68`
- Banner: V68
- Data field: 329 (+ CRC16)

Yeni P57 alanları:
- `imu_stale_warmup_events`
- `imu_stale_warmup_polls`
- `imu_stale_warmup_success`
- `imu_stale_warmup_timeouts`
- `imu_stale_warmup_first_ready_us`
- `imu_stale_warmup_max_ready_us`
- `imu_stale_warmup_last_status`
- `imu_stale_warmup_active`

`imu_stale_warmup_first_ready_us` son warmup olayında XLDA+GDA'nın ilk birlikte görüldüğü süreyi, `max_ready_us` ise test boyunca en kötü ölçülen değeri gösterir. Böylece IMU'nun gerçek restart/ODR-ready süresini tahmin etmek yerine ölçeriz.

## Değişmeyen güvenlik katmanları
- P55 repeated-pattern fast config repair aynen korunur.
- P56 stale pre-write register snapshot aynen korunur.
- Full IMU software-reset recovery aynen korunur.
- ESKF, barometre, lidar, scheduler, RCS, main control ve needle control değiştirilmedi.
- SD formatı değişmedi: V14, 384 byte.

## Test
CubeIDE:
1. Project > Clean
2. Build
3. Flash
4. Kartı power-cycle et
5. SD kart takılı olsun

UART:
```bash
python -u monitor_uart_p57.py --port COM21 --log uart_p57.txt
```

10–15 dakika yeterli ilk kabul testi. İlk 1–2 dakika sabit bırak, sonra birkaç dakika farklı eksenlerde hareket ettir, ardından soak bırak. Pattern/stale olduğunda reset atma.

### Ana kabul kriteri
P56'ya göre full recovery sayısının dramatik düşmesi gerekir:
- `warm success / warm events >= %95` hedef
- `warm timeouts ~= 0`
- `stale_recovery_escalations` çok düşük
- `imu_recovery_count` stale sayısını takip ETMEMELİ
- `imu_recovery_failures = 0`
- `miss_imu ~= 0`
- P55 pattern fast repair başarı oranı yüksek kalmalı
- ESKF divergence/covariance fault = 0
- SD write/drop/overrun = 0

Özellikle `readyUs=last/max` değerini kaydet. Eğer warmup timeout oluşursa bu sayı ve `status` P58 gerekip gerekmediğini belirleyecek.

## Sonraki kapı
P57 bu kriterleri sağlarsa IMU üzerinde yeni özellik eklemeyi bırakıp doğrudan NRF bench/link testi ve ardından aktüatör/needle + solenoid authorization testlerine geçebiliriz.
