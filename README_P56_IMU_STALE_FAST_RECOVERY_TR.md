# TARAGAY-T1 P56 — IMU STALE FAST RECOVERY

## Amaç

P55 repeated-word/config-loss olaylarını büyük ölçüde hızlı register repair ile çözmüş olsa da uzun bench testinde kalan full IMU recovery olayları `imu_stale_count` ile yakın ilerliyordu. P56 yalnızca bu stale hattını hedefler.

P56'da barometre, ESKF, kontrol, RCS, needle, scheduler ve SD V14 frame formatı değiştirilmemiştir.

## P56 akışı

İlk exact-six-axis stale tespitinde artık doğrudan full sensor reset başlatılmaz.

1. Stale örnek karantinaya alınır; ESKF/kontrole yayınlanmaz.
2. SPI transaction boundary soft-resync yapılır.
3. Bir sonraki 1 kHz IMU çağrısında WHO_AM_I + CTRL1_XL + CTRL2_G + CTRL3_C + CTRL4_C snapshot alınır.
4. WHO_AM_I geçersizse doğrudan mevcut full recovery safety-net'e escalation yapılır.
5. WHO_AM_I geçerli ve config sağlıklıysa register yazılmaz; soft-resync sonrası tek retry yapılır.
6. WHO_AM_I geçerli fakat config bozuksa yalnızca kritik registerlar geri yazılır ve hemen doğrulanır:
   - CTRL3_C = 0x44
   - CTRL4_C |= 0x04
   - CTRL1_XL = 0x8C
   - CTRL2_G = 0x88
7. ODR yeniden etkinleştirilmişse `HAL_Delay()` kullanılmadan yalnızca bir scheduler tick (~1 ms) beklenir.
8. Tek stale retry geçerli ve değişmiş örnek üretirse normal akış sürer.
9. Retry yine stale/invalid olursa veya DMA burst başarısız olursa mevcut bounded full recovery state machine devreye girer.

Bu nedenle P56 kısa yolu başarısız olduğunda P55/P36 güvenlik ağı kaldırılmış değildir.

## Yeni UART diagnostikleri

P56 UART frame: `$TGY67`

Toplam veri alanı: 321 (CRC ayrı)

Yeni alanlar:

- `imu_stale_fast_checks`
- `imu_stale_fast_attempts`
- `imu_stale_fast_success`
- `imu_stale_fast_failures`
- `imu_stale_retry_success`
- `imu_stale_recovery_escalations`
- `imu_stale_fast_last_us`
- `imu_stale_fast_max_us`
- `imu_stale_diag_valid`
- `imu_stale_diag_event_count`
- `imu_stale_diag_time_ms`
- `imu_stale_diag_age_us`
- `imu_stale_diag_reg_valid`
- `imu_stale_diag_whoami`
- `imu_stale_diag_ctrl1_xl`
- `imu_stale_diag_ctrl2_g`
- `imu_stale_diag_ctrl3_c`
- `imu_stale_diag_ctrl4_c`

P55 repeated-pattern diagnostikleri aynen korunmuştur.

## SD

SD binary wire format değiştirilmedi:

- `SDLOGGER_FORMAT_VERSION = 14`
- `SDLOGGER_FRAME_SIZE = 384`

Dolayısıyla P55/P54 V14 decoder zinciri kullanılmaya devam edebilir.

## Test

CubeIDE:

1. Project import et.
2. `Project > Clean...`
3. Build et.
4. Flash et.
5. SD kart takılı olsun.
6. UART bench monitörü:

```bash
python -u monitor_uart_p56.py --port COM21 --log uart_p56.txt
```

COM numarasını kendi sistemine göre değiştir.

### Önerilen test süresi

En az 20 dakika. Mümkünse 25–30 dakika.

- İlk 2 dakika: sistem sabit.
- 3–7 dakika: normal çok eksenli hareket/eğme/döndürme.
- Kalan süre: sistem açık; arada hareket ve sabit bekleme.
- IMU stale/pattern olduğunda manuel reset atma.

### P56 kabul kriterleri

İdeal:

- `imu_stale_count > 0` olduğunda `imu_stale_fast_checks` artmalı.
- Stale register kaybı varsa `imu_stale_fast_attempts` ve `imu_stale_fast_success` artmalı.
- `imu_stale_retry_success` mümkün olduğunca `imu_stale_count`'a yakın olmalı.
- `imu_stale_recovery_escalations` P55'teki stale kaynaklı full recovery sayısından ciddi düşük olmalı.
- `imu_recovery_count` belirgin azalmalı.
- `imu_recovery_failures = 0` hedeflenir.
- `miss_imu` mümkün olduğunca 0 olmalı.
- `eskf_div_count = 0` ve `eskf_cov_faults = 0` kalmalı.
- SD: `sd_write_errors = 0`, `sd_dropped = 0`, `sd_ring_overruns = 0`.

Özellikle son stale snapshot şu soruyu cevaplayacak:

`WHO_AM_I` hâlâ 0x6A/0x6B iken CTRL1/CTRL2/CTRL3 tekrar sıfırlanıyor mu, yoksa stale olayları registerlar sağlıklıyken mi oluşuyor?

Bu ayrım bir sonraki müdahalenin register-loss mu, SPI/data-path mi olacağını belirleyecek.

## Otomatik doğrulama

```bash
python tools/validate_p56.py
```

Beklenen:

`P56 validation: PASS ...`

Ayrıca `imu.c` ve `uart_telemetry.c` ARM Cortex-M4 hedefinde clang syntax kontrolünden geçirilmiştir. Full STM32CubeIDE link/build testi hedef bilgisayarda yapılmalıdır.
