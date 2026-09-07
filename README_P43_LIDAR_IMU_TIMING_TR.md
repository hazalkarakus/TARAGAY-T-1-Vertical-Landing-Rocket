# P43 — LiDAR Best-Effort Scheduler / IMU Deferred Quick-Retry

P43, doğrulanmış P42 scheduler rollback sürümünün dar kapsamlı devamıdır.
ESKF matematiği, public/correction 200 Hz yolu, görev öncelik sırası, LiDAR
sürücüsü, barometre, SD, PE9 interlock ve aktüatör güvenliği değiştirilmemiştir.

## 1. LiDAR scheduler politikası

- LiDAR görevi 1 kHz fiziksel ölçüm değildir; DMA tabanlı durum makinesine
  yapılan **best-effort 1 kHz servis çağrısıdır**.
- IMU koruması nedeniyle ertelenen bir LiDAR release'i yalnız bir IMU nesli
  taşınabilir.
- Taşınan release sonraki IMU'dan sonra çalışırsa bir kez servis edilir ve
  gerçek zamana yeniden bağlanır.
- İkinci IMU neslinde de yeterli zaman yoksa release atlanır ve yeniden bağlanır;
  tek bir carry bayrağı sınırsız gecikmeyi gizleyemez.
- LiDAR best-effort atlaması sert `deadline_miss_count` üretmez. Sert deadline
  sayaçları IMU/ESKF/barometre gibi gerçek periyodik görevler için korunur.

Live Expressions için yeni sayaçlar:

- `lidar_scheduler_carry_count`
- `lidar_scheduler_carry_served_count`
- `lidar_scheduler_best_effort_skip_count`

## 2. IMU pattern quick-retry

P40–P42'de repeated-word pattern görülünce ikinci üçlü burst aynı task çağrısı
içinde hemen okunuyordu. IMU 1.66 kHz ODR ile çalıştığı için register görüntüsü
henüz yenilenmeden yapılan bu okuma aynı hatayı tekrar yakalayabiliyor ve IMU
task'ının o çağrıdaki yükünü ikiye çıkarıyordu.

P43'te:

- bozuk triplet yayımlanmaz,
- `imu_pattern_retry_count` artırılır,
- tekrar okuma bir sonraki 1 ms IMU task'ına bırakılır,
- yeni triplet geçerliyse `imu_pattern_retry_success_count` artırılır,
- sonraki slot da bozuksa mevcut non-blocking IMU recovery başlatılır.

UART alan sırası ve sayısı değişmedi: **214 alan**. Teslimdeki
`monitor_uart_p43.py` ve `decode_flight_v14_p43.py` kullanılmalıdır; içerikleri
protokol uyumluluğu için doğrulanmış P42 araçlarıyla aynıdır.

## 3. İlk test

Gaz, basınç, motor ve solenoid güçleri kapalıyken en az 10 dakika çalıştırın.

Beklenenler:

- başlangıç satırı: `FW 8.19M-P43-LIDAR-IMU-TIMING`
- `miss_imu=0`, `miss_eskf=0`, `miss_baro=0`
- `miss_lidar=0` (best-effort servis sert deadline değildir)
- ESKF public/correction yaklaşık 200 Hz
- LiDAR fiziksel ölçüm yaklaşık 170–200 Hz
- SD state yaklaşık 200 Hz; dropped/overrun 0
- `imu_pattern_retry_success <= imu_pattern_retries`
- pattern tekrarı olursa başarı sayacı artık artabilmeli; iki ardışık hata yine
  non-blocking recovery ile güvenli biçimde ele alınmalı
- herhangi bir fault varken fiziksel RCS maskesi 0 kalmalı
