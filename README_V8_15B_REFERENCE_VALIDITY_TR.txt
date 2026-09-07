TGY V8.15B - BARO FRESHNESS + POSITION VALIDITY + STARTUP ORIGIN
===================================================================

TABAN
-----
V8.15A pointer/ABI düzeltmesi KORUNDU.

Doğrulanan estimator timing:
IMU / nominal predict     1000 Hz
public state               200 Hz
correction                 200 Hz
covariance                  50 Hz

Bu sürüm ESKF matematik/tuning değerlerini değiştirmez.
Yalnız reference/validity davranışını temizler.

1) BARO CONNECTED + FRESHNESS GATE
----------------------------------
Önceki problem:
Barometre fiziksel olarak yokken full_eskf_baro_reference_ready = 1
görülebiliyordu.

V8.15B baroyu ancak şu koşulların TAMAMI doğruysa kullanılabilir kabul eder:

initialized
connected
data_ready
healthy
pressure_valid
calibrated
filter_initialized
last_sample_timestamp_us != 0
sample age <= 50 ms

Yeni:
full_eskf_baro_fresh
full_eskf_baro_sample_age_us

Baro yok / stale:
full_eskf_baro_fresh = 0
full_eskf_baro_reference_ready = 0

ÖNEMLİ:
Daha önce başarılı şekilde alınmış baro reference bellekte korunur.
Kısa disconnect sonrası baro geri gelirse altitude origin yeniden sıfırlanmaz.
Sadece "current usable/ready" flag tekrar 1 olur.

Cold boot'ta baro yoksa reference hiçbir zaman oluşmaz.

2) LIDAR FRESHNESS
------------------
Aynı prensip LIDAR için de ayrı validity hesabına eklendi:

full_eskf_lidar_fresh
full_eskf_lidar_sample_age_us

fresh için:
initialized
connected
data_ready
distance_valid
filter_initialized
sample age <= 50 ms

3) X/Y POSITION VALIDITY
------------------------
Şu anda estimator'da:
GPS / optical flow / UWB / vision / başka absolute horizontal position aid YOK.

Bu nedenle:
full_eskf_horizontal_position_valid = 0

HER ZAMAN.

X/Y inertial state yine propagate edilir ve loglanır.
Ama GNC gelecekte bu flag 0 iken X/Y position'ı absolute konum diye
KULLANMAMALIDIR.

Z validity:
full_eskf_vertical_position_valid

şunlardan en az biri fresh + referenced ise 1:
- LIDAR
- barometer

Bench'te baro yokken fresh LIDAR yeterlidir.

4) STARTUP ORIGIN ZERO
----------------------
İlk startup sırasında inertial integration gyro/bootstrap tamamlanana kadar
bir miktar X/Y/Z position residue üretebiliyordu.

V8.15B yalnız BİR KEZ coordinate origin'i sıfırlar.

Şartlar:
gyro bootstrap done
stationary_detected = 1
continuous stationary >= 200 correction sample ~= 1 s
LIDAR reference ready = 1
LIDAR fresh = 1

O anda:
nominal position XYZ -> 0
nominal velocity XYZ -> 0

Covariance SIFIRLANMAZ.
Bu bir measurement correction değil, coordinate-frame translation'dır.

Yeni:
full_eskf_origin_zeroed
full_eskf_origin_zero_count
full_eskf_origin_zero_timestamp_us

Sıfırlama öncesi residue:
full_eskf_origin_pre_position_x_m
full_eskf_origin_pre_position_y_m
full_eskf_origin_pre_position_z_m

Beklenen:
origin_zero_count = 1
ve bir daha artmamalı.

5) SD V13 DEĞİŞMEDİ
-------------------
Frame hala V13 / 288 byte.

full_eskf_attitude_flags içindeki daha önce boş olan bitler kullanıldı:

bit0 enabled
bit1 initialized
bit2 healthy
bit3 gravity correction
bit4 stationary
bit5 horizontal_position_valid
bit6 vertical_position_valid
bit7 origin_zeroed

CRC / frame size değişmedi.

İLK TEST - BARO TAKILI DEĞİL
----------------------------
Motor PSU kapalı.
Sistem 5-10 saniye hareketsiz.

Live Expressions:

full_eskf_enabled
full_eskf_shadow_mode
full_eskf_initialized
full_eskf_healthy

full_eskf_baro_fresh
full_eskf_baro_reference_ready
full_eskf_baro_sample_age_us

full_eskf_lidar_fresh
full_eskf_lidar_reference_ready
full_eskf_lidar_sample_age_us

full_eskf_horizontal_position_valid
full_eskf_vertical_position_valid

full_eskf_origin_zeroed
full_eskf_origin_zero_count
full_eskf_origin_pre_position_x_m
full_eskf_origin_pre_position_y_m
full_eskf_origin_pre_position_z_m

full_eskf_position_x_m
full_eskf_position_y_m
full_eskf_position_z_m

full_eskf_velocity_x_mps
full_eskf_velocity_y_mps
full_eskf_velocity_z_mps

full_eskf_numerical_error_count

BEKLENEN - BARO YOK
-------------------
baro_fresh           = 0
baro_reference_ready = 0

lidar_fresh           = 1
lidar_reference_ready = 1

horizontal_position_valid = 0
vertical_position_valid   = 1

origin_zeroed     = 1
origin_zero_count = 1

Origin olayından sonra hareketsiz:
position XYZ başlangıçta ~0
velocity XYZ ~0

X/Y zamanla yavaş drift edebilir.
Bu yüzden horizontal_position_valid = 0 kalır.

BARO TAKILINCA
--------------
Baro sağlıklı ve fresh olduğunda:
baro_fresh = 1

İlk cold reference 8 yeni valid sample sonrası:
baro_reference_ready = 1

Baro çıkarılırsa <=50 ms içinde:
baro_fresh = 0
baro_reference_ready = 0

Takılırsa mevcut ESKF session'daki eski reference korunarak ready tekrar 1 olur.

PASS
----
baro yokken fake reference yok
LIDAR vertical validity = 1
horizontal validity = 0
origin exactly once
numerical_error_count = 0

Estimator output henüz hiçbir actuator'a bağlı değildir.
