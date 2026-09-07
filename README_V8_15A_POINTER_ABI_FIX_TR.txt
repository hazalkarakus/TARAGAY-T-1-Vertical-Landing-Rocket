TGY V8.15A - ESKF POINTER / ABI FIX
====================================

V8.15 temiz flash sonrasında da bozuk kaldı:
- 100 yerine 25601
- 20 yerine 5120
- 5 yerine 83886080
- float state ve CPU alanlarında imkansız değerler

Bu nedenle artık eski ELF/debug listesi varsayımı bırakıldı.

V8.15'te estimator açılınca hot-path'te büyük C struct'ları BY-VALUE
döndürülüyordu:

SensorManager_GetData()
AttitudeEstimator_GetData()
FullStateESKF_GetData()
Barometer_GetData()
Lidar_GetData()

Özellikle FullStateESKFData_t büyük ve:
- 1 kHz predict
- 1 kHz SD publish
- 200 Hz correction
- monitor

yollarında kopyalanıyordu.

V8.15A
------
Eski GetData() API'leri silinmedi.

Yeni const pointer API'leri:
SensorManager_GetDataPtr()
AttitudeEstimator_GetDataPtr()
FullStateESKF_GetDataPtr()
Barometer_GetDataPtr()
Lidar_GetDataPtr()

Aktif hot-path bu pointer API'lerine geçirildi:
- AttitudeEstimator_Update
- FullStateESKF_Predict
- FullStateESKF_CorrectMeasurements
- SDLogger_PublishSources
- V8.15 monitor

Böylece büyük return-by-value/stack/ABI kopyaları estimator yolundan çıkarıldı.

SHADOW MODE AYNI
----------------
1000 Hz nominal predict
200 Hz public output
200 Hz correction
50 Hz covariance

ESKF -> actuator bağlantısı YOK.

YENİ DIAGNOSTICS
----------------
v815a_magic_pre      beklenen 0x815A15A1
v815a_magic_post     beklenen 0x815A15A2
v815a_canary_ok      = 1

v815a_pointer_api_ok = 1
v815a_state_finite_ok= 1
v815a_shadow_core_ok = 1 hedef

Struct size diagnostikleri:
v815a_sensor_struct_size
v815a_attitude_struct_size
v815a_eskf_struct_size

İLK TEST
--------
Motor PSU kapalı.

Clean -> Build -> Debug -> Resume.
5-10 saniye hareketsiz bekle.

ÖNCE SADECE ŞUNLARI AÇ:
v815a_magic_pre
v815a_magic_post
v815a_canary_ok
v815a_pointer_api_ok
v815a_state_finite_ok

attitude_update_count
full_eskf_predict_count
full_eskf_public_output_count
full_eskf_covariance_predict_count

v815_attitude_delta_100ms
v815_eskf_predict_delta_100ms
v815_eskf_public_delta_100ms
v815_eskf_covariance_delta_100ms
v815_eskf_correction_delta_100ms

v815_eskf_initialized
v815_eskf_healthy
v815_eskf_shadow_mode
v815_eskf_rate_ok
v815_eskf_shadow_core_ok

cpu_load_percent
cpu_idle_percent

BEKLENEN
--------
delta /100 ms:
attitude   ~100
predict    ~100
public     ~20
covariance ~5
correction ~20

canary_ok       = 1
pointer_api_ok   = 1
state_finite_ok  = 1
rate_ok          = 1

Eğer bu sürümde canary bozulursa veya CPU/global değerler yine saçmalarsa,
bir sonraki adım return-by-value değil doğrudan SRAM/stack collision analizi
olacak; SD ring / estimator memory bank placement ayrılacak.
