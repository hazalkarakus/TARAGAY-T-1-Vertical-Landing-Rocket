TGY V8.19M SMALL BOARD V23 FULL SYSTEM

Amaç:
- V22'de çalışan IMU, BMP585 ve SD yolunu korumak.
- TGY_ALL_SENSORS_V20 küçük-board davranışındaki IMU -> RCS röle kontrolünü geri getirmek.
- NRF -> tahliye servo yolunu sürekli main-loop servis etmek.
- Needle valve / ana motor 10-cycle endurance sistemini aynen korumak.
- V20 LIDAR sürücüsünü 5 ms servis periyodunda çalıştırmak.

Önemli davranışlar:
- RCS röleleri active-low: PE7, PE11, PE15, PB15.
- Fiziksel RCS çıkışı V23'te aktif (ATT_CTRL_DRY_RUN=0).
- RCS legacy attitude controller 2 s startup delay sonrası çalışır.
- Absolute no-fire bölgesi ±5 derecedir; küçük hareketlerde röle açmaması normaldir.
- NRF link kaybında tahliye servosu CLOSED olur.
- Needle auto-control/10-cycle profile korunmuştur ve PA0 davranışı değişmemiştir.
- GNCActiveControl'un disarmed servisleri çağrılmıyor; aksi halde RCS relays sürekli safe konumuna zorlanıyordu.

İlk Live Expressions:
  v23_rcs_direct_enabled
  v23_rcs_estimator_healthy
  v23_rcs_requested_mask
  v23_rcs_applied_mask
  v23_rcs_fault
  v23_rcs_control_update_count
  v23_rcs_output_update_count
  v89_nrf_connected
  v89_remote_link_active
  v810_servo_pulse_us
  v87_lidar_connected
  v87_baro_connected

Beklenti:
- v23_rcs_direct_enabled = 1
- v23_rcs_estimator_healthy = 1
- 2 saniye startup sonrası kartı ±5 dereceden fazla eğince requested/applied mask değişebilir.
