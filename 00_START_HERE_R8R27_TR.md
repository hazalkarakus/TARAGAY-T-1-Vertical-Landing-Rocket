# P112R12R8R27 — 3 Pozlu IMU → Rocket Frame Kalibrasyonu + RCS Röle Bench

Bu revizyon **yalnızca basınçsız / gazsız bench testi** içindir. Uçuş sürümü değildir.

## Amaç

R8R26 iki poz (`DİK + +X`) kullandığı için kullanıcının +X yönündeki küçük hizalama hatasına fazla hassas kalabiliyordu. R8R27 üçüncü fiziksel referansı ekler:

1. **DİK** → rocket +Z
2. Roketin üstü fiziksel/CAD **+X** yönüne 12–20° → rocket +X
3. Roketin üstü fiziksel/CAD **+Y** yönüne 12–20° → rocket +Y doğrulaması

+Y, +X'in roket +Z ekseni etrafında 90° yanındaki sağ-el eksenidir (`X × Y = Z`).

## Matris kabul şartları

Kalibrasyon ancak aşağıdaki geometrik kontroller geçerse `cal_valid=1` olur:

- +X ve +Y tilt: 10–28° kabul penceresi
- ölçülen X/Y eksen açısı: 75–105°
- +Y işaret/sağ-el uyumu: `axis_agreement >= 0.90`
- determinant: 0.985–1.015
- ortonormallik hatası: <= 0.020

Yanlış yöndeki `-Y` pozu **kabul edilmez**; sistem `TILT_TOP_TO_PLUS_Y` aşamasında bekler.

## Güvenlik

- Sistem **0 bar / gazsız** olmalı.
- Mümkünse solenoid valf bobinlerini ayır; sadece röle kartını gözle.
- Ana iğne vana/motor fiziksel olarak çalışmamalı.
- Vent yolu kilitli kalır.
- PE9 ayrılması gerekmez.
- RCS röleleri yalnız `ready=1 + real ESKF valid + calibration valid + system_ok + E-STOP yok` iken bench API ile sürülebilir.
- Needle limiti değişmedi: **3 tur = 585 ADC**.

## Test

1. CubeIDE: `Clean -> Build -> Flash`.
2. UART monitor:

```bash
py monitor_uart_p112r12r8r27_three_pose_imu_rocket_frame_cal.py --port COM21 --duration 70
```

3. `HOLD_UPRIGHT`: roketi tam dik ve hareketsiz tut (~2 s kabul edilmiş örnek).
4. `TILT_TOP_TO_PLUS_X`: roketin üstünü fiziksel/CAD +X yönüne 12–20° eğ.
5. `HOLD_PLUS_X_TILT`: aynı pozu ~1.2 s sabit tut.
6. `TILT_TOP_TO_PLUS_Y`: önce tekrar dik konuma dön, sonra roketin üstünü fiziksel/CAD +Y yönüne 12–20° eğ.
7. `HOLD_PLUS_Y_TILT`: aynı pozu ~1.2 s sabit tut.
8. `CALIBRATED`: dik konuma dön. Monitor matrisi ve kalite değerlerini yazdırır.
9. Son doğrulama için yavaşça `+X -> 0 -> -X -> 0 -> +Y -> 0 -> -Y -> 0` yap; röleleri gözle.
10. Oluşan TXT'yi gönder.

## Fazlar

- 0: WAIT_SOURCE
- 1: HOLD_UPRIGHT
- 2: TILT_TOP_TO_PLUS_X
- 3: HOLD_PLUS_X_TILT
- 4: TILT_TOP_TO_PLUS_Y
- 5: HOLD_PLUS_Y_TILT
- 6: CALIBRATED
- 7: CAL_FAULT

## Ek UART alanları

R8R26 alanları korunur ve sona append-only olarak şunlar eklenir:

- `cal_y_tilt_samples`
- `cal_y_tilt_cdeg`
- `cal_xy_angle_cdeg`
- `cal_axis_agree_x10000`
- `cal_ortho_err_x10000`
- `cal_det_x10000`

## Sonraki adım

R8R27 ile tekrar edilebilir ve dört yön doğrulaması geçen matris elde edilirse sonraki revizyonda bu sabit `R_rocket_from_imu` firmware'e gömülür. Uçuş öncesi üç-poz kalibrasyon yapılmaz; yalnız IMU/kart mekanik montajı değişirse yeniden bench kalibrasyonu gerekir.
