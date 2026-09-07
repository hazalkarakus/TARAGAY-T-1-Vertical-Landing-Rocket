# P112R12R8R28 — 5 Poz IMU → Roket Frame Kalibrasyonu + RCS Röle Bench

Bu sürüm **uçuş sürümü değildir**. Kalibrasyon ve eksen/röle doğrulaması içindir.
Sistem **0 bar / gazsız / basınçsız** olmalıdır. Mümkünse valf bobinlerini ayırıp yalnız röle LED/kliklerini gözle.
Needle/motor fiziksel olarak yetkisizdir ve hareket etmemelidir.

## Neden R8R28?
R8R27 `DİK + +X + +Y` kullanıyordu. R8R28, insanın eğim açısı ve küçük poz hatalarını azaltmak için simetrik çiftler kullanır:

1. DİK
2. +X_ROCKET
3. -X_ROCKET
4. +Y_ROCKET
5. -Y_ROCKET

Firmware +X/-X ve +Y/-Y çiftlerini fark alarak eksenleri çıkarır. Böylece iki tarafta tam aynı derece eğmemiş olsan bile ortak hata büyük ölçüde iptal edilir.

## Test
1. CubeIDE: Clean → Build → Flash.
2. UART monitor:

```bash
py monitor_uart_p112r12r8r28_five_pose_imu_rocket_frame_cal.py --port COM21 --duration 80
```

3. Ekrandaki sırayı aynen takip et:
   - `HOLD_UPRIGHT`: roketi dik ve sabit tut (~2 s)
   - `TILT_TOP_TO_PLUS_X`: roketin üstünü fiziksel/CAD `+X_ROCKET` yönüne 12–20° eğ
   - `HOLD_PLUS_X_TILT`: sabit tut (~1.2 s)
   - `TILT_TOP_TO_MINUS_X`: önce dike dön, sonra `-X_ROCKET` yönüne 12–20° eğ
   - `HOLD_MINUS_X_TILT`: sabit tut
   - `TILT_TOP_TO_PLUS_Y`: önce dike dön, sonra `+Y_ROCKET` yönüne 12–20° eğ
   - `HOLD_PLUS_Y_TILT`: sabit tut
   - `TILT_TOP_TO_MINUS_Y`: önce dike dön, sonra `-Y_ROCKET` yönüne 12–20° eğ
   - `HOLD_MINUS_Y_TILT`: sabit tut
   - `CALIBRATED`: kalibrasyon tamam.

**Sensör kartı üzerindeki X/Y oklarını referans alma. Roket gövdesindeki fiziksel/CAD eksenleri kullan.**

## Kabul kontrolleri
- X +/- opposition >= 0.94
- Y +/- opposition >= 0.94
- X-Y açısı 80–100°
- handedness/axis agreement >= 0.95
- pair-derived Z ile upright Z agreement >= 0.97
- determinant 0.985–1.015
- ortogonallik hatası <= 0.020

Yanlış işaretli -X veya -Y pozu capture edilmez. Kalibrasyon geçmeden RCS röle bench çıkışı yetkilendirilmez.

## Kalibrasyon sonrası
`CALIBRATED` geldikten sonra dike dön ve kısa şekilde `+X → 0 → -X → 0 → +Y → 0 → -Y → 0` yap. Röle LED/kliklerini gözle. TXT dosyasını gönder.

Bu beş-poz test iki ayrı çalıştırmada tutarlı çıkınca ölçülen sabit matris bir sonraki revizyonda firmware'e gömülecek; uçuş öncesinde bu poz kalibrasyonu yapılmayacak.
