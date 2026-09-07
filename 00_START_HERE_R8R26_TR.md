# P112R12R8R26 — IMU → Rocket Frame Calibration + RCS Relay Bench

Bu revizyon **yalnızca basınçsız / gazsız bench testi** içindir. Uçuş sürümü değildir.

## Amaç

IMU/ESKF gövde eksenleri roketin fiziksel X/Y/Z eksenleriyle tam hizalı olmadığı için R8R26 sabit montaj dönüşümünü iki statik pozdan öğrenir:

1. Roket tam dik ve sabit.
2. Roketin üst kısmı fiziksel/CAD **+X** yönüne 12–20 derece eğik ve sabit.

Bu iki pozdan `R_rocket_from_imu` 3x3 dönüşüm matrisi hesaplanır. Sonra ESKF quaternionundan elde edilen UP vektörü ve bias-düzeltilmiş gyro vektörü bu matrisle rocket-frame'e çevrilir. RCS V7.13.4 bundan sonra düzeltilmiş iki lateral tilt eksenini görür.

## Güvenlik

- Sistem **0 bar / gazsız** olmalı.
- Mümkünse solenoid valf bobinlerini ayır; sadece röle kartını gözle.
- Ana iğne vana/motor fiziksel olarak çalışmamalı.
- Vent yolu kilitli kalır.
- PE9 ayrılması gerekmez.
- RCS röleleri yalnız `ready=1 + real ESKF valid + calibration valid + system_ok + E-STOP yok` koşullarında bench API üzerinden fiziksel olarak sürülebilir.

## Test

1. CubeIDE: `Clean -> Build -> Flash`.
2. UART monitor:

```bash
py monitor_uart_p112r12r8r26_imu_rocket_frame_cal.py --port COM21 --duration 45
```

3. `CAL PHASE = HOLD_UPRIGHT` görünürken roketi **tam dik** ve hareketsiz tut. Yaklaşık 2 s boyunca 200 kabul edilmiş örnek toplanır.
4. `CAL PHASE = TILT_TOP_TO_PLUS_X` görünce roketin **üst kısmını fiziksel/CAD +X yönüne** yaklaşık 12–20° eğ.
5. `HOLD_PLUS_X_TILT` görünce aynı açıyı yaklaşık 1.2 s sabit tut.
6. `CALIBRATED` görünce dik konuma dön. Bu andan sonra RCS röleleri algoritma isteğine göre tıklayabilir.
7. Sonra yavaşça +X, -X, +Y, -Y eğimlerini yap ve hangi IN rölesinin çektiğini not et.
8. Oluşan `uart_p112r12r8r26_imu_rocket_frame_cal.txt` dosyasını gönder.

## UART calibration alanları

- `cal_phase`: 0 wait source, 1 upright capture, 2 wait +X tilt, 3 tilted capture, 4 calibrated, 5 fault
- `cal_valid`
- `cal_fault`
- `cal_upright_samples`
- `cal_tilt_samples`
- `cal_tilt_cdeg`
- `cal_r00_x10000 ... cal_r22_x10000`

## Sonraki adım

R8R26 her açılışta bench kalibrasyonu yapar. Bu logdan gerçek montaj matrisi doğrulandıktan sonra bir sonraki revizyonda matrisi sabit firmware kalibrasyonu olarak kilitlemek hedeflenir; uçuşta bu iki-poz prosedürüne ihtiyaç bırakılmaz.

## Korunan kararlar

- Hover V19.6 kaynak otoritesi korunur.
- RCS V7.13.4 korunur.
- Legacy landing/GeneratedFlightControl landing otoritesi retired kalır.
- Needle maksimum travel: 3 tur = 585 ADC.
- Horizontal position `hpos_valid=0` iken position target gate korunur.
