# P112R12R8R29 — Paired-Axis Derived-Z IMU → Rocket Frame Calibration

## Amaç
R8R28'de kalan son hassasiyeti kapatır: **DİK poz final dönüşüm matrisinin Z eksenine artık karıştırılmaz.**
Final eksenler dört eğik pozdan çözülür:

- `+X / -X` düzlemi → fiziksel `+Y`
- `+Y / -Y` düzlemi → fiziksel `+X`
- `+Z = +X × +Y`
- ilk DİK poz → sadece yön/kalite doğrulaması

Bu nedenle DİK pozdaki birkaç derecelik elle tutma hatası final matrise girmemelidir.

## Güvenlik
- **0 bar / tamamen basınçsız.**
- Needle/motor hareket etmemeli.
- Pnömatik valf bobinlerini mümkünse ayır; röle kartı LED/klik gözlemi yeterli.
- Bu paket **INERT / DEPRESSURIZED / NOT FLIGHT-QUALIFIED**.

## Test sırası
1. CubeIDE: `Clean Project → Build Project → Flash`.
2. UART monitor:

```bash
py monitor_uart_p112r12r8r29_paired_axis_derived_z_cal.py --port COM21 --duration 80
```

3. Promptları sırayla uygula:

```text
DİK (olabildiğince dik ve sabit; artık yalnız doğrulama)
→ +X_ROCKET 12–20°
→ -X_ROCKET 12–20°
→ +Y_ROCKET 12–20°
→ -Y_ROCKET 12–20°
→ CALIBRATED
```

Sensör üzerindeki X/Y oklarını değil **roket gövdesindeki fiziksel/CAD eksenlerini** kullan.

## Beklenen PASS
- `cal_phase = 10`
- `cal_valid = 1`
- `cal_fault = 0`
- +X/-X/+Y/-Y = 120/120/120/120 sample
- `xOpp >= 0.94`, `yOpp >= 0.94`
- `80° <= XY <= 100°`
- `axis agreement >= 0.95`
- `z agreement >= 0.97` (yalnız DİK doğrulaması)
- determinant 0.985–1.015
- ortho error <= 0.020
- needle PWM/move = 0/0
- miss ve scheduler realign = 0

Bu gerçek test PASS olursa çıkan matrisi sonraki sabit-matris revizyonuna gömüp açılış kalibrasyonunu kaldıracağız.
