# P112R12R8R22 — GERCEK ESKF GIRISI / COMPUTE-ONLY / INERT

## Amaç
R8R21'de geçen synthetic flight-logic testinden sonra synthetic veri kapatıldı.
Son kaynak-otorite kontrol zinciri artık gerçek uçuş bilgisayarı verilerini okur:

`Full-State ESKF x,y,z,vx,vy,vz,pitch,yaw + filtreli IMU gyro -> Hover V19.6 + landing-aware yatay kontrol + RCS V7.13.4`

Bu revizyon **yalnız compute-only** doğrulamadır.
- Ana needle fiziksel komutu uygulanmaz.
- RCS fiziksel komutu uygulanmaz.
- Vent/RCS diğer fiziksel çıkışları INERT isolation altında kalır.
- UART yalnız diagnostiktir.
- BASINÇ / GAZ OLMADAN test edin.

## Kritik R8R22 farkları
- `APP_R8R19_FLIGHT_LOGIC_SYNTHETIC_TEST = 0`
- `fl_real = 1`, `fl_synth = 0`
- Pitch/yaw artık ayrı AttitudeEstimator yerine doğrudan **Full-State ESKF public state** kaynağından alınır.
- Pitch/yaw rate son RCS MATLAB otoritesine uygun şekilde filtreli IMU gyrodan gelir.
- ESKF public age >20 ms veya IMU age >10 ms ise flight-logic input reddedilir ve compute komutları 0'a çekilir.
- ESKF vertical validity/covariance/origin/output-inhibit ayrıca gate edilir.
- Eski GeneratedFlightControl / VerticalLandingControl / legacy GNC otoriteleri R8R21'deki gibi retired kalır.
- Needle maksimum travel: **3 tur = 585 ADC**.

## Önemli yatay navigasyon notu
Mevcut Full-State ESKF'de mutlak yatay aiding yoktur. Bu nedenle `horizontal_position_valid=0` beklenir.
R8R22 gerçek `x/y/vx/vy` değerlerini kısa bench diagnostik için kontrol algoritmasına verir, fakat bu **yatay konum kontrolünün uçuş kalifikasyonu değildir**.
Fiziksel RCS zaten uygulanmaz.

## Hızlı test — 15 saniye
1. CubeIDE: **Clean -> Build -> Flash**.
2. Roket BASINÇSIZ / GAZSIZ olsun.
3. PE9 bağlı kalsın; E-STOP ve VENT switchleri OFF.
4. UART:

```bash
py monitor_uart_p112r12r8r22_real_eskf_compute_only.py --port COM21 --duration 15
```

5. `ready=1` sonrası beklenen:
   - `fl_real=1`
   - `fl_synth=0`
   - `fl_input_valid=1`
   - `fl_vpos_valid=1`
   - `fl_origin_zeroed=1`
   - `fl_eskf_inhibit=0`
   - `fl_hpos_valid=0` mevcut mimaride normal/uyarıdır.
6. Güvenliyse roketi yalnız küçük ve kontrollü şekilde eğerek `fl_pitch/fl_yaw` değerlerinin gerçek hareketi izlemesini gözlemleyebilirsiniz. RCS request oluşması bu test için zorunlu değildir.
7. Fiziksel çıkışlar MUTLAKA:
   - `p110_pwm=0`
   - `needle_lpwm=0`
   - `needle_rpwm=0`
   - `rcs_mask=0`
   - `fl_rcs_applied_mask=0`

## Mission-state notu
Bu revizyon sensör->algoritma bağlantı testidir. MATLAB Hover V19.6 kaynak mantığında `ARM -> ASCENT -> HOVER -> DESCENT -> TOUCHDOWN` state yapısı bulunduğu için, roket bench üzerinde yerde sabitken state'in ASCENT'te kalması beklenebilir. R8R22 PASS kriteri state 0->4 tamamlamak değildir.

## Bana gönderilecek dosya
`uart_p112r12r8r22_real_eskf_compute_only.txt`

Logdan gerçek ESKF kaynak kalitesi, rate/attitude yönleri, RCS kararları, physical isolation ve scheduler/CPU birlikte kontrol edilecek.
