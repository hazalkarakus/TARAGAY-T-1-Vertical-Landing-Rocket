# V49 — Full-State ESKF Tabanlı İşaretli-PD RCS

Bu sürümde fiziksel dört valfli RCS yolu görsellerde tarif edilen yapıya
çekilmiştir. Klasik `AttitudeEstimator -> RCS` yolu fiziksel komut üretmez;
RCS'nin tek fiziksel kontrol kaynağı Full-State ESKF'dir.

## Veri yolu ve frekanslar

1. ISM330DLC IMU verisi 1000 Hz okunur.
2. Full-State ESKF nominal durumu 1000 Hz ilerletir.
3. ESKF roll/pitch çıktısı 200 Hz yayımlanır.
4. RCS denetleyicisi her yeni ESKF örneğini yalnız bir kez işler.
5. TIM7 kesmesi 1000 Hz'de atım ve cooldown sayaçlarını bağımsız azaltır.

RCS açıları:

- `e_roll = ESKF roll` (hedef sıfır derece)
- `e_pitch = ESKF pitch` (hedef sıfır derece)

Açısal hızlar, filtrelenmiş jiroskoptan ESKF biası çıkarılarak elde edilir:

- `roll_rate = gyro_x_filtered - eskf_gyro_bias_x`
- `pitch_rate = gyro_y_filtered - eskf_gyro_bias_y`

## İşaretli PD süre üreticisi

Her eksen için:

```text
t_signed_ms = 2.50 * angle_error_deg + 3.55 * rate_error_dps
```

Örnek:

```text
angle = +3 deg
rate  = +10 deg/s
t     = 2.50*3 + 3.55*10 = +43 ms
```

İşaret hangi hata-yönü valfinin açılacağını belirler. Roket sıfıra doğru hızlı
gelirken türev terimi toplam işareti açı sıfırı geçmeden değiştirebilir. Böylece
karşı valf doğal aktif frenleme yapar.

## Mekanik kırpma

- `|t| < 20 ms`: valf hiç açılmaz, komut sıfırdır.
- `20 ms <= |t| <= 60 ms`: hesaplanan süre uygulanır.
- `|t| > 60 ms`: süre 60 ms'ye kırpılır.

Bu çıkış PWM değildir. Röle/solenoid tek sefer açılır ve TIM7 sayaç sıfıra
geldiğinde otomatik kapanır. `HAL_Delay()` veya bloklayan bekleme kullanılmaz.

## Anti-chatter / cooldown

Her eksen kendi atımı bittikten sonra 100 ms kilitlenir. Bu sırada:

- sensörler ve ESKF çalışmaya devam eder,
- diğer eksenin bağımsız atımı devam edebilir,
- kilitli eksene yeni valf emri verilmez,
- karşıt valflerin aynı anda açılması çıkış interlock'u ile ayrıca engellenir.

## Fiziksel çıkışlar

Çıkışlar aktif-LOW'dur:

| Hata yönü | Röle | STM32 pini |
|---|---:|---|
| Roll pozitif hata düzeltme | IN1 | PE7 |
| Roll negatif hata düzeltme | IN2 | PE11 |
| Pitch pozitif hata düzeltme | IN3 | PE15 |
| Pitch negatif hata düzeltme | IN4 | PB15 |

Mekanik valf/nozul yönü ters çıkarsa kaynak kodu değiştirmek yerine önce
`ATT_CTRL_ROLL_*_SIGN` ve `ATT_CTRL_PITCH_*_SIGN` sabitleri kontrollü bir kuru
test ile doğrulanmalıdır.

## Uçuş ve güvenlik kapıları

- PE9 ayrılma olayı doğrulanmadan RCS kesin kapalıdır.
- ESKF/IMU sağlıksız veya ESKF örneği 100 ms'den eskiyse bütün RCS kapanır.
- NRF Switch-2 STOP kilidi aktif olduğunda canlı atım sayaçları da temizlenir;
  zamanlayıcı daha sonra valfi yeniden açamaz.
- Aynı eksendeki iki zıt valf hiçbir koşulda birlikte uygulanmaz.
- NRF alıcısı bu sürümde açık kalmıştır (`APP_NRF24_ENABLED=1`).

## Değişmeyen kapsam

Düşey iniş ve Simulink ana-vana sonuçları hâlâ compute-only'dir. Bu V49
değişikliği yalnız fiziksel roll/pitch RCS yolunu kapsar; uçuşta iğne vanayı
fiziksel olarak devreye almaz.

## İlk test sırası

1. Basınç ve gerçek solenoid olmadan röle/LED yükleriyle PE9 kapısını doğrula.
2. Her pini tek tek kontrol ederek pozitif/negatif yön eşlemesini doğrula.
3. 3 derece, 10 derece/s senaryosunda 43 ms komut görüldüğünü doğrula.
4. 8 ms eşdeğeri komutta rölenin hiç çekmediğini doğrula.
5. Büyük komutta sürenin 60 ms'yi geçmediğini doğrula.
6. Atım sonrası aynı eksenin 100 ms boyunca yeni emir almadığını doğrula.
7. Atım devam ederken NRF STOP verip bütün çıkışların anında kapandığını doğrula.

Kazançlar gerçek basınçlı HIL/drop-test verisiyle yeniden ayarlanmalıdır;
2.50/3.55 değerleri görseldeki 43 ms örneğini ve 281.5 deg/s2 fiziksel açısal
ivme modelini başlangıç noktası olarak kullanır.
