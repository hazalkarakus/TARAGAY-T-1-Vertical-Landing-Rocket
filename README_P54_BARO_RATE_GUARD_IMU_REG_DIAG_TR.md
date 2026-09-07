# TARAGAY-T1 P54 — Baro Rate Guard + IMU Register Diagnostic

P54, P53'ün başarılı ESKF/SD/scheduler korumalarını ve preflight barometre datum takibini aynen korur.

## Neden P54?

P53 soak kaydında iki yeni kanıt elde edildi:

1. Barometre ham akışında seyrek fakat fiziksel olarak imkânsız basınç sıçramaları vardı. Örnek sınıflar yaklaşık +48 Pa ve +1 kPa. Eski 50 Pa tek-sample eşiği +48 Pa sınıfını doğrudan geçirebiliyor; iki-benzer-sample teyidi de tekrarlayan büyük bir transport hatasını yeni seviye sanabiliyordu.
2. IMU repeated-word olaylarında üç redundant burstün altı ekseni de aynı 16-bit kelimeye dönüşüyordu (örn. 0xF5F5, 0xF9F9, 0xEFEF). Bu, gerçek hareketten çok SPI multi-byte transaction / register auto-increment / framing problemine işaret ediyor. P54 davranışı henüz değiştirmeden pattern anındaki sensör konfigürasyon registerlarını yakalar.

## Barometre değişikliği

Ham pressure guard artık zamana bağlıdır:

- noise floor: 4 Pa
- izin verilen rate: 180 Pa/s
- hard jump: 180 Pa

İzin:

`allowed = min(180 Pa, 4 Pa + 180 Pa/s * accepted_sample_dt)`

Böylece normal 200 Hz basınç değişimleri geçer; 5–10 ms içinde onlarca/yüzlerce Pa sıçrama filtre zincirine ulaşmaz. 180 Pa üzeri ani fark hiçbir zaman raw driver tarafından kabul edilmez.

P53 ground reference tracking + PE9/flight one-way freeze aynen korunur.

## IMU diagnostik değişikliği

P53'teki altı `imu_pat_corr_*` UART alanı zaten burst ile aynı repeated-word çıktıyı veriyordu. P54 bu altı alanı daha faydalı register snapshot ile değiştirir:

- imu_pat_reg_valid
- imu_pat_whoami
- imu_pat_ctrl1_xl
- imu_pat_ctrl2_g
- imu_pat_ctrl3_c
- imu_pat_ctrl4_c

Snapshot, ilk bad triplet sonrası soft bus resync'ten sonra ve deferred retry'dan önce alınır. Bu helper hiçbir register yazmaz; yalnızca diagnostiktir.

ISM330DLC için beklenen kritik değerler:

- WHOAMI: 0x6A (veya desteklenen alternatif 0x6B)
- CTRL1_XL: 0x8C
- CTRL2_G: 0x88
- CTRL3_C: bit mask 0x44 mevcut olmalı (BDU + IF_INC)
- CTRL4_C: bit 0x04 set olmalı

Özellikle pattern anında CTRL3_C içindeki IF_INC bitinin kaybolup kaybolmadığı bu testte görülecek.

## Wire format

- UART: `$TGY65`, V65, 297 field
- SD: V14 / 384 byte / 200 Hz — değişmedi

## İlk bench testi

1. Aktüatör/pnömatik enerji kapalı.
2. PE9 kapalı kalsın.
3. Kartı en az 10 dakika hareket ettirmeden açık bırak.
4. UART:
   `python -u monitor_uart_p54.py --port COM8`
5. 10 dakika boyunca `baro_ref_track_active=1`, `baro_ref_frozen=0`, `baro_ref_updates` artmalı.
6. Baro altitude mümkün olduğunca 0 çevresinde kalmalı.
7. Pattern oluşursa `regs=` alanı dolmalı.
8. Sonra PE9 açılır; reference freeze olmalı.

P54 final uçuş firmware'i değildir. UART flight-mode suppression ve actuator/fault-injection final doğrulamaları daha sonra yapılacaktır.
