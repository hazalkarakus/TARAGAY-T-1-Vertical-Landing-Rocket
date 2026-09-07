# P112R12R8R33 — Real Flight Logic → Physical RCS

Bu revizyon R8R32 fiziksel needle authority'sini aynen korur ve son RCS V7.13.4
request maskesini gerçek dört relay çıkışına bağlar.

## Tek RCS otoritesi

`Full-State ESKF -> fixed R8R30 IMU→rocket matrix -> RCS V7.13.4 -> SolenoidOutput`

Eski V49/AttitudeControl RCS yolu fiziksel maskeyi yazamaz. GNCActiveControl ve
GeneratedFlightControl RCS sahipliği emekli kalır.

## Fiziksel RCS authorization

Nonzero RCS maskesi ancak aşağıdakilerin hepsi doğruysa uygulanır:

- PE9 ayrılmış / `flight_active=1`
- actuator authorization aktif
- STOP latch yok
- SystemMonitor `system_ok=1`
- real ESKF input aktif ve valid
- synthetic input kapalı
- fixed mount matrix valid, fault yok
- RCS V7.13.4 fault yok
- flight-logic step age <= 50 ms

Herhangi bir kapı kaybolursa output SAFE/0 yapılır. 1 kHz servis eski kontrol
komutu üretmez; yalnız hızlı safety cut uygular.

## Elektrik kanal eşleşmesi

- `0x01` -> relay IN1 -> PB15
- `0x02` -> relay IN2 -> PE15
- `0x04` -> relay IN3 -> PE11
- `0x08` -> relay IN4 -> PE7

Bu liste elektrik kanal eşleşmesidir. Son fiziksel düzeltme yönü/sign kabulü
basınçsız relay-direction testinde gözlenmelidir.

## Bench doğrulama

Bu doğrulama BASINÇSIZ yapılmalıdır. Solenoid yüklerini relay kartından ayırmak
en temiz seçenektir; amaç relay maskesinin doğru fiziksel kanala ulaşmasını
görmektir.

1. Build + flash.
2. UART monitorü aç.
3. `ready=1`, `system_ok=1`, `eskf_ok=1`, `cal_phase=12` bekle.
4. PE9'u ayır.
5. Roketi yavaşça farklı yönlere eğerek RCS request üret.
6. `fl_rcs_req_mask`, `fl_rcs_applied_mask`, `rcs_mask` gözle.
7. PE9 öncesi fiziksel maskenin daima 0 olduğunu doğrula.
8. Opposing pair (`0x03` veya `0x0C`) fiziksel olarak uygulanmamalı.

Monitor:

`py monitor_uart_p112r12r8r33_real_flight_logic_physical_rcs.py --port COM21 --duration 30`

R8R33 hâlâ final atış kabul firmware'i değildir; bu revizyonun amacı fiziksel
RCS authority entegrasyonunu doğrulamaktır.
