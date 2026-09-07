# P112R12R8R16 — E-STOP EMERGENCY CLOSE + P73 FAST NRF — INERT ONLY

Bu revizyon R8R13 P73 FAST/REDUNDANT nRF tabanından gelir.
Sensörler, Full-State ESKF, scheduler, SD, nRF24 driver, P73 NRFTelemetry,
RemoteControl ve GeneratedFlightControl frozen bırakılmıştır.

## Yeni davranış
Switch-2 E-STOP latch olduğunda:
1. RCS ve ground-vent anında SAFE olur.
2. Normal flight actuator authorization iptal olur.
3. Tek izinli post-STOP motor hareketi: öğrenilmiş CLOSED reference'a safe-close.
4. CLOSED reference veya feedback geçersizse motor hareket ettirilmez.
5. Needle/autonomous fault varsa motor hareket ettirilmez.
6. Safe-close 6000 ms içinde tamamlanmazsa motor kesilir ve failure latch edilir.
7. CLOSED +/-12 ADC içine girince motor de-energize edilir.
8. STOP latch açık kalır; normal GNC motoru tekrar açamaz.

Safe-close normal same-target 200 ms jitter dwell'ini bypass eder. Bu nedenle vana
CLOSED hedefteyken elle açık tarafa taşınmış olsa bile E-STOP sonrası kapanma
komutu bir sonraki 1 kHz actuator tick'ine ulaşabilir.

## TGY73 yeni alanlar
stop_latched,
estop_close_active,
estop_close_complete,
estop_close_failed,
estop_close_fail_reason,
estop_close_start_count,
estop_close_elapsed_ms

Failure reason:
0 none
1 CLOSED reference yok
2 robust feedback geçersiz
3 autonomous fault
4 low-level needle fault
5 6000 ms timeout
6 CLOSED command reject

## Bench test
BASINCSIZ / GAZSIZ / INERT test.
1. Clean -> Build -> Flash -> Reset.
2. Sistem ready=1 olana kadar bekle.
3. Learned CLOSED reference yakalandıktan sonra iğne vanayı/mekanik hattı güvenli şekilde açık tarafa taşı.
4. Ground Switch-2 E-STOP'u uygula.
5. Motor CLOSED reference'a dönmeli.
6. CLOSED'a girince PWM 0 olmalı ve STOP latch 1 kalmalı.
7. Switch-2'yi bıraksan da motor normal komutla tekrar açılmamalı.

UART:
py monitor_uart_p112r12r8r15_estop_safe_close.py --port COM21 --duration 60

Beklenen final:
stop_latched=1
estop_close_complete=1
estop_close_failed=0
estop_close_fail_reason=0
p110_pwm=0
rcs_mask=0


## R8R16 değişikliği
- CLOSED toleransı içindeyse önceki transient actuator fault safe-close sonucunu engellemez.
- CLOSED dışında ve reference/feedback sağlıklıysa önceki STALL/LOW_LEVEL fault tam bir kez emergency-close için temizlenebilir.
- Retry sırasında fault tekrar oluşursa motor kesilir ve emergency-close FAIL kalır.
- Reference/feedback fault hiçbir zaman bypass edilmez.
