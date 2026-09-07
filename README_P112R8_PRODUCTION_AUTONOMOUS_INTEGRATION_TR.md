# P112R8 — Production Autonomous Integration

## R7'den farkı

P112R7'deki özel single-shot PA0 bench sequencer kaldırılmıştır. Production `GeneratedFlightControl` 200 Hz komut rotası tekrar tek ana iğne-vana komut üreticisidir.

Aşağıdaki bench-only davranışlar P112R8'de yoktur:
- PA0 ile `1023 -> ~800 -> 1023` hareketi
- single-shot DONE latch
- bench nedeniyle fiziksel RCS/remote-vent kapatma

Production tarafında tekrar etkin olan yollar:
- `GeneratedFlightControl -> NeedleValveAutonomousControl_SubmitCommand()`
- V49 ESKF signed-PD physical RCS policy
- production remote-vent physical policy

UART RX motor komutu kapalıdır.

## Korunan P112 çekirdeği

P110/P111/P112 adaptive motor kontrol parametre ve state-machine kaynakları değiştirilmemiştir. R8, SD boot/TIM7 fix içeren production tabanına geri dönerek R7'de fiziksel bench üzerinde doğrulanan adaptive katmanı gerçek GNC hedeflerine bağlar.

## Geçersiz GNC sonucu davranışı

Common authorization sağlıklı olduğu halde vertical controller sonucu geçersizse komut `0.0 = CLOSED` olur. Common authorization kaybolursa `RevokeAuthorization()` ile motor yolu kesilir.

## Test sırası

- CubeIDE Clean + Build
- Basınçsız boot / closed-reference
- PE9 bağlı preflight doğrulaması
- PE9 transition kuru bench
- GNC command/target telemetry doğrulaması
- HIL ile non-zero valve command doğrulaması
- fault injection: stale command / feedback invalid / authorization loss

Gerçek pnömatik/itki testi ancak bu kuru entegrasyonlardan sonra yapılmalıdır.
