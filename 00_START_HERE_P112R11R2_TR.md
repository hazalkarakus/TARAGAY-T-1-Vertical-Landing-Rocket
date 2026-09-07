# P112R11R2 — 950+ CLOSED reference mechanical commissioning

**DO NOT FLY.** Bu sürüm yalnızca basınçsız/inert bench commissioning içindir.

R11R1 güvenlik düzeltmesi korunur. Ek değişiklik: CLOSED referansı artık **950–1023 ADC** aralığında yakalanabilir.
Pot/mekanik ayar için yazılım konvansiyonu değişmedi: **CLOSED yüksek ADC, OPEN yönünde ADC azalır.**

Kısa commissioning hareketi yine yakalanan CLOSED değerine göre 78 ADC'dir:
- CLOSED 950 -> OPEN target 872 -> HOLD -> CLOSE 950
- CLOSED 1008 -> OPEN target 930 -> HOLD -> CLOSE 1008
- CLOSED 1014 -> OPEN target 936 -> HOLD -> CLOSE 1014

Başlangıçta tam preflight readiness gerekir. Hareket başladıktan sonra R11R1 actuator-critical abort kuralları korunur.
PE9 ayrılması, flight_active, P83 feedback/mode kaybı, actuator/needle fault, operator PA0 abort ve timeout güvenli abort üretir.
