# P112R10R3 — GNC MOTOR BENCH / SETTLED TARGET HOLD

Bu sürüm P112R10R2 inert motor bench kaydında görülen P111 same-target retrigger davranışını düzeltmek için hazırlanmıştır. UÇUŞ İÇİN DEĞİLDİR.

## Neden R10R3?
R10R2'de ilk OPEN hareketi P110 tarafından PASS kabul edildi ve `moves_completed=1` oldu. Ancak GNC 200 Hz'de aynı OPEN hedefini tekrar göndermeye devam ettiği için P111, kabul edilmiş settled konumu kalıcı HOLD olarak tanımadı ve aynı hedef için ikinci bir P110 hareketi başlattı. Bu gereksiz ikinci breakaway overshoot/fault üretti.

## R10R3 değişikliği
- P110 adaptive correction-feasibility mantığı R10R2 ile aynıdır.
- Nominal +/-8 ADC tolerance, 12 ADC hard overshoot, breakaway/search/sustain/brake/coast learning değişmemiştir.
- P111 yalnız bir P110 PASS sonrasında o tamamlanan hedefi ve kabul edilmiş gerçek pozisyonu latch eder.
- GNC aynı hedefi tekrar yolluyorsa (hedef farkı <=8 ADC) ve feedback kabul edilen settled pozisyondan <=8 ADC drift etmişse P111 HOLD'da kalır; yeni powered P110 request açılmaz.
- GNC hedefi gerçekten değişirse (>8 ADC), latch temizlenir ve normal autonomous retarget/move yolu çalışır.
- Feedback kabul edilmiş settled pozisyondan >8 ADC drift ederse latch temizlenir ve normal closed-loop düzeltme tekrar aktif olur.
- Authorization revoke veya fault latch'i settled hold'u temizler.

Bu global toleransı büyütmek değildir. P110'nun o hareket için verdiği PASS kararını P111'in aynı target tekrarlarında korumasıdır.

## Beklenen bench akışı
TAMAMEN BASINCSIZ / INERT BENCH ONLY.
RCS, vent, gaz ve propulsion donanımı fiziksel olarak ayrık tutulmalıdır.
Vana başlangıçta CLOSED (~1015..1023 ADC), PE9 bağlı boot edilir.
Preflight READY sonrası PE9 ayrılır.

Beklenen tek çevrim:
CLOSED -> ~789 OPEN -> P110 PASS -> P111 HOLD (aynı 789 tekrarlarında YENİ OPEN YOK) -> GNC 1023 -> CLOSE -> PASS -> CLOSED HOLD.

PASS kriterleri:
- preflight_ready=1
- actuator_authorized=1
- OPEN target görülür
- target CLOSED'a dönmeden önce `moves_completed` tam 1 olur
- final `moves_completed` tam 2 olur
- needle_fault=0, p111_fault=0
- p112_hard_off_count=0
- final PWM=0
- final target CLOSED ve ADC baseline'a yakın

R10R3 doğrulanmadan bu P111 davranışı production R8'e taşınmamalıdır.
