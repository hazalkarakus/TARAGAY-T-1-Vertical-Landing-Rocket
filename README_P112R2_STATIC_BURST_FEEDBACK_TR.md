# P112R2 — Static Burst Feedback Qualifier

P112R1 logunda motor tamamen kapalıyken PC1 geri beslemesi NORMAL -> QUARANTINE geçti.
P112R2, hızlı ADC örnek gürültüsünü gerçek kaynak/wiper kararsızlığından ayırmak için her ADC kanalında 3 uzun-sample okumanın medianını kullanır.

Bu image MOTORU HİÇ HAREKET ETTİRMEZ. Bridge her TIM7 tickinde OFF tutulur.

Test:
1. Gaz/basınç bağlama.
2. UART monitorünü aç.
3. STM32 reset.
4. Pota/mekaniğe dokunmadan en az 15 s kaydet.
5. uart_p112r2_static_burst.txt dosyasını gönder.

PASS beklentisi: p83_feedback_valid=1, p83_mode=1, rate/quarantine sayaçları artmıyor ve raw/filter window p-p küçük.
Burst medianla da quarantine artarsa eşikleri gevşetme: PC1 wiper/kablo/ground/3.3V dalı fiziksel olarak düzeltilmeli.

BENCH ONLY / DO NOT FLY / NO GAS / NO PRESSURE.
