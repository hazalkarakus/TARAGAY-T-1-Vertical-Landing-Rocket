# R8R35R3R10R2 — E-STOP + Flight Feedback Recovery

Bu sürüm R3R10R1 60 s RAM-first logger tabanıdır. Kontrol otoriteleri, ESKF, RCS ve SD RAM-first mimarisi korunur.

Değişiklikler:
- E-STOP geldiğinde P83 feedback anlık quarantine/invalid ise motor OFF kalır ve en fazla 1200 ms feedback reacquire beklenir.
- Feedback NORMAL olduktan sonra 120 ms sürekli-valid doğrulama yapılır.
- Canlı feedback doğrulanmışsa geçmiş P111 FEEDBACK / ADC_INVALID latch'i yalnız bir kez ve yalnız CLOSED E-STOP hareketi için temizlenebilir. İkinci close-path fault motoru kapatıp fail eder.
- Normal aktif uçuşta P110 FEEDBACK_INVALID abort sonrası motor durmuşken, P83 NORMAL 250 ms sürekli stabil ise en fazla iki transient feedback fault otomatik temizlenebilir. Sonraki 200 Hz GNC komutu normal yetkilendirme yoluyla hareketi yeniden başlatır.
- Kör hareket yoktur; CLOSED reference kaybı bypass edilmez; STOP latch hiçbir zaman temizlenmez.
- Flight SDIO prohibition + 7.5 Hz / ~68.3 s RAM flight logger aynen korunur.

## Son inert doğrulama
1. Basınçsız/inert, motor gücü bağlı.
2. READY=1 sonrası PE9 ayır.
3. Needle'ın transient feedback quarantine sonrası yeniden komut alabildiğini doğrula.
4. E-STOP ver. stop_latched=1, auth=0, RCS=0 ve needle CLOSED ref +/-12 ADC içinde olmalı.
5. estop_close_complete=1, failed=0 gör.
6. RAM overflow=0, flight sırasında new SD DMA starts=0, postflight flush_complete=1.

Bu test geçmeden basınçlı/atış testine geçilmemelidir.
