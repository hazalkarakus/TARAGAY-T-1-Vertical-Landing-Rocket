# P112R12R8R35R3R3 — SD PHASE RETRY HOTFIX

Bu revizyon yalnız SD background admission davranışını değiştirir.

R8R35R3R2 fiziksel logunda:
- TIM5 capture üretimi `successful_push + dropped ~= 200 Hz` idi.
- SD main-context update/pop yalnız yaklaşık 41 Hz çalıştı.
- Ring 127/127 doldu ve drop/overrun arttı.
- Kök neden: `last_sd_service_us` slack kontrolünden ÖNCE güncelleniyordu; defer edilen SD çağrısı sonraki main-loop turunda hemen tekrar denenmiyor, 1 ms bekleyip aynı scheduler fazına kilitlenebiliyordu.

R3R3 düzeltmesi:
- `last_sd_service_us` yalnız SDLogger_Update gerçekten kabul edilip çalıştırıldığında güncellenir.
- Slack yetersizse timestamp değişmez; sonraki main-loop spin tekrar dener.
- 200 Hz capture, 384-byte V14 frame, 1-frame bounded drain, 320 us SD admission, DMA writer ve kontrol/actuator kodları DEĞİŞMEDİ.

Basınçsız doğrulama hedefi:
- SD ready/log = 1/1 final
- drop/overrun delta = 0/0 (startup transient varsa READY sonrası delta 0)
- update/pop >= 200 Hz
- ring < 127 ve düşen trend
- write error = 0
- scheduler miss/realign = 0
