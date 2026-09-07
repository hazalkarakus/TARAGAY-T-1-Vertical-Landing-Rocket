# P112R12R8R35R3 — SD SERVICE CLOSURE

Bu revizyon R8R35R2 kontrol/needle/RCS/ESKF davranisini DEGISTIRMEZ. Yalniz SD logger servis kapasitesini kapatir.

Degisiklikler:
- V14 384 B / 200 Hz capture AYNI.
- SD, nRF/remote'dan once kalir.
- SD admission: 520 us -> 320 us.
- Ring drain: her main-pass'te en fazla 1 frame; bounded micro-slice.
- Needle SEARCH/DRIVE/BRAKE sirasinda SD main-context service artik tamamen suppress edilmez. TIM7 actuator ISR aynen bagimsizdir.
- SD throughput UART diagnostigi eklendi: capture/push/pop, DMA start/complete, update/defer/suppress, drain/write timing.
- Needle/RCS/safety thresholdlari degistirilmedi.

Test (basincsiz):
`py monitor_uart_p112r12r8r35r3_sd_service_closure.py --port COM21 --duration 35`

Hedef: sd_ready/logging=1, drop/overrun/write-error delta=0, frame push/pop ~200 Hz, miss/realign=0.
