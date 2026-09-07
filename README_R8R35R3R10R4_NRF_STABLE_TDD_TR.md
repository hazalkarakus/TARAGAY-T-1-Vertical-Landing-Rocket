# TARAGAY-T1 R8R35R3R10R4 — nRF Stable TDD

## Neden
R3R10R3 uzun inert testinde roket flight modunda nRF downlink'i tamamen kapatinca roket alicisi STOP paketini alabildi, fakat normal OFF/OFF ground heartbeat ciddi sekilde seyrekledi. Flight baslangicindan STOP'a kadar normal durumda yalniz birkac yeni gecerli komut kabul edildi. STOP aktif edilince ground emergency retransmit yolu tekrar cok sayida paket ulastirdi.

Bu imza roket SPI/nRF alicisinin tamamen cokmesinden ziyade mevcut ground-station explicit-TDD dongusunun bekledigi 32-byte cevabi alamadiginda normal heartbeat cadence'ini kaybetmesiyle uyumludur.

## R4 degisikligi
- Flight'ta tamamen RX-only politika kaldirildi.
- Her kabul edilen 4-byte command paketine mevcut +4.5 ms TDD zamaninda tam **bir** 32-byte FAST telemetry cevabi verilir.
- Flight'ta duplicate frame YOK.
- Flight'ta 19-page detail rotation YOK.
- Async TX tamamlaninca mevcut nRF state-machine tekrar 4-byte PRX'e doner.
- Preflight telemetry davranisi korunur.
- Komut parser, STOP latch, actuator authority, RCS, needle, ESKF, scheduler ve RAM-first SD logger degistirilmedi.

## Beklenen flight RF akisi
Ground PTX command (~20 Hz) -> Rocket PRX accepts -> +4.5 ms Rocket one FAST PTX -> Rocket PRX -> Ground next command.

## Kabul
Basinssiz/inert testte PE9 sonrasi ve STOP'tan once:
- accepted command rate tercihen 19–21 Hz, minimum kabul hedefi 15 Hz
- nrf_link=1 surekli (ilk 1 s grace sonrasi link-down sample 0)
- packet age <500 ms, ideal <150 ms
- flight FAST TX start artar
- nrf_tlm_tx_fail=0
- nrf_errors=0
- nrf_invalid=0
- STOP test sonunda latch olmalidir

## Not
Bu revizyon overshoot politikasini degistirmez. Kullanici istegi geregi R4 yalniz nRF haberlesme davranisina odaklanir.


## R4R1 notu
Bu klasörde R4 nRF davranışı korunarak postflight SD safe-replay gate düzeltmesi eklenmiştir. Ayrıntı: `README_R8R35R3R10R4R1_POSTFLIGHT_SD_SAFE_REPLAY_TR.md`.
