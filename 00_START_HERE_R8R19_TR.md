# P112R12R8R19 — Flight Logic Dry-Run (INERT)

## Amaç
Bu rev, son MATLAB kaynak otoritelerini STM32 tarafına compute-only olarak taşır:

1. Hover V19.6 mission/vertical controller
2. yatayKontrol_EnvelopeUyumlu
3. RCS_Denge_Kontrol_V3_TekBlok V7.13.4

R8R19 fiziksel uçuş build'i değildir. Basınç/gaz kullanılmaz.

## Kritik güvenlik profili
- APP_R8R19_FLIGHT_LOGIC_COMPUTE_ONLY = 1
- APP_R8R19_FLIGHT_LOGIC_SYNTHETIC_TEST = 1
- APP_P112R12_INERT_OUTPUT_ISOLATION_MODE = 1
- R8R18 motor-bench stimulus = 0
- Yeni flight logic needle komutunu fiziksel motora UYGULAMAZ.
- Yeni flight logic RCS request üretir fakat physical applied mask daima 0'dır.
- Eski GeneratedFlightControl bu testte HoldSafe edilir.
- Existing P112/P110/P111, ESKF, sensor, scheduler, nRF ve SD çekirdeği değiştirilmemiştir.

## 3.5 mm boğaz
Gönderilen Hover V19.6 içindeki F_MAP 4.5 mm boğaza aittir.
R8R19 dry-run için yalnız başlangıç tahmini olarak:

    scale = (3.5 / 4.5)^2 = 0.604938...

ile force ordinatları ölçeklenmiştir.

Bu harita PROVISIONAL / SIMULATION-ONLY'dir.
3.5 mm gerçek statik itki-vana karakterizasyonu yapılmadan flight map kabul edilmez.

## Otomatik hızlı test senaryosu
Preflight `ready=1` olduktan 1 saniye sonra synthetic mission sıfırlanıp başlar.
Yaklaşık 10.2 saniyede logic otomatik olarak şu durumları dolaşır:

- 0 ARM
- 1 ASCENT
- 2 HOVER (8.0 s)
- 3 DESCENT
- 4 TOUCHDOWN

Yatay konum, attitude ve angular-rate synthetic uyarıları RCS V7.13.4'ün request üretmesini sağlar.
Fiziksel RCS çıkışı uygulanmaz.

## Test
CubeIDE:
1. Clean Project
2. Build Project
3. Flash
4. Gaz/basınç YOK
5. PE9 için özel işlem gerekmez; bu test synthetic input kullanır.
6. UART:

    py monitor_uart_p112r12r8r19_flight_logic_dryrun.py --port COM21 --duration 25

PASS için:
- state 0,1,2,3,4 görülmeli
- final state 4
- RCS request mask en az bir kez non-zero
- fl_rcs_fault = 0
- fl_rcs_applied_mask = 0
- gerçek rcs_mask = 0
- p110_pwm = 0
- p111 move delta = 0
- IMU/LiDAR/nRF/ESKF miss delta = 0 hedeflenir
- final system_ok=1, eskf_ok=1

## Sonraki kapı
R8R19 PASS sonrası:
- synthetic input kapatılacak
- aynı compute-only logic gerçek ESKF x/y/z/v ve attitude/rate ile beslenecek
- fiziksel çıktılar hâlâ kapalı kalacak
- bunun PASS olması sonrası actuator/RCS output authorization ayrı revde ele alınacak
