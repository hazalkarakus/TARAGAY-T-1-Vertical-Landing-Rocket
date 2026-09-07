# P112R12R8 - Clean Production Candidate / INERT dry-run

Bu surumde sentetik vertical-state, target-step, PA0 commissioning, revoke/fault
enjeksiyonu, R12R5 fiziksel scale/cap ve R12R7 no-motion wrapper yoktur.

Aktif zincir:

`sensors -> Full-State ESKF -> GeneratedFlightControl -> authorization -> P111 -> P110/P112 -> main needle motor`

R12R6R2 feedback root-cause propagation ve R12R7'de fiziksel olarak dogrulanan
50 ms GNC step-age fail-safe'i production guard olarak korunur. RCS/ground vent/
vent servo bu inert profilde hard-isolated kalir.

## Guvenlik

Yalniz tamamen BASINCSIZ/INERT bench. Gaz/propulsion/pyro/ignition/enerjik yukler
fiziksel olarak izole. RCS ve vent fiziksel yukleri bagli olmamali. Ana igne
motoru gercek GNC komutuna gore hareket edebilir; mekanik yolu acik ve emniyetli
tut.

## Test

1. CubeIDE -> Import Existing Projects into Workspace.
2. Clean Project -> Build Project.
3. Build hatasizsa Flash -> Reset.
4. Monitoru calistir:

   `py monitor_uart_p112r12r8_production_candidate_inert.py --port COM21 --duration 60`

5. `READY_PE9_CONNECTED` gorulunce PE9'i ayir.
6. Sonra sisteme dokunmadan en az 60 s kayit al.

Stationary inert bench'te beklenen tipik sonuc: gercek GNC 0, target CLOSED,
`moves=0`, P111 HOLD, P110 WAIT, faults=0, RCS applied=0, SD ready=1.

Gercek sensor/GNC durumu nonzero komut uretiyorsa ana igne motoru hareket
edebilir; bu durumda hareketin P110 PASS ile bitmesi ve final PWM=0 olmasi
beklenir. Bu surum UCUŞA HAZIR/QUALIFIED anlamina gelmez.
