TGY V30 - 2 SWITCH SAFE VENT/STOP TEST

TABAN: Kullanıcının yüklediği V26.

SWITCH-1 (F103 PA0):
- nRF paketinde bit0.
- F407 üzerinde darbeli VENT REQUEST üretir.
- 100 ms ON / 400 ms OFF yalnızca gösterge/test zamanlamasıdır.
- Mevcut RCS karşılıklı-valf interlock'u KORUNDU.
- Dört valfi aynı anda fiziksel olarak açan bypass KODLANMADI.
- v30_vent_physical_enabled her zaman 0'dır.

SWITCH-2 (F103 PA1):
- nRF paketinde bit1.
- Bir kere alınırsa v30_stop_latched=1 olur.
- SolenoidOutput_ForceSafe() çağrılır.
- NeedleValveController_Stop() çağrılır.
- Reset/power-cycle yapılmadan latch temizlenmez.
- F407 kırmızı LED PD14 sürekli yanar.

SWITCH-1 yeşil LED:
- RemoteControl içindeki mevcut yeşil LED davranışı yalnızca bit0'a bağlıdır.

Live Expressions:
remote_rx_command_flags
remote_rx_command
remote_rx_switch2_command
v30_remote_flags
v30_switch1_vent_request
v30_vent_pulse_active
v30_vent_pulse_count
v30_vent_physical_enabled
v30_switch2_stop_request
v30_stop_latched
v30_stop_latch_count

NOT: Fiziksel dört-valf eşzamanlı tahliye, mevcut güvenlik interlock'unu devre dışı bırakmayı gerektirir ve bu test sürümünde özellikle yapılmamıştır.
