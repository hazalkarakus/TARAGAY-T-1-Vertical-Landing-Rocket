TARAGAY-T1 V8.17 - MATLAB CONTROL PORT READY
=============================================

Amaç: En son MATLAB/Simulink dikey iniş kontrol algoritmasını STM32 sistemine
algoritmanın arayüzünü değiştirmeden entegre etmeye hazır hale getirmek.

Dondurulan arayüz:
    Valve_Cmd = fcn(z, v, m_guncel, P_main_bar)

z          : Roket CG yüksekliği [m]
v          : Düşey hız [m/s], aşağı yön negatif
m_guncel   : Güncel toplam kütle [kg]
P_main_bar : Ana hat basıncı [bar veya Pa]
Valve_Cmd  : Normalize iğne vana komutu [0..1]

Bu sürümde MATLAB algoritmasının gövdesi BİLEREK aktif değildir.
APP_MATLAB_VALVE_CONTROLLER_IMPLEMENTED = 0
Bu nedenle otomatik needle komutu 0 ve GNC arm reddedilir.

Veri yolu:
IMU/BARO/LIDAR -> FullStateESKF -> ControlInputProvider
              -> MatlabValveController_fcn(...) -> Valve_Cmd -> NeedleController

Şimdiki giriş kaynakları:
- z: ESKF (CG koordinatına dönüştürülmüş)
- v: ESKF velocity_z, aşağı negatif
- m_guncel: MODEL 27.5 kg
- P_main_bar: MODEL 300 bar

Kütle ve basınç MODEL olarak açıkça işaretlenir; canlı sensör gibi gösterilmez.
Uçuş öncesinde live pressure ve güncel mass modeline geçilecek.

Matlab entegrasyon noktası:
App/Modules/Control/MatlabValveController/matlab_valve_controller.c

Yeni raw diagnostic:
0x1000F180
  +0x00 magic       0x817C17D1
  +0x04 version     0x00081700
  +0x08 size        0x00000040
  +0x0C flags
  +0x10 z [mm]
  +0x14 v [mm/s]
  +0x18 mass [g]
  +0x1C pressure [mbar]
  +0x20 Valve_Cmd x10000
  +0x24 z source
  +0x28 v source
  +0x2C mass source
  +0x30 pressure source
  +0x34 seq begin
  +0x38 seq end
  +0x3C checksum

Source enum:
0 invalid, 1 ESKF, 2 MODEL, 3 SENSOR

Beklenti, algoritma entegre edilmeden önce:
z source=1, v source=1, mass source=2, pressure source=2
implemented flag=0, output_valid=0, Valve_Cmd=0

Sonraki adım: en son MATLAB V10.6 fonksiyon gövdesini buraya birebir port etmek.
