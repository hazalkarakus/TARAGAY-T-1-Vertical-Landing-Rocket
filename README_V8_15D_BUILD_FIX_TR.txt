V8.15D BUILD FIX
================

Hata:
app_tasks.c içinde cpu_load_percent ve cpu_idle_percent undeclared.

Neden:
Bu global'ler scheduler.c içinde tanımlıydı ancak scheduler.h üzerinden
export edilmiyordu. app_tasks.c scheduler.h include etmesine rağmen sembol
bildirimi görünmüyordu.

Düzeltme:
- scheduler.h içine CPU live-metric extern bildirimleri eklendi.
- Fixed diagnostic blok CPU değerlerini artık scheduler'ın zaten ürettiği
  integer cpu_load_percent_x100 / cpu_idle_percent_x100 değişkenlerinden
  doğrudan alıyor.
- Gereksiz V815D_PercentToX100() helper kaldırıldı.

Test:
Clean -> Build.
Build geçince V8.15D raw-memory testi aynı şekilde 0x1000F000 adresinden
yapılır.
