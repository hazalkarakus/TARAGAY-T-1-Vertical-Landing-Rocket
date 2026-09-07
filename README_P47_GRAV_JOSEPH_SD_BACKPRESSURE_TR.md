# P47 – GRAV Joseph PSD Fix + SD Backpressure

Taban P46R1. P46R1 logunda kalan tek covariance fault GRAV stage / BGY
(state 13), raw -0.000410027 idi. Ayni testte SD ring 194 drop/overrun verdi.

P47 dar kapsam:
- GRAV two-state scalar covariance update: Joseph form
  P+ = (I-KH)P(I-KH)^T + KRK^T.
- P46 root-cause latch, z/vz public guard ve state-preserving rollback aynen kalir.
- 128-frame CCM ring buyutulmez; occupancy-aware drain: 4/320us, 6/480us,
  8/650us.
- DATA DMA guard sektorden onceliklidir. Tek guard pending tutulur ve backlog
  dusunce en yeni data sonuna yazilir.
- TGY58 SD ring/backpressure/card-busy/write-max ve GRAV Joseph sayaclarini verir.
- SD binary V14 / 384 byte ve 128/96 MiB preallocation degismez.
