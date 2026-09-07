V8.15D FIXED DIAGNOSTIC BLOCK
=============================

PURPOSE
-------
Live Expressions/DWARF is not trusted.

Firmware writes a raw 192-byte block to a LINKER-FIXED address:

    0x1000F000

Do NOT add v815d_diag to Live Expressions for the first test.

CUBEIDE TEST
------------
1) Motor PSU OFF.
2) Clean -> Build -> Debug -> Resume.
3) Leave board running 5-10 seconds.
4) Click Suspend/Pause.
5) Window -> Show View -> Memory (or Memory Browser).
6) Add address:

       0x1000F000

7) Display at least 192 bytes.
8) Prefer 32-bit hexadecimal word rendering if CubeIDE offers it.
9) Send a screenshot of the raw memory.

FIRST 5 WORDS MUST BE
---------------------
Address      Word
0x1000F000   0x815D15D1   MAGIC
0x1000F004   0x11223344   endian test
0x1000F008   0x0008150D   V8.15D version
0x1000F00C   0x000000C0   block size = 192 bytes
0x1000F010   sequence_begin

If these are correct, the debugger is reading the actual fixed memory address.

STABLE SNAPSHOT RULE
--------------------
sequence_begin at 0x1000F010
sequence_end   at 0x1000F0B4

They must be:
- equal
- even

checksum:
0x1000F0B8 = checksum_xor
0x1000F0BC = checksum_xor_inv

They must satisfy:
checksum_xor_inv == bitwise NOT checksum_xor

WORD/OFFSET MAP
---------------
+0x00 magic                         u32 hex
+0x04 endian_magic                  u32 hex
+0x08 version                       u32 hex
+0x0C block_size_bytes              u32
+0x10 sequence_begin                u32
+0x14 flags                         u32 hex

+0x18 attitude_update_count         u32
+0x1C eskf_predict_count            u32
+0x20 eskf_public_count             u32
+0x24 eskf_covariance_count         u32
+0x28 eskf_correction_count         u32

+0x2C cpu_load_x100                 u32   5534 = 55.34%
+0x30 cpu_idle_x100                 u32

+0x34 baro_update_count             u32
+0x38 baro_pressure_pa              s32   INT32_MIN = invalid/not connected
+0x3C baro_temperature_mC           s32   23950 = 23.950 C
+0x40 baro_altitude_mm              s32

+0x44 lidar_update_count            u32
+0x48 lidar_distance_mm             s32

+0x4C baro_age_us                   u32   FFFFFFFF = no sample
+0x50 lidar_age_us                  u32

+0x54 origin_zero_count             u32
+0x58 eskf_numerical_error_count    u32

+0x5C sd_error_count                u32
+0x60 sd_write_error_count          u32

+0x64 sd_ring_address               u32 hex
+0x68 sd_ring_end_address           u32 hex
+0x6C bss_end_address               u32 hex
+0x70 msp_address                   u32 hex
+0x74 main_sram_gap_bytes           u32

+0x78 position_x_mm                 s32
+0x7C position_y_mm                 s32
+0x80 position_z_mm                 s32

+0x84 velocity_x_mmps               s32
+0x88 velocity_y_mmps               s32
+0x8C velocity_z_mmps               s32

+0x90 roll_mdeg                     s32
+0x94 pitch_mdeg                    s32
+0x98 yaw_mdeg                      s32

+0x9C baro_reference_mm             s32
+0xA0 lidar_reference_mm            s32
+0xA4 baro_innovation_mm            s32
+0xA8 lidar_innovation_mm           s32

+0xAC eskf_reset_count              u32
+0xB0 eskf_gap_skip_count           u32
+0xB4 sequence_end                  u32
+0xB8 checksum_xor                  u32 hex
+0xBC checksum_xor_inv              u32 hex

FLAGS +0x14
-----------
bit  0 block alive
bit  1 attitude enabled
bit  2 attitude initialized
bit  3 attitude healthy

bit  4 ESKF enabled
bit  5 ESKF shadow mode
bit  6 ESKF initialized
bit  7 ESKF healthy
bit  8 stationary

bit  9 physical barometer connected
bit 10 physical barometer healthy
bit 11 ESKF baro fresh
bit 12 ESKF baro reference ready

bit 13 physical LIDAR connected
bit 14 physical LIDAR distance valid
bit 15 ESKF lidar fresh
bit 16 ESKF lidar reference ready

bit 17 origin zeroed
bit 18 horizontal position valid
bit 19 vertical position valid

bit 20 ESKF float state finite
bit 21 all checked ESKF/attitude boolean fields are actually 0/1

bit 22 SD capture ring is inside expected CCM region
bit 23 SD logger ready
bit 24 SD logging active
bit 25 V8.15C RAM magic still correct
bit 26 physical barometer pressure valid
bit 27 physical LIDAR distance valid

BARO TEST
---------
When barometer is physically connected and healthy, expected:
- flags bit9  = 1
- flags bit10 = 1
- flags bit26 = 1
- baro_pressure_pa near local atmospheric pressure
- baro_temperature_mC plausible
- baro_age_us small
- after ESKF reference acquisition:
  bit11 = 1
  bit12 = 1

When barometer is unplugged:
- bit9,10,11,12,26 should be 0
- pressure word may be 0x80000000
- baro_age_us may become large/FFFFFFFF

IMPORTANT
---------
If raw memory at 0x1000F000 is correct while Live Expressions is nonsense:
the fault is debugger/DWARF display, NOT estimator RAM.

If raw memory itself contains impossible values with correct magic/sequence/
checksum:
the corruption is actually happening inside firmware and the next step is
per-buffer canaries / write-watch instrumentation.
