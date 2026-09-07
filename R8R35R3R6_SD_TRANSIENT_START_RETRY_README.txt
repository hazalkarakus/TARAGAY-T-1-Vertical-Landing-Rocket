TARAGAY-T1 R8R35R3R6 - SD TRANSIENT START RETRY

Purpose:
Keep the P71 known-good SD logger and R3R3 phase-retry service policy, while
preventing one proven powered-needle SDIO DMA START glitch from killing the
logger immediately during flight.

Evidence basis:
- Powered needle path: SD stays healthy pre-PE9, then one DMA start error appears
  after needle motion and the R3R5 firmware shuts logging down.
- Needle motor power disconnected: same firmware, PE9 and many RCS pulses run
  with SD ready/logging maintained, no drops/overruns/errors.

R3R6 software change:
- ONLY a DMA *start* failure may be retried during flight.
- The same writer buffer and the same sector address are retained.
- 5 ms backoff between attempts.
- Maximum 8 in-flight start retries.
- No SD card re-enumeration in flight.
- No full host re-init in flight.
- No HAL_DMA_Abort() is introduced in the in-flight start-retry path.
- Existing transfer-timeout / active-transfer-error fail-visible behavior is
  preserved.

Unchanged:
- P71 SD/FatFs/SDIO data path and adaptive ring drain.
- 200 Hz V14 / 384-byte capture.
- R3R3 scheduler phase-retry admission.
- Needle controller, RCS, TaragayFlightLogic, Full-State ESKF, scheduler.

IMPORTANT:
This is a software containment layer for a measured electrical transient. It is
not a substitute for fixing motor/driver EMI and supply/ground integrity. Keep
DO_NOT_FLY status until the powered-needle test and subsequent long soak pass.
