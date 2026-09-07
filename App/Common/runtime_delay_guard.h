#ifndef RUNTIME_DELAY_GUARD_H
#define RUNTIME_DELAY_GUARD_H

#include <stdint.h>

void RuntimeDelayGuard_Init(void);
void RuntimeDelayGuard_MarkRuntimeStarted(void);
uint32_t RuntimeDelayGuard_GetViolationCount(void);
uint32_t RuntimeDelayGuard_GetLastRequestedMs(void);

#endif
