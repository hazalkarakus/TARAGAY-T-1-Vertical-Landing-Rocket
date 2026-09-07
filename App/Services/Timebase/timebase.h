#ifndef TIMEBASE_H
#define TIMEBASE_H

#include "main.h"
#include <stdint.h>

void Timebase_Init(void);

uint32_t millis(void);
uint32_t micros(void);

uint8_t Timebase_HasElapsedMs(
    uint32_t now,
    uint32_t previous,
    uint32_t interval_ms
);

uint8_t Timebase_HasElapsedUs(
    uint32_t now,
    uint32_t previous,
    uint32_t interval_us
);

#endif
