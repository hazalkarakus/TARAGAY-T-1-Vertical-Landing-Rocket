#ifndef APP_COMMON_FILTERS_BUTTERWORTH_FILTER_H
#define APP_COMMON_FILTERS_BUTTERWORTH_FILTER_H

#include <stdint.h>

typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;

    float z1;
    float z2;

    uint8_t configured;
    uint8_t initialized;

} Butterworth2LPF_t;

uint8_t Butterworth2LPF_Init(
    Butterworth2LPF_t *filter,
    float sample_rate_hz,
    float cutoff_hz
);

void Butterworth2LPF_Reset(
    Butterworth2LPF_t *filter,
    float initial_value
);

float Butterworth2LPF_Process(
    Butterworth2LPF_t *filter,
    float input
);

uint8_t Butterworth2LPF_IsConfigured(
    const Butterworth2LPF_t *filter
);

#endif
