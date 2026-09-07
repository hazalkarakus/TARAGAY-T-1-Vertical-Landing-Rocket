#include "Common/Filters/butterworth_filter.h"

#include <math.h>

#define BUTTERWORTH_PI_F       3.14159265358979323846f
#define BUTTERWORTH_SQRT2_F    1.41421356237309504880f

uint8_t Butterworth2LPF_Init(
    Butterworth2LPF_t *filter,
    float sample_rate_hz,
    float cutoff_hz
)
{
    float k;
    float normalization;

    if (filter == 0)
    {
        return 0U;
    }

    filter->b0 = 0.0f;
    filter->b1 = 0.0f;
    filter->b2 = 0.0f;
    filter->a1 = 0.0f;
    filter->a2 = 0.0f;

    filter->z1 = 0.0f;
    filter->z2 = 0.0f;

    filter->configured = 0U;
    filter->initialized = 0U;

    if ((sample_rate_hz <= 0.0f) ||
        (cutoff_hz <= 0.0f) ||
        (cutoff_hz >= (0.5f * sample_rate_hz)))
    {
        return 0U;
    }

    /*
     * Bilinear transform with frequency pre-warping.
     * This produces a normalized second-order Butterworth low-pass biquad.
     */
    k = tanf(BUTTERWORTH_PI_F * cutoff_hz / sample_rate_hz);

    normalization =
        1.0f /
        (1.0f +
         (BUTTERWORTH_SQRT2_F * k) +
         (k * k));

    filter->b0 = (k * k) * normalization;
    filter->b1 = 2.0f * filter->b0;
    filter->b2 = filter->b0;

    filter->a1 =
        2.0f * ((k * k) - 1.0f) * normalization;

    filter->a2 =
        (1.0f -
         (BUTTERWORTH_SQRT2_F * k) +
         (k * k)) * normalization;

    filter->configured = 1U;

    return 1U;
}

void Butterworth2LPF_Reset(
    Butterworth2LPF_t *filter,
    float initial_value
)
{
    if ((filter == 0) ||
        (filter->configured == 0U))
    {
        return;
    }

    /*
     * Direct Form II Transposed steady-state initialization.
     * A constant input therefore starts without a filter startup impulse.
     */
    filter->z1 =
        (1.0f - filter->b0) * initial_value;

    filter->z2 =
        (filter->b2 - filter->a2) * initial_value;

    filter->initialized = 1U;
}

float Butterworth2LPF_Process(
    Butterworth2LPF_t *filter,
    float input
)
{
    float output;
    float next_z1;
    float next_z2;

    if ((filter == 0) ||
        (filter->configured == 0U))
    {
        return input;
    }

    if (filter->initialized == 0U)
    {
        Butterworth2LPF_Reset(filter, input);
        return input;
    }

    output =
        (filter->b0 * input) +
        filter->z1;

    next_z1 =
        (filter->b1 * input) -
        (filter->a1 * output) +
        filter->z2;

    next_z2 =
        (filter->b2 * input) -
        (filter->a2 * output);

    filter->z1 = next_z1;
    filter->z2 = next_z2;

    return output;
}

uint8_t Butterworth2LPF_IsConfigured(
    const Butterworth2LPF_t *filter
)
{
    if (filter == 0)
    {
        return 0U;
    }

    return filter->configured;
}
