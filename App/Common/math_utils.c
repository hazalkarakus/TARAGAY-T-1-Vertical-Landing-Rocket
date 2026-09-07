#include "Common/math_utils.h"

#include <math.h>

float MathUtils_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

float MathUtils_AbsFloat(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

float MathUtils_LowPassFloat(float previous_value, float new_value, float alpha)
{
    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }

    if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    return previous_value + alpha * (new_value - previous_value);
}

float MathUtils_Vector3Norm(float x, float y, float z)
{
    return sqrtf((x * x) + (y * y) + (z * z));
}
