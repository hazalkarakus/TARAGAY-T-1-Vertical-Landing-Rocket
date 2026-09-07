#ifndef MATH_UTILS_H
#define MATH_UTILS_H

float MathUtils_ClampFloat(float value, float min_value, float max_value);
float MathUtils_AbsFloat(float value);
float MathUtils_LowPassFloat(float previous_value, float new_value, float alpha);
float MathUtils_Vector3Norm(float x, float y, float z);

#endif
