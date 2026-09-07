#ifndef RTWTYPES_H
#define RTWTYPES_H

#include <stdint.h>

typedef double real_T;
typedef float real32_T;
typedef double real64_T;
typedef double time_T;
typedef char char_T;
typedef unsigned char uchar_T;
typedef int8_t int8_T;
typedef uint8_t uint8_T;
typedef int16_t int16_T;
typedef uint16_t uint16_T;
typedef int32_t int32_T;
typedef uint32_t uint32_T;
typedef int64_t int64_T;
typedef uint64_t uint64_T;
typedef uint8_T boolean_T;
typedef void *pointer_T;

#define MAX_int8_T   INT8_MAX
#define MIN_int8_T   INT8_MIN
#define MAX_uint8_T  UINT8_MAX
#define MAX_int16_T  INT16_MAX
#define MIN_int16_T  INT16_MIN
#define MAX_uint16_T UINT16_MAX
#define MAX_int32_T  INT32_MAX
#define MIN_int32_T  INT32_MIN
#define MAX_uint32_T UINT32_MAX

#ifndef false
#define false (0U)
#endif
#ifndef true
#define true (1U)
#endif

#endif
