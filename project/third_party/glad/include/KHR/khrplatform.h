#ifndef __khrplatform_h_
#define __khrplatform_h_

/*
** Copyright (c) 2008-2018 The Khronos Group Inc.
** (shortened for brevity)
*/

/* Platform-specific types and definitions for OpenGL */
/* This is a minimal version sufficient for GLAD. */

#ifdef _WIN32
#   if defined(_MSC_VER) && !defined(__clang__)
        typedef signed   __int8  khronos_int8_t;
        typedef unsigned __int8  khronos_uint8_t;
        typedef signed   __int16 khronos_int16_t;
        typedef unsigned __int16 khronos_uint16_t;
        typedef signed   __int32 khronos_int32_t;
        typedef unsigned __int32 khronos_uint32_t;
        typedef signed   __int64 khronos_int64_t;
        typedef unsigned __int64 khronos_uint64_t;
#   else
#       include <stdint.h>
        typedef int8_t    khronos_int8_t;
        typedef uint8_t   khronos_uint8_t;
        typedef int16_t   khronos_int16_t;
        typedef uint16_t  khronos_uint16_t;
        typedef int32_t   khronos_int32_t;
        typedef uint32_t  khronos_uint32_t;
        typedef int64_t   khronos_int64_t;
        typedef uint64_t  khronos_uint64_t;
#   endif
#else
#   include <stdint.h>
    typedef int8_t    khronos_int8_t;
    typedef uint8_t   khronos_uint8_t;
    typedef int16_t   khronos_int16_t;
    typedef uint16_t  khronos_uint16_t;
    typedef int32_t   khronos_int32_t;
    typedef uint32_t  khronos_uint32_t;
    typedef int64_t   khronos_int64_t;
    typedef uint64_t  khronos_uint64_t;
#endif

typedef float   khronos_float_t;
typedef int32_t khronos_intptr_t;
typedef uint32_t khronos_uintptr_t;

#define KHRONOS_SUPPORT_INT64   1
#define KHRONOS_SUPPORT_FLOAT   1

#endif /* __khrplatform_h_ */