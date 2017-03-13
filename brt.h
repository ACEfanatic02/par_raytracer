#ifndef _BRT_H_
#define _BRT_H_

#include <cstdlib>
#include <cstdint>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef uintptr_t uptr;
typedef intptr_t iptr;

#define Kilobytes(n) (n * 1024)
#define Megabytes(n) (Kilobytes(n) * 1024)
#define Gigabytes(n) (Megabytes(n) * 1024)

#define PI32 (3.1415927f)
#define DEG2RAD(x) ((x)/180.0f * PI32)

#define GLSL(src) "#version 140\n\n" # src

#include <cstdio>

#ifndef _WIN32
#define __debugbreak(...) ((void)0)
#endif

#ifdef _WIN32
#define snprintf _snprintf
#endif

#undef assert
#define assert(e) ASSERT(e)
#define Assert(e, ...) ASSERT(e)
#define ASSERT(e) do { if (!(e)) { fprintf(stderr, "[%s:%u] Assert failed: %s\n", __FILE__, __LINE__, # e); __debugbreak(); } } while(0)


#define array_count(a) (sizeof((a)) / sizeof((a)[0]))

#define byte_offset(ptr, offset) (((u8 *)ptr) + (offset))

#endif
