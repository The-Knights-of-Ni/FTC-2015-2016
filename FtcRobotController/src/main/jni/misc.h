#ifndef MISC
#define MISC

#include <stdint.h>
#include <stdio.h>

#define private public
#define class struct
#define new 0; assert(0 && "don't use new/delete, use malloc/free or the direct system calls");//

#define min(a, b) (((a) < (b))?(a):(b))
#define max(a, b) (((a) > (b))?(a):(b))
#define clamp(a, low, high) (max(low, min(a, high)))

typedef unsigned int uint;

typedef uint8_t byte;
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;
typedef uint8 bool8;

typedef float float32;
typedef double float64;

#define ever (;;)

#define len(array) sizeof(array)/sizeof((array)[0])

#ifdef __clang__
#define crash __builtin_trap()
#endif

#ifdef DEBUG
#define assert(this_is_true) if(!(this_is_true)) {printf("failed assertion: %s", #this_is_true); exit(EXIT_SUCCESS);}
#else
#define assert(this_is_true)
#endif

#define kilobyte 1024
#define megabyte 1024*1024
#define gigabyte 1024*1024*1024

#endif
