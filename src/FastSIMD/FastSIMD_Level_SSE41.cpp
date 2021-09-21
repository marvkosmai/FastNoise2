#include "FastSIMD/FastSIMD.h"

#define SIMDPP_ARCH_X86_SSE4_1 1

#if FASTSIMD_COMPILE_SSE41
#include "Internal/SSE.h"
#define FS_SIMD_CLASS FastSIMD::SSE41
#include "Internal/SourceBuilder.inl"
#endif
