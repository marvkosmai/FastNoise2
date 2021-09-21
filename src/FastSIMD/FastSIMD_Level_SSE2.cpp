#include "FastSIMD/FastSIMD.h"


#define SIMDPP_ARCH_X86_SSE2 1

#if FASTSIMD_COMPILE_SSE2
#include "Internal/SSE.h"
#define FS_SIMD_CLASS FastSIMD::SSE2
#include "Internal/SourceBuilder.inl"
#endif
