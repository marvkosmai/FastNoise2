#include <benchmark/benchmark.h>
#include "FastNoise/FastNoise.h"
#include "FastNoise/Metadata.h"

#include "../NoiseTool/DemoNodeTrees.inl"

#include "magic_enum.h"

static const size_t gPositionCount = 8192;
static float gPositionFloats[gPositionCount];

FastNoise::SmartNode<> BuildGenerator( benchmark::State& state, const FastNoise::Metadata* metadata, FastSIMD::eLevel level )
{    
    FastNoise::SmartNode<> generator = metadata->CreateNode( level );

    FastNoise::SmartNode<> source = FastNoise::New<FastNoise::Constant>( level );

    for( const auto& memberNode : metadata->memberNodeLookups )
    {
        if( !memberNode.setFunc( generator.get(), source ) )
        {
            // If constant source is not valid try all other node types in order
            for( const FastNoise::Metadata* tryMetadata : FastNoise::Metadata::GetAll() )
            {
                FastNoise::SmartNode<> trySource = tryMetadata->CreateNode( level );

                // Other node types may also have sources
                if( memberNode.setFunc( generator.get(), trySource ) )
                {
                    for( const auto& tryMemberNode : tryMetadata->memberNodeLookups )
                    {
                        if( !tryMemberNode.setFunc( trySource.get(), source ) )
                        {
                            state.SkipWithError( "Could not set valid sources for generator" );
                            return {};
                        }                        
                    }
                    break;
                }
            }
        }
    }
    return generator;
}

void BenchFastNoiseGenerator2D( benchmark::State& state, const FastNoise::SmartNode<> generator )
{
    if (!generator) return;

    float* data = new float[gPositionCount];
    size_t totalData = 0;
    int seed = 0;

    for( auto _ : state )
    {
        (void)_;
        generator->GenPositionArray2D( data, gPositionCount, gPositionFloats, gPositionFloats, 0, 0, seed++ );
        totalData += gPositionCount;
    }

    delete[] data;
    state.SetItemsProcessed( totalData );
}

void BenchFastNoiseGenerator3D( benchmark::State& state, const FastNoise::SmartNode<> generator )
{
    if (!generator) return;

    float* data = new float[gPositionCount];
    size_t totalData = 0;
    int seed = 0;

    for( auto _ : state )
    {
        (void)_;
        generator->GenPositionArray3D( data, gPositionCount, gPositionFloats, gPositionFloats, gPositionFloats, 0, 0, 0, seed++ );
        totalData += gPositionCount;
    }

    delete[] data;
    state.SetItemsProcessed( totalData );
}

void BenchFastNoiseGenerator4D( benchmark::State& state, const FastNoise::SmartNode<> generator )
{
    if (!generator) return;

    float* data = new float[gPositionCount];
    size_t totalData = 0;
    int seed = 0;

    for( auto _ : state )
    {
        (void)_;
        generator->GenPositionArray4D( data, gPositionCount, gPositionFloats, gPositionFloats, gPositionFloats, gPositionFloats, 0, 0, 0, 0, seed++ );
        totalData += gPositionCount;
    }

    delete[] data;
    state.SetItemsProcessed( totalData );
}

template<typename T>
void RegisterBenchmarks( FastSIMD::eLevel level, const char* groupName, const char* name, T generatorFunc )
{
    std::string benchName = "0D/";

#ifdef MAGIC_ENUM_SUPPORTED
    auto enumName = magic_enum::flags::enum_name( level );
    auto find = enumName.find( '_' );
    if( find != std::string::npos )
    {
        benchName += enumName.data() + find + 1;
    }
    else
    {
        benchName += enumName;
    }
#else
    benchName += std::to_string( (int)level );
#endif

    return;

    benchName += '/';
    benchName += groupName;
    benchName += '/';
    benchName += name;

    benchName[0] = '4';
    benchmark::RegisterBenchmark( benchName.c_str(), [=]( benchmark::State& st ) { BenchFastNoiseGenerator4D( st, generatorFunc( st ) ); } );

    benchName[0] = '3';
    benchmark::RegisterBenchmark( benchName.c_str(), [=]( benchmark::State& st ) { BenchFastNoiseGenerator3D( st, generatorFunc( st ) ); } );

    benchName[0] = '2';
    benchmark::RegisterBenchmark( benchName.c_str(), [=]( benchmark::State& st ) { BenchFastNoiseGenerator2D( st, generatorFunc( st ) ); } );
}

int main( int argc, char** argv )
{
    benchmark::Initialize( &argc, argv );

    for( size_t idx = 0; idx < gPositionCount; idx++ )
    {
        gPositionFloats[idx] = (float)idx * 0.6f;
    }
    
    for( FastSIMD::eLevel level = FastSIMD::CPUMaxSIMDLevel(); level != FastSIMD::Level_Null; level = (FastSIMD::eLevel)(level >> 1) )
    {
        if( !(level & FastSIMD::COMPILED_SIMD_LEVELS & FastNoise::SUPPORTED_SIMD_LEVELS) )
        {
            continue;
        }

        for( const FastNoise::Metadata* metadata : FastNoise::Metadata::GetAll() )
        {
            const char* groupName = "Misc";

            if( !metadata->groups.empty() )
            {
                groupName = metadata->groups[metadata->groups.size() - 1];
            }

            std::string nodeName = FastNoise::Metadata::FormatMetadataNodeName( metadata, false );

           RegisterBenchmarks( level, groupName, nodeName.c_str(), [=]( benchmark::State& st ) { return BuildGenerator( st, metadata, level ); } );
        }

        for( const auto& nodeTree : gDemoNodeTrees )
        {
            RegisterBenchmarks( level, "Node Trees", nodeTree[0], [=]( benchmark::State& st )
            {
                FastNoise::SmartNode<> rootNode = FastNoise::NewFromEncodedNodeTree( nodeTree[1], level );

                if( !rootNode )
                {
                    st.SkipWithError( "Could not generate node tree from encoded string" );                    
                }

                return rootNode;
            } );
            
        }
    }

    benchmark::RunSpecifiedBenchmarks();

    return 0;
}

#include "FastSIMD/FastSIMD.h"
#include "../src/FastSIMD/Internal/AVX.h"
#define FS_SIMD_CLASS FastSIMD::AVX2

namespace Fast
{
    


using float32v = FS_SIMD_CLASS::float32v;
using int32v = FS_SIMD_CLASS::int32v;
using mask32v = FS_SIMD_CLASS::mask32v;
using FS = FS_SIMD_CLASS;

float32v GenFS( int32v seed, float32v x, float32v y, float32v z )
    {
        float32v jitter = float32v( 1.0f );
        std::array<float32v, 4> value;
        std::array<float32v, 4> distance;
        
        value.fill( float32v( INFINITY ) );
        distance.fill( float32v( INFINITY ) );
        
        int32v xc = FS_Convertf32_i32( x ) + int32v( -1 );
        int32v ycBase = FS_Convertf32_i32( y ) + int32v( -1 );
        int32v zcBase = FS_Convertf32_i32( z ) + int32v( -1 );
        
        float32v xcf = FS_Converti32_f32( xc ) - x;
        float32v ycfBase = FS_Converti32_f32( ycBase ) - y;
        float32v zcfBase = FS_Converti32_f32( zcBase ) - z;
    
        xc *= int32v( 1337 );
        ycBase *= int32v( 130879 );
        zcBase *= int32v( 987098 );
    
        for( int xi = 0; xi < 3; xi++ )
        {
            float32v ycf = ycfBase;
            int32v yc = ycBase;
            for( int yi = 0; yi < 3; yi++ )
            {
                float32v zcf = zcfBase;
                int32v zc = zcBase;
                for( int zi = 0; zi < 3; zi++ )
                {
                    int32v hash = seed;
                    hash ^= xc ^ yc ^ zc;            
                    hash *= int32v( 0x27d4eb2d );

                    float32v xd = FS_Converti32_f32( hash & int32v( 0x3ff ) ) - float32v( 0x3ff / 2.0f );
                    float32v yd = FS_Converti32_f32( ( hash >> 10 ) & int32v( 0x3ff ) ) - float32v( 0x3ff / 2.0f );
                    float32v zd = FS_Converti32_f32( ( hash >> 20 ) & int32v( 0x3ff ) ) - float32v( 0x3ff / 2.0f );
                
                    float32v invMag = jitter * FS_InvSqrt_f32( FS_FMulAdd_f32( xd, xd, FS_FMulAdd_f32( yd, yd, zd * zd ) ) );
                    xd = FS_FMulAdd_f32( xd, invMag, xcf );
                    yd = FS_FMulAdd_f32( yd, invMag, ycf );
                    zd = FS_FMulAdd_f32( zd, invMag, zcf );
                
                    float32v newCellValue = float32v( (float)(1.0 / INT_MAX) ) * FS_Converti32_f32( hash );
                    float32v newDistance = xd * xd + yd * yd + zd * zd;
                
                    for( int i = 0; ; i++ )
                    {
                        mask32v closer = newDistance < distance[i];

                        float32v localDistance = distance[i];
                        float32v localCellValue = value[i];

                        distance[i] = FS_Select_f32( closer, newDistance, distance[i] );
                        value[i] = FS_Select_f32( closer, newCellValue, value[i] );

                        if( i > 2 )
                        {
                            break;
                        }

                        newDistance = FS_Select_f32( closer, localDistance, newDistance );
                        newCellValue = FS_Select_f32( closer, localCellValue, newCellValue );
                    }
            
                    zcf += float32v( 1 );
                    zc += int32v( 987098 );
                }
                ycf += float32v( 1 );
                yc += int32v( 130879 );
            }
            xcf += float32v( 1 );
            xc += int32v( 1337 );
        }
    
        return value[2];
    }

static void BenchFastSIMD(benchmark::State& state)
{
    float* data = new float[gPositionCount];
    size_t totalData = 0;
    int seed = 0;

    for( auto _ : state )
    {
        (void)_;
        for(size_t i = 0; i < gPositionCount; i+=8)
        {
            int32v seed = FS_Load_i32( gPositionFloats + i );
            float32v pos = FS_Load_f32( gPositionFloats + i );
            FS_Store_f32( data + i, GenFS( seed, pos, pos, pos ) );
        }
        totalData += gPositionCount;
    }

    delete[] data;
    state.SetItemsProcessed( totalData );
}

BENCHMARK(BenchFastSIMD)->Repetitions( 9 );
}


#define SIMDPP_ARCH_X86_AVX2 1
#define SIMDPP_ARCH_X86_FMA3 1

#include "simdpp/simd.h"

using float32v = simdpp::float32<32>; // CHANGE THIS: 8 = single vector length
using int32v = simdpp::int32<float32v::length>;
using mask32v = simdpp::mask_float32<float32v::length>;

float32v GenSimdPP( int32v seed, float32v x, float32v y, float32v z )
    {
        float32v jitter = simdpp::make_float( 1.0f );
        std::array<float32v, 4> value;
        std::array<float32v, 4> distance;
        
        value.fill( simdpp::make_float( INFINITY ) );
        distance.fill( simdpp::make_float( INFINITY ) );
        
        int32v xc = simdpp::to_int32( x ) + simdpp::make_int<int32v>( -1 );
        int32v ycBase = simdpp::to_int32( y ) + simdpp::make_int<int32v>( -1 );
        int32v zcBase = simdpp::to_int32( z ) + simdpp::make_int<int32v>( -1 );
        
        float32v xcf = to_float32( xc ) - x;
        float32v ycfBase = to_float32( ycBase ) - y;
        float32v zcfBase = to_float32( zcBase ) - z;
    
        xc = xc * simdpp::make_int<int32v>( 1337 );
        ycBase = ycBase * simdpp::make_int<int32v>( 130879 );
        zcBase = zcBase * simdpp::make_int<int32v>( 987098 );
    
        for( int xi = 0; xi < 3; xi++ )
        {
            float32v ycf = ycfBase;
            int32v yc = ycBase;
            for( int yi = 0; yi < 3; yi++ )
            {
                float32v zcf = zcfBase;
                int32v zc = zcBase;
                for( int zi = 0; zi < 3; zi++ )
                {
                    int32v hash = seed;
                    hash = hash ^ xc ^ yc ^ zc;            
                    hash = hash * simdpp::make_int<int32v>( 0x27d4eb2d );

                    float32v xd = to_float32( hash & simdpp::make_int<int32v>( 0x3ff ) ) - simdpp::make_float<float32v>( 0x3ff / 2.0f );
                    float32v yd = to_float32( ( hash >> 10 ) & simdpp::make_int<int32v>( 0x3ff ) ) - simdpp::make_float<float32v>( 0x3ff / 2.0f );
                    float32v zd = to_float32( ( hash >> 20 ) & simdpp::make_int<int32v>( 0x3ff ) ) - simdpp::make_float<float32v>( 0x3ff / 2.0f );
                
                    float32v invMag = jitter * simdpp::rsqrt_e( simdpp::fmadd( xd, xd, simdpp::fmadd( yd, yd, zd * zd ) ) );
                    xd = simdpp::fmadd( xd, invMag, xcf );
                    yd = simdpp::fmadd( yd, invMag, ycf );
                    zd = simdpp::fmadd( zd, invMag, zcf );
                
                    float32v newCellValue = simdpp::make_float<float32v>( (float)(1.0 / INT_MAX) ) * to_float32( hash );
                    float32v newDistance = xd * xd + yd * yd + zd * zd;
                
                    for( int i = 0; ; i++ )
                    {
                        mask32v closer = newDistance < distance[i];

                        float32v localDistance = distance[i];
                        float32v localCellValue = value[i];

                        distance[i] = simdpp::blend( closer, newDistance, distance[i] );
                        value[i] = simdpp::blend( closer, newCellValue, value[i] );

                        if( i > 2 )
                        {
                            break;
                        }

                        newDistance = simdpp::blend( closer, localDistance, newDistance );
                        newCellValue = simdpp::blend( closer, localCellValue, newCellValue );
                    }
            
                    zcf = zcf + simdpp::make_float<float32v>( 1 );
                    zc = zc + simdpp::make_int<int32v>( 987098 );
                }
                ycf = ycf + simdpp::make_float<float32v>( 1 );
                yc = yc + simdpp::make_int<int32v>( 130879 );
            }
            xcf = xcf + simdpp::make_float<float32v>( 1 );
            xc = xc + simdpp::make_int<int32v>( 1337 );
        }
    
        return value[2];
    }

static void BenchSIMDpp(benchmark::State& state)
{
    float* data = new float[gPositionCount];
    size_t totalData = 0;
    int seed = 0;

    for( auto _ : state )
    {
        (void)_;
        for(size_t i = 0; i < gPositionCount; i+=int32v::length)
        {
            int32v seed = simdpp::bit_cast<int32v>( float32v( simdpp::load_u( gPositionFloats + i ) ) );
            float32v pos = simdpp::load_u( gPositionFloats + i );
            simdpp::store_u( data + i, GenSimdPP( seed, pos, pos, pos ) );
        }
        totalData += gPositionCount;
    }

    delete[] data;
    state.SetItemsProcessed( totalData );
}

BENCHMARK(BenchSIMDpp)->Repetitions( 9 );