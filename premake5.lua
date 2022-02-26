project "FastNoise2"
    kind "StaticLib"
    language "C++"

    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

    includedirs
	{
        "include"
    }

    files
    {
        "include/FastSIMD/*.h",
        "src/FastSIMDD/Internal/*.h",
        "src/FastSIMD/*.cpp",
        "include/FastNoise/*.h",
        "include/FastNoise/Generators/*.h",
        "src/FastNoise/*.cpp"
    }

    defines
    {
        "FASTNOISE_STATIC_LIB",
        "FASTNOISE_EXPORT"
    }

    buildoptions
    {
        "/arch:AVX",
        "/arch:AVX512"
    }

    filter "system:windows"
        systemversion "latest"
        cppdialect "C++17"
        staticruntime "Off"
        runtime "Debug"
    
    filter { "system:windows", "configurations:Release" }
        staticruntime "Off"
        runtime "Release"
