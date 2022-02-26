project "FastNoise2"
    kind "StaticLib"
    language "C++"

    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

    files
    {
        "include/FastSIMD/*.h",
        "src/FastSIMDD/Internal/*.h",
        "src/FastSIMD/*.cpp",
        "include/FastNoise/*.h",
        "include/FastNoise/Generators/*.h",
        "src/FastNoise/*.cpp"
    }

    filter "system:windows"
        systemversion "latest"
        cppdialect "C++17"
        staticruntime "On"
    
    filter { "system:windows", "configurations:Release" }
        buildoptions "/MT"
