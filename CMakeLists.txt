cmake_minimum_required(VERSION 3.16)
project(ControllerToolbox VERSION 1.0.0 LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Absolute path to the root data/ directory, injected as a compile definition so
# all CSV-generating binaries write to the same place regardless of CWD at runtime.
add_compile_definitions(PROJECT_DATA_DIR="${CMAKE_SOURCE_DIR}/data")

# MSVC does not define M_PI and other POSIX math constants by default.
# _USE_MATH_DEFINES exposes them from <cmath> on Windows.
if(MSVC)
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

# Global fallback definition of M_PI - safe on all platforms
add_compile_definitions(M_PI=3.14159265358979323846)

add_subdirectory(lib)
add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(scripts)

# nlohmann/json - required by case-study executables.
# Prefer a system/vcpkg installation; fall back to FetchContent so
# the build works out-of-the-box without pre-installed packages.
find_package(nlohmann_json CONFIG QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.11.3
        GIT_SHALLOW    TRUE
    )
    set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(nlohmann_json)
endif()

add_subdirectory(case-study)

option(CTRL_BUILD_BENCHMARKS "Build latency benchmarks (bench_controllers)" OFF)
if(CTRL_BUILD_BENCHMARKS)
    add_subdirectory(benchmark)
endif()

find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()