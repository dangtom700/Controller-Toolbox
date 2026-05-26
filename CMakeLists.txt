cmake_minimum_required(VERSION 3.16)
project(ControllerToolbox CXX)
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
add_subdirectory(case-study)
# benchmarks are intentionally excluded from the default build

find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()