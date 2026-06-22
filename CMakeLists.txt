cmake_minimum_required(VERSION 3.16...3.31)
project(ControllerToolbox VERSION 1.0.0 LANGUAGES CXX)
enable_testing()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Disable experimental C++20 module dependency scanning (CMake 3.28+ CMP0155 NEW).
# This project uses standard #include headers - GCC/MinGW's -fmodules-ts scanner
# crashes on paths with spaces on Windows. Harmless on older CMake versions.
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

# Absolute path to the root data/ directory, injected as a compile definition so
# all CSV-generating binaries write to the same place regardless of CWD at runtime.
add_compile_definitions(PROJECT_DATA_DIR="${CMAKE_SOURCE_DIR}/data")

# MSVC does not define M_PI and other POSIX math constants by default.
# _USE_MATH_DEFINES exposes them from <cmath> on Windows.
if(MSVC)
    add_compile_definitions(_USE_MATH_DEFINES)
endif()

# Global fallback definition of M_PI - only needed on non-MSVC where _USE_MATH_DEFINES
# is not set, and only when the compiler doesn't already provide it (MinGW, older GCC).
if(NOT MSVC)
    add_compile_definitions(M_PI=3.14159265358979323846)
endif()

# MinGW (MSYS2 UCRT64/MINGW64) dynamically links libgcc/libstdc++/libwinpthread by default.
# Those DLLs live under msys64/<env>/bin, which isn't on PATH outside an MSYS2 shell --
# and Python's loader for .pyd extensions ignores PATH entirely since 3.8. Link them
# statically so every binary (executables and ctrl_toolbox.pyd) is self-contained.
if(MINGW)
    add_link_options(-static-libgcc -static-libstdc++ -static)
endif()

# --- DIST-2: embedded-only build shortcut ------------------------------------
option(CTRL_BUILD_EMBEDDED_ONLY "Build only the header-only embedded subset (no Eigen)" OFF)
if(CTRL_BUILD_EMBEDDED_ONLY)
    add_library(ctrl_embedded INTERFACE)
    target_include_directories(ctrl_embedded INTERFACE lib/embedded)
    include(GNUInstallDirs)
    install(DIRECTORY lib/embedded/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/controller_toolbox/embedded)
    return()
endif()

# --- Eigen3: prefer system/vcpkg; FetchContent fallback for CI wheel builds --
# Set CTRL_FETCH_EIGEN_IF_MISSING=ON when building PyPI wheels on machines
# that do not have Eigen installed (cibuildwheel Linux/macOS containers).
option(CTRL_FETCH_EIGEN_IF_MISSING "Fetch Eigen3 via FetchContent if not found on the system" OFF)
if(CTRL_FETCH_EIGEN_IF_MISSING)
    find_package(Eigen3 QUIET NO_MODULE)
    if(NOT Eigen3_FOUND)
        message(STATUS "Eigen3 not found on system -- fetching via FetchContent")
        include(FetchContent)
        FetchContent_Declare(
            Eigen3
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG        3.4.0
            GIT_SHALLOW    TRUE
        )
        set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
        set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(Eigen3)
    endif()
endif()

add_subdirectory(lib)

# nlohmann/json - required by case-study executables and test_tugsim_regression.
# Prefer a system/vcpkg installation; fall back to FetchContent so
# the build works out-of-the-box without pre-installed packages.
# Must be fetched BEFORE add_subdirectory(tests) and add_subdirectory(case-study)
# so that nlohmann_json::nlohmann_json is a valid target in both.
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

add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(scripts)
add_subdirectory(case-study)

option(CTRL_BUILD_BENCHMARKS "Build latency benchmarks (bench_controllers)" OFF)
if(CTRL_BUILD_BENCHMARKS)
    add_subdirectory(benchmark)
endif()

# Python bindings via pybind11.
# Enable:  cmake .. -DCTRL_BUILD_PYTHON_BINDINGS=ON
# Requires Python 3.8+ development headers and pybind11 (conda: pybind11; vcpkg: pybind11).
option(CTRL_BUILD_PYTHON_BINDINGS "Build pybind11 Python bindings (ctrl_toolbox module)" OFF)
if(CTRL_BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings)
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