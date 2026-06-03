cmake_minimum_required(VERSION 3.16)

# ---------------------------------------------------------------------------
# Python bindings for Controller Toolbox via pybind11
#
# Enable:  cmake .. -DCTRL_BUILD_PYTHON_BINDINGS=ON
# Install: cmake --install build --component python
#
# The resulting module is imported as:
#   import ctrl_toolbox
#   ctrl_toolbox.features()     # -> {'hinf': True, 'fuzzy': True, ...}
#   pid = ctrl_toolbox.DiscretePID(params, Ts)
# ---------------------------------------------------------------------------

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Prefer a conda/vcpkg/system pybind11; fall back to FetchContent.
find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
    message(STATUS "pybind11 not found via find_package - fetching via FetchContent")
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.13.6
        GIT_SHALLOW    TRUE
    )
    # pybind11 v2.13 uses cmake_minimum_required(VERSION 3.5). CMake 3.27+ warns
    # "Compatibility with CMake < 3.10 will be removed" for any sub-project that
    # declares a minimum below 3.10. Our root already requires 3.16, so the
    # warning is a false alarm - suppress it for the duration of this fetch.
    set(CMAKE_WARN_DEPRECATED OFF)
    FetchContent_MakeAvailable(pybind11)
    set(CMAKE_WARN_DEPRECATED ON)
endif()

pybind11_add_module(ctrl_toolbox
    module.cpp
    plantmodel_bindings.cpp
    controllers_bindings.cpp
    estimation_bindings.cpp
    advanced_bindings.cpp
    analysis_bindings.cpp
)

target_link_libraries(ctrl_toolbox PRIVATE controller_toolbox)
target_include_directories(ctrl_toolbox PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(ctrl_toolbox PRIVATE cxx_std_20)

# Silence MSVC warnings that pybind11 macros trigger.
target_compile_options(ctrl_toolbox PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/wd4127 /wd4244 /wd4267>
)

# Optional: install the .so/.pyd alongside the library headers.
install(TARGETS ctrl_toolbox
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT python
)
