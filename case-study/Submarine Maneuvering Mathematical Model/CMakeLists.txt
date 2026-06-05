cmake_minimum_required(VERSION 3.18)
project(submarine_maneuvering_sim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Eigen ----------------------------------------------------------------
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# ---- ctrl_toolbox core sources (relative paths) ---------------------------
set(CTRL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../lib")

file(GLOB CTRL_SOURCES "${CTRL_ROOT}/*.cpp")

# ---- Submarine sim target -------------------------------------------------
add_executable(submarine_sim
    sim/src/main.cpp
    sim/src/submarine_plant.cpp
    sim/src/controllers.cpp
    sim/src/simulation_runner.cpp
    ${CTRL_SOURCES}
)

target_include_directories(submarine_sim PRIVATE
    sim/include
    ${CTRL_ROOT}
)

target_link_libraries(submarine_sim PRIVATE
    Eigen3::Eigen
)

target_compile_definitions(submarine_sim PRIVATE
    SUB_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# Windows: silence MSVC warnings for external headers
if(MSVC)
    target_compile_options(submarine_sim PRIVATE /W3 /wd4244 /wd4267)
endif()
