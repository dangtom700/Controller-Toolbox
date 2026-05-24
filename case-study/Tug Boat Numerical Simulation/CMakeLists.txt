cmake_minimum_required(VERSION 3.20)
project(TugBoatSim CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Paths ─────────────────────────────────────────────────────────────────────
set(TOOLBOX_DIR "${CMAKE_SOURCE_DIR}/../../lib")
set(SIM_INC     "${CMAKE_SOURCE_DIR}/sim/include")
set(SIM_SRC     "${CMAKE_SOURCE_DIR}/sim/src")

# ── Eigen (header-only) ───────────────────────────────────────────────────────
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# ── nlohmann/json (header-only, single file expected at sim/include/json.hpp) ─
# Drop nlohmann/json.hpp into sim/include/ — no CMake package needed.

# ── Toolbox sources ───────────────────────────────────────────────────────────
# Only the .cpp files we actually need (not the full toolbox).
set(TOOLBOX_SRCS
    ${TOOLBOX_DIR}/DiscretePID.cpp
    ${TOOLBOX_DIR}/KalmanFilter.cpp
    ${TOOLBOX_DIR}/DiscreteSMC.cpp
    ${TOOLBOX_DIR}/DiscreteMPC.cpp
    ${TOOLBOX_DIR}/ExtremumSeeker.cpp
    ${TOOLBOX_DIR}/PlantModel.cpp
    ${TOOLBOX_DIR}/FuzzyLogic.cpp
    ${TOOLBOX_DIR}/SystemAnalysis.cpp
    ${TOOLBOX_DIR}/RecursiveLeastSquares.cpp
)

# ── Simulation sources ────────────────────────────────────────────────────────
set(SIM_SRCS
    ${SIM_SRC}/plant_parameters.cpp
    ${SIM_SRC}/environment.cpp
    ${SIM_SRC}/physics_plant.cpp
    ${SIM_SRC}/thrust_allocator.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/telemetry_logger.cpp
    ${SIM_SRC}/simulation_runner.cpp
    ${SIM_SRC}/main.cpp
)

add_executable(tug_sim ${SIM_SRCS} ${TOOLBOX_SRCS})

target_include_directories(tug_sim PRIVATE
    ${SIM_INC}
    ${TOOLBOX_DIR}
)

target_link_libraries(tug_sim PRIVATE Eigen3::Eigen)

# Release build optimisations
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
target_compile_options(tug_sim PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/O2 /W3>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-O3 -Wall -Wextra>
)
