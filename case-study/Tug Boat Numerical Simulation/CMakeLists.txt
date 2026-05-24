include(FetchContent)

# nlohmann/json — single-header download; included as "json.hpp" in sim sources
FetchContent_Declare(
    nlohmann_json_single
    URL      https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp
    DOWNLOAD_NO_EXTRACT TRUE
    DOWNLOAD_DIR "${CMAKE_CURRENT_BINARY_DIR}/json_include"
)
FetchContent_MakeAvailable(nlohmann_json_single)

set(SIM_INC "${CMAKE_CURRENT_SOURCE_DIR}/sim/include")
set(SIM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/sim/src")

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

add_executable(tug_sim ${SIM_SRCS})

target_include_directories(tug_sim PRIVATE
    ${SIM_INC}
    "${CMAKE_CURRENT_BINARY_DIR}/json_include"
)
target_link_libraries(tug_sim PRIVATE controller_toolbox)
target_compile_features(tug_sim PRIVATE cxx_std_17)
target_compile_definitions(tug_sim PRIVATE
    TUG_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
