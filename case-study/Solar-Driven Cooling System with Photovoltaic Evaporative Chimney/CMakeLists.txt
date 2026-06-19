set(SIM_INC "${CMAKE_CURRENT_SOURCE_DIR}/sim/include")
set(SIM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/sim/src")

set(SIM_SRCS
    ${SIM_SRC}/solar_plant.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/telemetry_logger.cpp
    ${SIM_SRC}/simulation_runner.cpp
    ${SIM_SRC}/main.cpp
)

add_executable(solar_cooling_sim ${SIM_SRCS})

target_include_directories(solar_cooling_sim PRIVATE
    ${SIM_INC}
)
target_link_libraries(solar_cooling_sim PRIVATE 
    controller_toolbox
    nlohmann_json::nlohmann_json
)
target_compile_features(solar_cooling_sim PRIVATE cxx_std_20)
target_compile_definitions(solar_cooling_sim PRIVATE
    SOLAR_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
