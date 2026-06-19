set(SIM_INC "${CMAKE_CURRENT_SOURCE_DIR}/sim/include")
set(SIM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/sim/src")

set(SIM_SRCS
    ${SIM_SRC}/humid_plant.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/telemetry_logger.cpp
    ${SIM_SRC}/simulation_runner.cpp
    ${SIM_SRC}/main.cpp
)

add_executable(humidification_sim ${SIM_SRCS})

target_include_directories(humidification_sim PRIVATE
    ${SIM_INC}
)
target_link_libraries(humidification_sim PRIVATE
    controller_toolbox
    nlohmann_json::nlohmann_json
)
target_compile_features(humidification_sim PRIVATE cxx_std_20)
target_compile_definitions(humidification_sim PRIVATE
    HUMID_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# -- Robustness analysis (fault sweep, Monte Carlo, WCET) --------------------
add_executable(humidification_robustness
    ${SIM_SRC}/humid_plant.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/robustness_main.cpp
)
target_include_directories(humidification_robustness PRIVATE
    ${SIM_INC}
    ${CMAKE_SOURCE_DIR}/case-study/common
)
target_link_libraries(humidification_robustness PRIVATE
    controller_toolbox
    nlohmann_json::nlohmann_json
)
target_compile_features(humidification_robustness PRIVATE cxx_std_20)
target_compile_definitions(humidification_robustness PRIVATE
    HUMID_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
