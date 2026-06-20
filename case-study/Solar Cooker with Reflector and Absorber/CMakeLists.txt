set(SIM_INC "${CMAKE_CURRENT_SOURCE_DIR}/sim/include")
set(SIM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/sim/src")

set(SIM_SRCS
    ${SIM_SRC}/solar_cooker_plant.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/simulation_runner.cpp
    ${SIM_SRC}/main.cpp
)

add_executable(solar_cooker_sim ${SIM_SRCS})

target_include_directories(solar_cooker_sim PRIVATE
    ${SIM_INC}
)
target_link_libraries(solar_cooker_sim PRIVATE
    controller_toolbox
    nlohmann_json::nlohmann_json
)
target_compile_features(solar_cooker_sim PRIVATE cxx_std_20)
target_compile_definitions(solar_cooker_sim PRIVATE
    SOLAR_COOKER_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# -- Robustness analysis (fault sweep, Monte Carlo, WCET) --------------------
add_executable(solar_cooker_robustness
    ${SIM_SRC}/solar_cooker_plant.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/robustness_main.cpp
)
target_include_directories(solar_cooker_robustness PRIVATE
    ${SIM_INC}
    ${CMAKE_SOURCE_DIR}/case-study/common
)
target_link_libraries(solar_cooker_robustness PRIVATE
    controller_toolbox
    nlohmann_json::nlohmann_json
)
target_compile_features(solar_cooker_robustness PRIVATE cxx_std_20)
target_compile_definitions(solar_cooker_robustness PRIVATE
    SOLAR_COOKER_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
