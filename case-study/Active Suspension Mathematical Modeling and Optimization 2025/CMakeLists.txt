set(SIM_INC "${CMAKE_CURRENT_SOURCE_DIR}/sim/include")
set(SIM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/sim/src")

set(SIM_SRCS
    ${SIM_SRC}/susp_plant.cpp
    ${SIM_SRC}/road_profile.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/simulation_runner.cpp
    ${SIM_SRC}/main.cpp
)

add_executable(susp_sim ${SIM_SRCS})

target_include_directories(susp_sim PRIVATE
    ${SIM_INC}
)
target_link_libraries(susp_sim PRIVATE
    controller_toolbox
    nlohmann_json::nlohmann_json
)
target_compile_features(susp_sim PRIVATE cxx_std_20)
target_compile_definitions(susp_sim PRIVATE
    SUSP_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# -- Robustness analysis (fault sweep, Monte Carlo, WCET) --------------------
add_executable(susp_robustness
    ${SIM_SRC}/susp_plant.cpp
    ${SIM_SRC}/road_profile.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/robustness_main.cpp
)
target_include_directories(susp_robustness PRIVATE
    ${SIM_INC}
    ${CMAKE_SOURCE_DIR}/case-study/common
)
target_link_libraries(susp_robustness PRIVATE
    controller_toolbox
    nlohmann_json::nlohmann_json
)
target_compile_features(susp_robustness PRIVATE cxx_std_20)
target_compile_definitions(susp_robustness PRIVATE
    SUSP_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
