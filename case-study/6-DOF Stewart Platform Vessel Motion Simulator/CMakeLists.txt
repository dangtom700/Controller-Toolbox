set(SIM_INC "${CMAKE_CURRENT_SOURCE_DIR}/sim/include")
set(SIM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/sim/src")

set(SIM_SRCS
    ${SIM_SRC}/stewart_plant.cpp
    ${SIM_SRC}/cfd_input_model.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/simulation_runner.cpp
    ${SIM_SRC}/main.cpp
)

add_executable(stewart_sim ${SIM_SRCS})

target_include_directories(stewart_sim PRIVATE
    ${SIM_INC}
)
target_link_libraries(stewart_sim PRIVATE
    controller_toolbox
    nlohmann_json::nlohmann_json
)
target_compile_features(stewart_sim PRIVATE cxx_std_20)
target_compile_definitions(stewart_sim PRIVATE
    STEWART_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# -- Robustness analysis (fault sweep, Monte Carlo, WCET) --------------------
add_executable(stewart_robustness
    ${SIM_SRC}/stewart_plant.cpp
    ${SIM_SRC}/cfd_input_model.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/robustness_main.cpp
)
target_include_directories(stewart_robustness PRIVATE
    ${SIM_INC}
    ${CMAKE_SOURCE_DIR}/case-study/common
)
target_link_libraries(stewart_robustness PRIVATE
    controller_toolbox
    nlohmann_json::nlohmann_json
)
target_compile_features(stewart_robustness PRIVATE cxx_std_20)
target_compile_definitions(stewart_robustness PRIVATE
    STEWART_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
