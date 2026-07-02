# CMakeLists.txt - Bouyancy-Driven Airship in Vertical Plane (TEMPLATE)
# Register in case-study/CMakeLists.txt:  add_subdirectory("Bouyancy-Driven Airship in Vertical Plane")
# Add target bouyancy_driven_airship_in_vertical_plan_sim to compile.bat AND compile.sh.

add_executable(bouyancy_driven_airship_in_vertical_plan_sim
    sim/src/main.cpp
    sim/src/bouyancy_driven_airship_in_vertical_plan_plant.cpp
    sim/src/controllers.cpp
    sim/src/simulation_runner.cpp
)
target_include_directories(bouyancy_driven_airship_in_vertical_plan_sim PRIVATE
    sim/include
    ${CMAKE_SOURCE_DIR}/lib
)
target_link_libraries(bouyancy_driven_airship_in_vertical_plan_sim PRIVATE controller_toolbox nlohmann_json::nlohmann_json)
target_compile_definitions(bouyancy_driven_airship_in_vertical_plan_sim PRIVATE
    BOUYANCY_DRIVEN_AIRSHIP_IN_VERTICAL_PLAN_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(bouyancy_driven_airship_in_vertical_plan_sim PRIVATE cxx_std_20)

# -- Robustness analysis (fault sweep, Monte Carlo, WCET) --------------------
add_executable(bouyancy_driven_airship_in_vertical_plan_robustness
    sim/src/robustness_main.cpp
    sim/src/bouyancy_driven_airship_in_vertical_plan_plant.cpp
    sim/src/controllers.cpp
    sim/src/simulation_runner.cpp
)
target_include_directories(bouyancy_driven_airship_in_vertical_plan_robustness PRIVATE
    sim/include
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/case-study/common
)
target_link_libraries(bouyancy_driven_airship_in_vertical_plan_robustness PRIVATE controller_toolbox nlohmann_json::nlohmann_json)
target_compile_definitions(bouyancy_driven_airship_in_vertical_plan_robustness PRIVATE
    BOUYANCY_DRIVEN_AIRSHIP_IN_VERTICAL_PLAN_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(bouyancy_driven_airship_in_vertical_plan_robustness PRIVATE cxx_std_20)
