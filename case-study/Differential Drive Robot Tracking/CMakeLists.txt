# CMakeLists.txt - Differential Drive Robot Tracking (FUHAC case study)
# Registered in case-study/CMakeLists.txt via add_subdirectory("Differential Drive Robot Tracking").
# Both targets below are listed in compile.bat AND compile.sh.

add_executable(differential_drive_robot_tracking_sim
    sim/src/main.cpp
    sim/src/differential_drive_robot_tracking_plant.cpp
    sim/src/trajectory.cpp
    sim/src/controllers.cpp
    sim/src/simulation_runner.cpp
)
target_include_directories(differential_drive_robot_tracking_sim PRIVATE
    sim/include
    ${CMAKE_SOURCE_DIR}/lib
)
target_link_libraries(differential_drive_robot_tracking_sim PRIVATE controller_toolbox nlohmann_json::nlohmann_json)
target_compile_definitions(differential_drive_robot_tracking_sim PRIVATE
    DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(differential_drive_robot_tracking_sim PRIVATE cxx_std_20)

# -- Robustness analysis (WCET, Monte Carlo, fault sweep) --------------------
add_executable(differential_drive_robot_tracking_robustness
    sim/src/robustness_main.cpp
    sim/src/differential_drive_robot_tracking_plant.cpp
    sim/src/trajectory.cpp
    sim/src/controllers.cpp
    sim/src/simulation_runner.cpp
)
target_include_directories(differential_drive_robot_tracking_robustness PRIVATE
    sim/include
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}/case-study/common
)
target_link_libraries(differential_drive_robot_tracking_robustness PRIVATE controller_toolbox nlohmann_json::nlohmann_json)
target_compile_definitions(differential_drive_robot_tracking_robustness PRIVATE
    DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(differential_drive_robot_tracking_robustness PRIVATE cxx_std_20)
