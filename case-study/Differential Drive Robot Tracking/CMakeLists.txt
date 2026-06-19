# CMakeLists.txt - Differential Drive Robot Tracking (TEMPLATE)
# Register in case-study/CMakeLists.txt:  add_subdirectory("Differential Drive Robot Tracking")
# Add target differential_drive_robot_tracking_sim to compile.bat AND compile.sh.

add_executable(differential_drive_robot_tracking_sim
    sim/src/main.cpp
    sim/src/differential_drive_robot_tracking_plant.cpp
    sim/src/controllers.cpp
    sim/src/simulation_runner.cpp
)
target_include_directories(differential_drive_robot_tracking_sim PRIVATE
    sim/include
    ${CMAKE_SOURCE_DIR}/lib
)
target_link_libraries(differential_drive_robot_tracking_sim PRIVATE controller_toolbox Eigen3::Eigen nlohmann_json::nlohmann_json)
target_compile_definitions(differential_drive_robot_tracking_sim PRIVATE
    DIFFERENTIAL_DRIVE_ROBOT_TRACKING_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(differential_drive_robot_tracking_sim PRIVATE cxx_std_20)
