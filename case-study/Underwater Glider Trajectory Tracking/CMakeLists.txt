# CMakeLists.txt - Underwater Glider Trajectory Tracking (TEMPLATE)
# Register in case-study/CMakeLists.txt:  add_subdirectory("Underwater Glider Trajectory Tracking")
# Add target underwater_glider_trajectory_tracking_sim to compile.bat AND compile.sh.

add_executable(underwater_glider_trajectory_tracking_sim
    sim/src/main.cpp
    sim/src/underwater_glider_trajectory_tracking_plant.cpp
    sim/src/controllers.cpp
    sim/src/simulation_runner.cpp
)
target_include_directories(underwater_glider_trajectory_tracking_sim PRIVATE
    sim/include
    ${CMAKE_SOURCE_DIR}/lib
)
target_link_libraries(underwater_glider_trajectory_tracking_sim PRIVATE controller_toolbox Eigen3::Eigen nlohmann_json::nlohmann_json)
target_compile_definitions(underwater_glider_trajectory_tracking_sim PRIVATE
    UNDERWATER_GLIDER_TRAJECTORY_TRACKING_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(underwater_glider_trajectory_tracking_sim PRIVATE cxx_std_20)
