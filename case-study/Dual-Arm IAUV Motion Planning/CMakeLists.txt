# CMakeLists.txt - Dual-Arm IAUV Motion Planning (TEMPLATE)
# Register in case-study/CMakeLists.txt:  add_subdirectory("Dual-Arm IAUV Motion Planning")
# Add target dual_arm_iauv_motion_planning_sim to compile.bat AND compile.sh.

add_executable(dual_arm_iauv_motion_planning_sim
    sim/src/main.cpp
    sim/src/dual_arm_iauv_motion_planning_plant.cpp
    sim/src/controllers.cpp
    sim/src/simulation_runner.cpp
)
target_include_directories(dual_arm_iauv_motion_planning_sim PRIVATE
    sim/include
    ${CMAKE_SOURCE_DIR}/lib
)
target_link_libraries(dual_arm_iauv_motion_planning_sim PRIVATE controller_toolbox Eigen3::Eigen nlohmann_json::nlohmann_json)
target_compile_definitions(dual_arm_iauv_motion_planning_sim PRIVATE
    DUAL_ARM_IAUV_MOTION_PLANNING_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(dual_arm_iauv_motion_planning_sim PRIVATE cxx_std_20)
