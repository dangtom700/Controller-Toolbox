# CMakeLists.txt - Residential Building Comfort SMPC (TEMPLATE)
# Register in case-study/CMakeLists.txt:  add_subdirectory("Residential Building Comfort SMPC")
# Add target residential_building_comfort_smpc_sim to compile.bat AND compile.sh.

add_executable(residential_building_comfort_smpc_sim
    sim/src/main.cpp
    sim/src/residential_building_comfort_smpc_plant.cpp
    sim/src/controllers.cpp
    sim/src/simulation_runner.cpp
)
target_include_directories(residential_building_comfort_smpc_sim PRIVATE
    sim/include
    ${CMAKE_SOURCE_DIR}/lib
)
target_link_libraries(residential_building_comfort_smpc_sim PRIVATE controller_toolbox Eigen3::Eigen nlohmann_json::nlohmann_json)
target_compile_definitions(residential_building_comfort_smpc_sim PRIVATE
    RESIDENTIAL_BUILDING_COMFORT_SMPC_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(residential_building_comfort_smpc_sim PRIVATE cxx_std_20)
