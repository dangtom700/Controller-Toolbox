cmake_minimum_required(VERSION 3.16...3.31)

# -- Catch2 v3 via FetchContent ---------------------------------------------
include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.4
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

# -- Catch2 pilot test suite ------------------------------------------------
add_executable(test_catch2_pilot test_catch2_pilot.cpp)
target_link_libraries(test_catch2_pilot PRIVATE controller_toolbox Catch2::Catch2WithMain)
target_include_directories(test_catch2_pilot PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# -- Catch2 advanced test suite --------------------------------------------
add_executable(test_catch2_advanced test_catch2_advanced.cpp)
target_link_libraries(test_catch2_advanced PRIVATE controller_toolbox Catch2::Catch2WithMain)
target_include_directories(test_catch2_advanced PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# -- Catch2 auto-scheduling test suite (Part 20) ---------------------------
add_executable(test_autoscheduling test_autoscheduling.cpp)
target_link_libraries(test_autoscheduling PRIVATE controller_toolbox Catch2::Catch2WithMain)
target_include_directories(test_autoscheduling PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# -- Catch2 stability margins regression suite (Part 25 / T6) ---------------
add_executable(test_stability_margins test_stability_margins.cpp)
target_link_libraries(test_stability_margins PRIVATE controller_toolbox Catch2::Catch2WithMain)
target_include_directories(test_stability_margins PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(test_stability_margins PRIVATE cxx_std_20)

# -- Legacy hand-rolled test executables (kept unchanged) -------------------
add_executable(test_controllers test_controllers.cpp)
target_link_libraries(test_controllers PRIVATE controller_toolbox)
target_include_directories(test_controllers PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_tuners_extended test_tuners_extended.cpp)
target_link_libraries(test_tuners_extended PRIVATE controller_toolbox)
target_include_directories(test_tuners_extended PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_integration test_integration.cpp)
target_link_libraries(test_integration PRIVATE controller_toolbox)
target_include_directories(test_integration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(tune_controllers tune_controllers.cpp)
target_link_libraries(tune_controllers PRIVATE controller_toolbox)
target_include_directories(tune_controllers PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# -- Tug Boat S2 SMC regression test (P9-1) ------------------------------------
set(TUG_SIM_ROOT "${CMAKE_SOURCE_DIR}/case-study/Tug Boat Numerical Simulation")
add_executable(test_tugsim_regression
    test_tugsim_regression.cpp
    ${TUG_SIM_ROOT}/sim/src/plant_parameters.cpp
    ${TUG_SIM_ROOT}/sim/src/environment.cpp
    ${TUG_SIM_ROOT}/sim/src/physics_plant.cpp
    ${TUG_SIM_ROOT}/sim/src/thrust_allocator.cpp
    ${TUG_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_tugsim_regression PRIVATE
    ${TUG_SIM_ROOT}/sim/include
)
target_link_libraries(test_tugsim_regression PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_tugsim_regression PRIVATE cxx_std_20)
target_compile_definitions(test_tugsim_regression PRIVATE
    TUG_SIM_SOURCE_DIR="${TUG_SIM_ROOT}"
)

# -- Porous Fiber Plate Humidification regression tests (Part 29) -----------
set(HUMID_SIM_ROOT "${CMAKE_SOURCE_DIR}/case-study/Porous Fiber Plate Humidification System")
add_executable(test_humidification
    test_humidification.cpp
    ${HUMID_SIM_ROOT}/sim/src/humid_plant.cpp
    ${HUMID_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_humidification PRIVATE
    ${HUMID_SIM_ROOT}/sim/include
)
target_link_libraries(test_humidification PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_humidification PRIVATE cxx_std_20)

# -- Boiler case-study controller regression tests (T1a) ---------------------
set(BOILER_SIM_ROOT "${CMAKE_SOURCE_DIR}/case-study/Boiler Control")
add_executable(test_boiler_regression
    test_boiler_regression.cpp
    ${BOILER_SIM_ROOT}/sim/src/boiler_plant.cpp
    ${BOILER_SIM_ROOT}/sim/src/linearizer.cpp
    ${BOILER_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_boiler_regression PRIVATE
    ${BOILER_SIM_ROOT}/sim/include
)
target_link_libraries(test_boiler_regression PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_boiler_regression PRIVATE cxx_std_20)

# -- Solar case-study controller regression tests (T1c) ----------------------
set(SOLAR_SIM_ROOT "${CMAKE_SOURCE_DIR}/case-study/Solar-Driven Cooling System with Photovoltaic Evaporative Chimney")
add_executable(test_solar_regression
    test_solar_regression.cpp
    ${SOLAR_SIM_ROOT}/sim/src/solar_plant.cpp
    ${SOLAR_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_solar_regression PRIVATE
    ${SOLAR_SIM_ROOT}/sim/include
)
target_link_libraries(test_solar_regression PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_solar_regression PRIVATE cxx_std_20)
target_compile_definitions(test_solar_regression PRIVATE
    SOLAR_SIM_SOURCE_DIR="${SOLAR_SIM_ROOT}"
)

# -- Humidification controller regression tests (T1d) ------------------------
add_executable(test_humid_regression
    test_humid_regression.cpp
    ${HUMID_SIM_ROOT}/sim/src/humid_plant.cpp
    ${HUMID_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_humid_regression PRIVATE
    ${HUMID_SIM_ROOT}/sim/include
)
target_link_libraries(test_humid_regression PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_humid_regression PRIVATE cxx_std_20)

# -- Active Suspension regression tests (Part 44) ----------------------------
set(SUSP_SIM_ROOT "${CMAKE_SOURCE_DIR}/case-study/Active Suspension Mathematical Modeling and Optimization 2025")
add_executable(test_susp_regression
    test_susp_regression.cpp
    ${SUSP_SIM_ROOT}/sim/src/susp_plant.cpp
    ${SUSP_SIM_ROOT}/sim/src/road_profile.cpp
    ${SUSP_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_susp_regression PRIVATE ${SUSP_SIM_ROOT}/sim/include)
target_link_libraries(test_susp_regression PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_susp_regression PRIVATE cxx_std_20)

# -- Non-Inverting Buck-Boost Converter regression tests (Part 44) -----------
set(CONV_SIM_ROOT "${CMAKE_SOURCE_DIR}/case-study/Non-Inverting Buck-Boost Converter")
add_executable(test_buck_boost_regression
    test_buck_boost_regression.cpp
    ${CONV_SIM_ROOT}/sim/src/buck_boost_plant.cpp
    ${CONV_SIM_ROOT}/sim/src/input_profile.cpp
    ${CONV_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_buck_boost_regression PRIVATE ${CONV_SIM_ROOT}/sim/include)
target_link_libraries(test_buck_boost_regression PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_buck_boost_regression PRIVATE cxx_std_20)

# -- Solar Cooker regression tests (Part 44) ----------------------------------
set(COOKER_SIM_ROOT "${CMAKE_SOURCE_DIR}/case-study/Solar Cooker with Reflector and Absorber")
add_executable(test_solar_cooker_regression
    test_solar_cooker_regression.cpp
    ${COOKER_SIM_ROOT}/sim/src/solar_cooker_plant.cpp
    ${COOKER_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_solar_cooker_regression PRIVATE ${COOKER_SIM_ROOT}/sim/include)
target_link_libraries(test_solar_cooker_regression PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_solar_cooker_regression PRIVATE cxx_std_20)

# -- SMISMO regression tests (Part 44) ----------------------------------------
set(SMISMO_SIM_ROOT "${CMAKE_SOURCE_DIR}/case-study/Separate Meter In Separate Meter Out")
add_executable(test_smismo_regression
    test_smismo_regression.cpp
    ${SMISMO_SIM_ROOT}/sim/src/smismo_plant.cpp
    ${SMISMO_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_smismo_regression PRIVATE ${SMISMO_SIM_ROOT}/sim/include)
target_link_libraries(test_smismo_regression PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_smismo_regression PRIVATE cxx_std_20)

# -- S-OTEC regression tests (Part 44) ----------------------------------------
set(SOTEC_SIM_ROOT "${CMAKE_SOURCE_DIR}/case-study/Solar Ocean Thermal Energy Conversion System")
add_executable(test_sotec_regression
    test_sotec_regression.cpp
    ${SOTEC_SIM_ROOT}/sim/src/sotec_plant.cpp
    ${SOTEC_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_sotec_regression PRIVATE ${SOTEC_SIM_ROOT}/sim/include)
target_link_libraries(test_sotec_regression PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_sotec_regression PRIVATE cxx_std_20)

# -- 6-DOF Stewart Platform regression tests (Part 61) -----------------------
set(STEWART_SIM_ROOT "${CMAKE_SOURCE_DIR}/case-study/6-DOF Stewart Platform Vessel Motion Simulator")
add_executable(test_stewart_regression
    test_stewart_regression.cpp
    ${STEWART_SIM_ROOT}/sim/src/stewart_plant.cpp
    ${STEWART_SIM_ROOT}/sim/src/cfd_input_model.cpp
    ${STEWART_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_stewart_regression PRIVATE ${STEWART_SIM_ROOT}/sim/include)
target_link_libraries(test_stewart_regression PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_stewart_regression PRIVATE cxx_std_20)

# -- Bouyancy-Driven Airship in Vertical Plane regression tests -------------
set(AIRSHIP_SIM_ROOT "${CMAKE_SOURCE_DIR}/case-study/Bouyancy-Driven Airship in Vertical Plane")
add_executable(test_bouyancy_driven_airship_regression
    test_bouyancy_driven_airship_regression.cpp
    ${AIRSHIP_SIM_ROOT}/sim/src/bouyancy_driven_airship_in_vertical_plan_plant.cpp
    ${AIRSHIP_SIM_ROOT}/sim/src/controllers.cpp
)
target_include_directories(test_bouyancy_driven_airship_regression PRIVATE ${AIRSHIP_SIM_ROOT}/sim/include)
target_link_libraries(test_bouyancy_driven_airship_regression PRIVATE
    controller_toolbox Catch2::Catch2WithMain nlohmann_json::nlohmann_json
)
target_compile_features(test_bouyancy_driven_airship_regression PRIVATE cxx_std_20)

# -- Embedded subset tests (DIST-2) ------------------------------------------
# No Eigen dependency: links only Catch2. Verifies header-only embedded subset
# compiles and behaves correctly with float and double instantiations.
add_executable(test_embedded_subset test_embedded_subset.cpp)
target_link_libraries(test_embedded_subset PRIVATE Catch2::Catch2WithMain)
target_include_directories(test_embedded_subset PRIVATE ${CMAKE_SOURCE_DIR})
target_compile_features(test_embedded_subset PRIVATE cxx_std_17)

enable_testing()
catch_discover_tests(test_embedded_subset)
catch_discover_tests(test_catch2_pilot)
catch_discover_tests(test_catch2_advanced)
catch_discover_tests(test_tugsim_regression)
catch_discover_tests(test_humidification)
catch_discover_tests(test_boiler_regression)
catch_discover_tests(test_solar_regression)
catch_discover_tests(test_humid_regression)
catch_discover_tests(test_susp_regression)
catch_discover_tests(test_buck_boost_regression)
catch_discover_tests(test_solar_cooker_regression)
catch_discover_tests(test_sotec_regression)
catch_discover_tests(test_smismo_regression)
catch_discover_tests(test_stewart_regression)
catch_discover_tests(test_bouyancy_driven_airship_regression)
add_test(NAME controller_tests  COMMAND test_controllers)
add_test(NAME tuner_tests       COMMAND test_tuners_extended)
add_test(NAME integration_tests COMMAND test_integration)
add_test(NAME tune_controllers  COMMAND tune_controllers)