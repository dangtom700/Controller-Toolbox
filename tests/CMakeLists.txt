cmake_minimum_required(VERSION 3.16)

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

enable_testing()
catch_discover_tests(test_catch2_pilot)
catch_discover_tests(test_catch2_advanced)
catch_discover_tests(test_tugsim_regression)
add_test(NAME controller_tests  COMMAND test_controllers)
add_test(NAME tuner_tests       COMMAND test_tuners_extended)
add_test(NAME integration_tests COMMAND test_integration)
add_test(NAME tune_controllers  COMMAND tune_controllers)