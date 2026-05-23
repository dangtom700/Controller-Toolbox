cmake_minimum_required(VERSION 3.16)

add_executable(test_controllers test_controllers.cpp)
target_link_libraries(test_controllers PRIVATE controller_toolbox)
target_include_directories(test_controllers PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_tuners_extended test_tuners_extended.cpp)
target_link_libraries(test_tuners_extended PRIVATE controller_toolbox)
target_include_directories(test_tuners_extended PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_integration test_integration.cpp)
target_link_libraries(test_integration PRIVATE controller_toolbox)
target_include_directories(test_integration PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

enable_testing()
add_test(NAME controller_tests  COMMAND test_controllers)
add_test(NAME tuner_tests       COMMAND test_tuners_extended)
add_test(NAME integration_tests COMMAND test_integration)
