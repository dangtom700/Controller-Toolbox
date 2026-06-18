add_executable(bench_controllers bench_controllers.cpp)

target_link_libraries(bench_controllers PRIVATE controller_toolbox)

target_include_directories(bench_controllers PRIVATE
    ${CMAKE_SOURCE_DIR}/lib
)

# Place the binary in build/benchmark/ for easy discovery
set_target_properties(bench_controllers PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/benchmark"
)
