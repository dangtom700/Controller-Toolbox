cmake_minimum_required(VERSION 3.16)
project(ControllerToolbox CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Absolute path to the root data/ directory, injected as a compile definition so
# all CSV-generating binaries write to the same place regardless of CWD at runtime.
add_compile_definitions(PROJECT_DATA_DIR="${CMAKE_SOURCE_DIR}/data")

add_subdirectory(lib)
add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(scripts)
add_subdirectory(case-study)
# benchmarks are intentionally excluded from the default build

find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()
