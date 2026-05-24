set(SIM_INC "${CMAKE_CURRENT_SOURCE_DIR}/sim/include")
set(SIM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/sim/src")

set(SIM_SRCS
    ${SIM_SRC}/boiler_plant.cpp
    ${SIM_SRC}/linearizer.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/simulation_runner.cpp
    ${SIM_SRC}/telemetry_logger.cpp
    ${SIM_SRC}/main.cpp
)

add_executable(boiler_sim ${SIM_SRCS})

target_include_directories(boiler_sim PRIVATE
    ${SIM_INC}
)
target_link_libraries(boiler_sim PRIVATE controller_toolbox)
target_compile_features(boiler_sim PRIVATE cxx_std_17)
target_compile_definitions(boiler_sim PRIVATE
    BOILER_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
