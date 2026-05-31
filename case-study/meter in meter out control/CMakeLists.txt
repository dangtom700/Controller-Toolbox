set(SIM_INC "${CMAKE_CURRENT_SOURCE_DIR}/sim/include")
set(SIM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/sim/src")

add_executable(smismo_sim
    ${SIM_SRC}/smismo_plant.cpp
    ${SIM_SRC}/smismo_controllers.cpp
    ${SIM_SRC}/smismo_main.cpp
)

target_include_directories(smismo_sim PRIVATE ${SIM_INC})
target_link_libraries(smismo_sim PRIVATE controller_toolbox)
target_compile_features(smismo_sim PRIVATE cxx_std_20)
target_compile_definitions(smismo_sim PRIVATE
    SMISMO_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
