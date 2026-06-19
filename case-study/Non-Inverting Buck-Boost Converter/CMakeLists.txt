set(SIM_INC "${CMAKE_CURRENT_SOURCE_DIR}/sim/include")
set(SIM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/sim/src")

set(SIM_SRCS
    ${SIM_SRC}/buck_boost_plant.cpp
    ${SIM_SRC}/input_profile.cpp
    ${SIM_SRC}/controllers.cpp
    ${SIM_SRC}/simulation_runner.cpp
    ${SIM_SRC}/main.cpp
)

add_executable(buck_boost_sim ${SIM_SRCS})

target_include_directories(buck_boost_sim PRIVATE
    ${SIM_INC}
)
target_link_libraries(buck_boost_sim PRIVATE
    controller_toolbox
    nlohmann_json::nlohmann_json
)
target_compile_features(buck_boost_sim PRIVATE cxx_std_20)
target_compile_definitions(buck_boost_sim PRIVATE
    BUCK_BOOST_SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
