cmake_minimum_required(VERSION 3.16)

add_executable(toolbox_examples toolbox_examples.cpp)
target_link_libraries(toolbox_examples PRIVATE controller_toolbox)
target_compile_features(toolbox_examples PRIVATE cxx_std_17)

add_subdirectory("Boiler Control")
add_subdirectory("Tug Boat Numerical Simulation")
add_subdirectory("Solar-Driven Cooling System with Photovoltaic Evaporative Chimney")
add_subdirectory("meter in meter out control")
add_subdirectory("Porous Fiber Plate Humidification System")
