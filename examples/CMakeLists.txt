function(fuzzylib_add_example name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE fuzzylib::fuzzylib)
endfunction()

fuzzylib_add_example(tipper               tipper/tipper.cpp)
fuzzylib_add_example(water_dam            water_dam/water_dam.cpp)
fuzzylib_add_example(inverted_pendulum    inverted_pendulum/inverted_pendulum.cpp)
fuzzylib_add_example(car_platooning       car_platooning/car_platooning.cpp)
fuzzylib_add_example(quadcopter           quadcopter/quadcopter.cpp)
fuzzylib_add_example(drone_swarm          drone_swarm/drone_swarm.cpp)

fuzzylib_add_example(ardupilot_bridge     ardupilot_bridge/ardupilot_bridge.cpp)
target_include_directories(ardupilot_bridge PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
