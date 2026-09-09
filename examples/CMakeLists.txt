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

fuzzylib_add_example(ardupilot_swarm_bridge ardupilot_bridge/ardupilot_swarm_bridge.cpp)
target_include_directories(ardupilot_swarm_bridge PRIVATE ${CMAKE_SOURCE_DIR}/third_party)

# sumo_acc links against SUMO's own libtraci (its native C++ TraCI client
# library), a real external dependency this repo can't vendor -- unlike
# every other example, so it's built only when SUMO is actually found.
# Point CMake at a SUMO install (the "sumo" Python package's install dir,
# or a from-source build) via the SUMO_HOME environment variable, the
# same variable SUMO's own tooling uses.
find_path(FUZZYLIB_SUMO_INCLUDE_DIR NAMES libsumo/libsumo.h
    HINTS "$ENV{SUMO_HOME}/include" "$ENV{SUMO_HOME}/../include")
find_library(FUZZYLIB_SUMO_TRACI_LIBRARY NAMES tracicpp
    HINTS "$ENV{SUMO_HOME}/lib64" "$ENV{SUMO_HOME}/lib" "$ENV{SUMO_HOME}/bin")

if(FUZZYLIB_SUMO_INCLUDE_DIR AND FUZZYLIB_SUMO_TRACI_LIBRARY)
    message(STATUS "SUMO libtraci found -- building sumo_acc example")
    fuzzylib_add_example(sumo_acc sumo_acc/sumo_acc.cpp)
    target_include_directories(sumo_acc PRIVATE ${FUZZYLIB_SUMO_INCLUDE_DIR})
    target_link_libraries(sumo_acc PRIVATE ${FUZZYLIB_SUMO_TRACI_LIBRARY})
    target_compile_definitions(sumo_acc PRIVATE LIBTRACI=1)
    get_filename_component(FUZZYLIB_SUMO_LIB_DIR ${FUZZYLIB_SUMO_TRACI_LIBRARY} DIRECTORY)
    set_target_properties(sumo_acc PROPERTIES BUILD_RPATH "${FUZZYLIB_SUMO_LIB_DIR}")
else()
    message(STATUS "SUMO libtraci not found -- skipping sumo_acc example (set SUMO_HOME to enable)")
endif()
