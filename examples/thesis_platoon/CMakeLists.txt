# The thesis' fuzzy controllers have no SUMO dependency, so they are always
# built (and unit-tested). The scenario runner and GA trainers that drive
# SUMO through libtraci are added by examples/CMakeLists.txt only when SUMO
# is found.
add_library(thesis_fis STATIC thesis_fis.cpp thesis_params.cpp easy_ga.cpp)
target_include_directories(thesis_fis PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(thesis_fis PUBLIC fuzzylib::fuzzylib)
