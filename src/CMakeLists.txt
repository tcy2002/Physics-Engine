include_directories(.)

add_subdirectory(common)

add_library(PEngine SHARED core/world.h core/world.cpp)
add_library(PESim STATIC core/simulator.h core/simulator.cpp)

target_link_libraries(PEngine PECommon)

