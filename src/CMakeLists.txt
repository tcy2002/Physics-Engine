include_directories(${ENGINE_INCLUDE_DIR})

add_subdirectory(phys)   # physics module
add_subdirectory(viewer) # opengl ui module
add_subdirectory(common) # common auxiliary module

add_library(PEngine SHARED core/world.h core/world.cpp core/simulator.h core/simulator.cpp)
add_library(PEViewer SHARED core/viewer.h core/viewer.cpp)

target_link_libraries(PEngine PECommon PEPhys)
target_link_libraries(PEViewer PECommon PEOpenglViewer)

