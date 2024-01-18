include_directories(.)

add_subdirectory(common)

add_library(PEngine SHARED core/world.cpp)
add_library(PESim STATIC core/simulator.cpp)
add_library(PEViewer STATIC core/viewer.cpp)

target_include_directories(PEViewer PUBLIC ${SV_INCLUDE_DIR})
target_link_libraries(PEViewer PRIVATE ${SV_LIB_DIR}/${CMAKE_BUILD_TYPE}/libSimpleViewer.dll.a)

target_link_libraries(PEngine PECommon)
