add_subdirectory(utils)

add_library(PEngine SHARED core/world.cpp)
add_library(PESim STATIC core/simulator.cpp)
add_library(PEViewer STATIC core/viewer.cpp)

target_link_libraries(PEViewer PRIVATE
        ${SV_LIB_DIR}/${CMAKE_BUILD_TYPE}/libSimpleViewer.a
        ${SV_LIB_DIR}/freeglut.lib ${SV_LIB_DIR}/glew32.lib opengl32)

target_link_libraries(PEngine PEUtils)
