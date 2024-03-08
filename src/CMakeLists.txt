file(GLOB_RECURSE PHYS_SOURCES phys/*.cpp)

add_library(PEngine SHARED core/world.cpp ${PHYS_SOURCES})
add_library(PEViewer STATIC core/viewer.cpp)
add_library(PEUtils STATIC utils/logger.cpp utils/thread_pool.cpp)

target_link_libraries(PEViewer PRIVATE
        ${SV_LIB_DIR}/${CMAKE_BUILD_TYPE}/libSimpleViewer.a
        ${SV_LIB_DIR}/freeglut.lib ${SV_LIB_DIR}/glew32.lib opengl32)

target_link_libraries(PEngine PEViewer)

target_link_libraries(PEngine PEUtils)
