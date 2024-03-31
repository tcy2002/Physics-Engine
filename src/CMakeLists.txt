file(GLOB_RECURSE PHYS_SOURCES phys/*.cpp)

add_library(PEngine STATIC intf/world.cpp
        ${PHYS_SOURCES})
add_library(PEViewer STATIC intf/viewer.cpp)
add_library(PEUtils STATIC
        utils/logger.cpp
        utils/thread_pool.cpp
        utils/jacobian_entry.cpp)

target_link_libraries(PEngine PEUtils)
target_link_libraries(PEViewer PRIVATE
        ${SV_LIB_DIR}/${CMAKE_CFG_INTDIR}/SimpleViewer.lib
        ${SV_LIB_DIR}/freeglut.lib ${SV_LIB_DIR}/glew32.lib opengl32)