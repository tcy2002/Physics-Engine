file(GLOB_RECURSE PHYS_SOURCES phys/*.cpp)

add_library(PEngine SHARED intf/world.cpp
        ${PHYS_SOURCES})
add_library(PEViewer STATIC intf/simulator.cpp intf/viewer.cpp)
add_library(PEUtils STATIC
        utils/logger.cpp
        utils/thread_pool.cpp
        utils/bvh.cpp
        utils/file_system.cpp)

if(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
target_link_libraries(PEngine PRIVATE PEUtils)
target_link_libraries(PEViewer PRIVATE PEUtils
        ${SV_LIB_DIR}/SimpleViewer_${CMAKE_CFG_INTDIR}.lib
        ${SV_LIB_DIR}/freeglut.lib ${SV_LIB_DIR}/glew32.lib opengl32)
else()
target_link_libraries(PEViewer PRIVATE
        ${SV_LIB_DIR}/libSimpleViewer.a
        ${SV_LIB_DIR}/libglut.a ${SV_LIB_DIR}/libGLEW.a 
        pthread X11 Xrandr Xi Xxf86vm GL)
endif()
