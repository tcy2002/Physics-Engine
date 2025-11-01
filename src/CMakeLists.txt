file(GLOB_RECURSE PHYS_SOURCES physics/*.cpp)

add_library(PEngine SHARED interface/world.cpp
        ${PHYS_SOURCES})
add_library(PEViewer STATIC interface/viewer.cpp interface/simulator.cpp)
add_library(PEUtils STATIC
        utils/logger.cpp
        utils/thread_pool.cpp
        utils/bvh.inl)

if(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
target_link_libraries(PEngine PRIVATE PEUtils PEViewer)
target_link_libraries(PEViewer PRIVATE
        ${SV_LIB_DIR}/SimpleViewer_${CMAKE_CFG_INTDIR}.lib
        ${SV_LIB_DIR}/freeglut.lib ${SV_LIB_DIR}/glew32.lib opengl32)
else()
target_link_libraries(PEViewer PRIVATE
        ${SV_LIB_DIR}/libSimpleViewer.a
        ${SV_LIB_DIR}/libglut.a ${SV_LIB_DIR}/libGLEW.a 
        pthread X11 Xrandr Xi Xxf86vm GL)
endif()
