file(GLOB_RECURSE VIEWER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(SimpleViewer STATIC ${VIEWER_SOURCES})

if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
    set(OPENGL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/opengl/lib/x64)
else()
    set(OPENGL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/opengl/lib)
endif()
set(OPENGL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/opengl/include)

if(WIN32)
    set(OPENGL_LIB opengl32)
else()
    set(OPENGL_LIB GL)
endif()

target_include_directories(SimpleViewer PRIVATE ${OPENGL_INCLUDE_DIR})
target_link_libraries(SimpleViewer PRIVATE ${OPENGL_LIB_DIR}/glew32.lib ${OPENGL_LIB_DIR}/libfreeglut.a ${OPENGL_LIB})
