file(GLOB_RECURSE PE_VIEWER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(PEOpenglViewer STATIC ${PE_VIEWER_SOURCES})

if(WIN32)
    set(OPENGL_LIB opengl32)
else()
    set(OPENGL_LIB GL)
endif()

target_include_directories(PEOpenglViewer PRIVATE ${OPENGL_INCLUDE_DIR})
target_link_directories(PEOpenglViewer INTERFACE ${OPENGL_LIB_DIR})
target_link_libraries(PEOpenglViewer PRIVATE glew32 freeglut ${OPENGL_LIB})
