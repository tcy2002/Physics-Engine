add_library(
        Ui
        viewer.h viewer.cpp
        camera.h camera.cpp
        mesh_renderer.h mesh_renderer.cpp
        shader_program.h shader_program.cpp
        shader_vert.h shader_frag.h
)

target_include_directories(Ui PRIVATE ${OPENGL_INCLUDE_DIR})
target_link_libraries(Ui PRIVATE ${OPENGL_LIB_DIR}/glew32.lib ${OPENGL_LIB_DIR}/libfreeglut.a opengl32)
