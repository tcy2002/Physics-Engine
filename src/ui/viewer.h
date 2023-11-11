#pragma once

#include <functional>
#include "camera.h"
#include "mesh_renderer.h"
#include "shader_program.h"

namespace pe_ui {

/**
 * @brief The singleton Viewer class
 *
 */
class Viewer {
private:
    Viewer(int width, int height);
    ~Viewer();

    Camera* _camera;
    ShaderProgram* _program;
    pe::Array<MeshRenderer*> _meshes;

public:
    static Viewer& initViewerInstance(int width = 800, int height = 600);

    void setCamera(const pe_cg::Vector3& position, real yaw, real pitch);
    void addMesh(const pe_cg::Mesh& mesh,
                 const pe_cg::Transform& transform = pe_cg::Transform::identity(),
                 const pe_cg::Vector3& color = pe_cg::Vector3(0.25, 0.3, 0.8));
    void run();

    void display();
    void reshape(int, int);
    void keyboard(unsigned char, int, int);
    void keyboardUp(unsigned char, int, int);
    void mouse(int, int, int, int);
    void motion(int, int);
};

} // namespace ui