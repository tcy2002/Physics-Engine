#include "viewer/opengl_viewer.h"
#include "phys/shape/box_shape.h"
#include <thread>

using namespace pe_viewer;

int main() {
    auto box_mesh = pe_phys_shape::BoxShape({0.5, 0.5, 0.5}).getMesh();

    auto render_thread = std::thread([&]{
        OpenglViewer::open("UITest", 800, 600);
    });

    auto log_thread = std::thread([&]{
        int num = 0, id = -1;
        while (++num < 600) {
            if (num % 10 == 0) {
                std::cout << num / 10 << std::endl;
            }
            if (num == 100) {
                OpenglViewer::setCamera({0, 0, 3}, 0, 0);
            }
            if (num == 200) {
                std::cout << "add mesh: " << id << std::endl;
                id = OpenglViewer::addMesh(box_mesh, true);
            }
            if (num == 500) {
                std::cout << "del mesh: " << id << std::endl;
                OpenglViewer::delMesh(id);
            }
            for (auto& p : box_mesh.vertices) {
                p.position *= 0.999;
            }
            OpenglViewer::updateMesh(id, box_mesh);
            PE_Sleep(5);
        }
        OpenglViewer::close();
    });
    render_thread.join();
    log_thread.join();

    pe_common::Transform transform;
    transform.setEulerRotation(0, 0, 1);
    int mesh_id = OpenglViewer::addMesh(box_mesh);
    transform.setTranslation({1, 0, 0});
    OpenglViewer::updateMeshTransform(mesh_id, transform);
    mesh_id = OpenglViewer::addMesh(box_mesh);
    transform.setTranslation({-1, 0, 0});
    OpenglViewer::updateMeshTransform(mesh_id, transform);

    int line_id = OpenglViewer::addLine({{0, -1.414, 0}, {1, 0, 1}, {0, 1.414, 0}});
    OpenglViewer::updateLineWidth(line_id, 1);
    line_id = OpenglViewer::addLine({{0,  -1.414, 0}, {-1, 0, -1}, {0, 1.414, 0}});
    OpenglViewer::updateLineWidth(line_id, 1);

    OpenglViewer::open("PhysicsEngine", 800, 600);
    OpenglViewer::close();
    return 0;
}
