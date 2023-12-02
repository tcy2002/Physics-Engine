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
        int num = 0;
        while (++num < 600) {
            if (num % 10 == 0) {
                std::cout << num / 10 << std::endl;
            }
            if (num == 100) {
                OpenglViewer::setCamera({0, 0, 3}, 0, 0);
            }
            if (num == 200) {
                std::cout << "add mesh: " << OpenglViewer::addMesh(box_mesh, true) << std::endl;
            }
            if (num == 500) {
                std::cout << "del mesh: " << 0 << std::endl;
                OpenglViewer::delMesh(0);
            }
            for (auto& p : box_mesh.vertices) {
                p.position *= 0.999;
            }
            OpenglViewer::updateMesh(0, box_mesh);
            Sleep(10);
        }
        OpenglViewer::close();
    });
    render_thread.join();
    log_thread.join();
    pe_common::Transform transform;
    transform.setEulerRotation(0, 0, 1);
    std::cout << "add mesh: " << OpenglViewer::addMesh(box_mesh) << std::endl;
    std::cout << "set transform: " << OpenglViewer::updateMeshTransform(0, transform) << std::endl;
    OpenglViewer::open("PhysicsEngine", 800, 600);
    OpenglViewer::close();
    return 0;
}
