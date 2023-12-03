#include "viewer/opengl_viewer.h"
#include "phys/shape/box_shape.h"
#include <thread>

using namespace simple_viewer;

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
                id = OpenglViewer::addObj({ObjType::OBJ_MESH, true, box_mesh});
                std::cout << "add mesh: " << id << std::endl;
            }
            if (num == 500) {
                std::cout << "del mesh: " << id << std::endl;
                OpenglViewer::updateObj({ObjUpdateType::OBJ_CLEAR_ALL_TYPE, id, ObjType::OBJ_MESH});
            }
            for (auto& p : box_mesh.vertices) {
                p.position *= 0.999;
            }
            OpenglViewer::updateObj({ObjUpdateType::OBJ_UPDATE_MESH, id, ObjType::OBJ_MESH, box_mesh});
            PE_Sleep(5);
        }
        OpenglViewer::close();
    });
    render_thread.join();
    log_thread.join();

    pe_common::Transform transform;
    transform.setEulerRotation(0, 0, 1);
    int mesh_id = OpenglViewer::addObj({ObjType::OBJ_MESH, false, box_mesh});
    transform.setTranslation({1, 0, 0});
    OpenglViewer::updateObj({ObjUpdateType::OBJ_UPDATE_TRANSFORM, mesh_id, ObjType::OBJ_MESH, transform});
    mesh_id = OpenglViewer::addObj({ObjType::OBJ_MESH, false, box_mesh});
    transform.setTranslation({-1, 0, 0});
    OpenglViewer::updateObj({ObjUpdateType::OBJ_UPDATE_TRANSFORM, mesh_id, ObjType::OBJ_MESH, transform});

    OpenglViewer::addObj({ObjType::OBJ_LINE, false,
                          {{0, -1.414, 0}, {1, 0, 1}, {0, 1.414, 0}}});
    OpenglViewer::addObj({ObjType::OBJ_LINE, false,
                          {{0,  -1.414, 0}, {-1, 0, -1}, {0, 1.414, 0}}});

    OpenglViewer::open("PhysicsEngine", 800, 600);
    OpenglViewer::close();
    return 0;
}
