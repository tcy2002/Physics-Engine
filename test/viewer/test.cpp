#include "viewer/opengl_viewer.h"
#include "viewer/default_mesh.h"
#include <thread>

int main() {
    auto mesh = simple_viewer::_cube_mesh;
    auto render_thread = std::thread([&]{
        simple_viewer::open("UITest", 800, 600);
    });
    auto log_thread = std::thread([&]{
        int num = 0, id = -1;
        while (++num < 600) {
            if (num % 10 == 0) {
                std::cout << num / 10 << std::endl;
            }
            if (num == 100) {
                simple_viewer::setCamera({0, 0, 3}, 0, 0);
            }
            if (num == 200) {
                id = simple_viewer::addObj(simple_viewer::ObjInitParam(simple_viewer::ObjType::OBJ_MESH, true, mesh));
                std::cout << "add mesh: " << id << std::endl;
                simple_viewer::showAxis();
            }
            if (num == 500) {
                std::cout << "del mesh: " << id << std::endl;
                simple_viewer::updateObj(simple_viewer::ObjUpdateParam(simple_viewer::ObjUpdateType::OBJ_CLEAR_ALL));
            }
            for (auto& p : mesh.vertices) {
                p.position *= 0.999;
            }
            simple_viewer::updateObj(simple_viewer::ObjUpdateParam(simple_viewer::ObjUpdateType::OBJ_UPDATE_MESH, id, simple_viewer::ObjType::OBJ_MESH, mesh));
            COMMON_Sleep(5);
        }
        simple_viewer::close();
    });
    render_thread.join();
    log_thread.join();

    simple_viewer::setCamera({0, 0, 3}, 0, 0);

    simple_viewer::Transform transform;
    transform.setEulerRotation(0, 0, 1);
    int id = simple_viewer::addObj(simple_viewer::ObjInitParam(simple_viewer::ObjType::OBJ_CYLINDER, false, std::pair<float, float>(0.4, 1)));
    transform.setTranslation({1, 0, 0});
    simple_viewer::updateObj({simple_viewer::ObjUpdateType::OBJ_UPDATE_TRANSFORM, id, simple_viewer::ObjType::OBJ_CYLINDER, transform});
    id = simple_viewer::addObj(simple_viewer::ObjInitParam(simple_viewer::ObjType::OBJ_CUBE, false, simple_viewer::Vector3(0.8, 0.9, 1)));
    transform.setTranslation({-1, 0, 0});
    simple_viewer::updateObj(simple_viewer::ObjUpdateParam(simple_viewer::ObjUpdateType::OBJ_UPDATE_TRANSFORM, id, simple_viewer::ObjType::OBJ_CUBE, transform));

    simple_viewer::addObj(simple_viewer::ObjInitParam(simple_viewer::ObjType::OBJ_LINE, false, {{0, -1.414, 0}, {1, 0, 1}, {0, 1.414, 0}}));
    simple_viewer::addObj(simple_viewer::ObjInitParam(simple_viewer::ObjType::OBJ_LINE, false, {{0, -1.414, 0}, {-1, 0, -1}, {0, 1.414, 0}}));

    simple_viewer::open("UITest", 800, 600);

    return 0;
}
