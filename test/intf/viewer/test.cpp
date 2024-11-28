#include "intf/viewer.h"

void testViewer() {
    pe_intf::Viewer::open("ViewerTest", 800, 600, {0, 5, 10}, 0, (float)(PE_PI / 6.0));

    int id = pe_intf::Viewer::addCylinder(1.2, 1.0);
    pe_intf::Viewer::updateColor(id, pe_phys_shape::ShapeType::ST_Cylinder, pe::Vector3(0.3, 0.3, 0.7));

    pe::Transform transform = pe::Transform::identity();
    pe::Transform rotation;
    rotation.setEulerRotation(0, 0, 0.01);
    while (pe_intf::Viewer::getKeyState('q') != 0 && pe_intf::Viewer::isOpen()) {
        auto t = COMMON_GetTickCount();
        transform *= rotation;
        pe_intf::Viewer::updateTransform(id, pe_phys_shape::ShapeType::ST_Cylinder, transform);
        COMMON_USleep(10000 - I(COMMON_GetMicroTickCount() - t));
    }

    pe_intf::Viewer::close();
}

int main() {
    testViewer();
}
