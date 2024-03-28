#include "intf/viewer.h"

void testViewer() {
    pe_intf::Viewer::open("ViewerTest", 800, 600, {0, 5, 10}, 0, (float)(PE_PI / 6.0));

    int id = pe_intf::Viewer::addCylinder(1.2, 1.0);
    pe_intf::Viewer::updateCylinderColor(id, pe::Vector3(0.3, 0.3, 0.7));

    pe::Transform transform = pe::Transform::identity();
    pe::Transform rotation;
    rotation.setEulerRotation(0, 0, 0.01);
    while (pe_intf::Viewer::getKeyState('q') != 0) {
        auto t = COMMON_GetTickCount();
        transform *= rotation;
        pe_intf::Viewer::updateCylinderTransform(id, transform);
        COMMON_Sleep(10 - (int)(COMMON_GetTickCount() - t));
    }

    pe_intf::Viewer::close();
}

int main() {
    testViewer();
}
