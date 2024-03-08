#include "core/viewer.h"

void testViewer() {
    pe_core::Viewer::open();

    int id = pe_core::Viewer::addCube(pe::Vector3(1, 1, 1));
    pe_core::Viewer::updateCubeColor(id, pe::Vector3(0.3, 0.3, 0.7));

    pe::Transform transform = pe::Transform::identity();
    pe::Transform rotation;
    rotation.setEulerRotation(0, 0, 0.01);
    while (pe_core::Viewer::getKeyState('q') != 0) {
        auto t = COMMON_GetTickCount();
        transform *= rotation;
        pe_core::Viewer::updateCubeTransform(id, transform);
        COMMON_Sleep(10 - (int)(COMMON_GetTickCount() - t));
    }

    pe_core::Viewer::close();
}

int main() {
    testViewer();
}
