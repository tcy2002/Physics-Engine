#include "ui/viewer.h"
#include "phys/shape/box_shape.h"

using namespace pe_ui;

int main() {
    pe_phys_shape::BoxShape box({0.5, 0.5, 0.5});
    auto& viewer = Viewer::initViewerInstance("PhysicsEngine", 800, 600);
    viewer.setCamera({0, 0, 3}, 0, 0);
    viewer.addMesh(box.getMesh());
    viewer.run();

    return 0;
}