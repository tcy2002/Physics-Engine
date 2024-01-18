#include "viewer.h"
#include <opengl_viewer.h>

namespace pe_core {

    void Viewer::open() {
        simple_viewer::setCamera(simple_viewer::Vector3(0, 0, 10), 0, 0);
        simple_viewer::open("ViewerTest", 800, 600);
    }

    void Viewer::close() {
        simple_viewer::close();
    }

} // namespace pe_core