#pragma once

#include "phys/phys_general.h"
#include "common/mesh.h"
#include <opengl_viewer.h>

namespace pe_core {

    class Viewer {
    protected:
        pe::HashMap<int, simple_viewer::ObjType> _obj_map;

        static simple_viewer::Vector3 convertVector3(pe::Vector3 vector);
        static simple_viewer::Matrix3 convertMatrix3(pe::Matrix3 matrix);
        static simple_viewer::Transform convertTransform(pe::Transform transform);

    public:
        static void open();
        static void close();

        static int getKeyState(char key);
        static int getMouseButtonState(int button);

        int addCube(pe::Vector3 size);
        void updateCubeTransform(int id, pe::Transform transform);
        void updateCubeColor(int id, pe::Vector3 color);
        void removeCube(int id);
    };

} // namespace pe_core