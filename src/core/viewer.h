#pragma once

#include "phys/phys_general.h"
#include "common/mesh.h"
#include "opengl_viewer.h"

namespace pe_core {

    class Viewer {
    protected:
        pe::HashMap<int, simple_viewer::ObjType> _obj_map;

        static common::Vector3<float> convertVector3(pe::Vector3 vector);
        static common::Matrix3x3<float> convertMatrix3(pe::Matrix3 matrix);
        static common::Transform<float> convertTransform(pe::Transform transform);
        static common::Mesh<float> convertMesh(const pe::Mesh& mesh);

    public:
        static void open();
        static void close();

        static int getKeyState(char key);
        static int getMouseButtonState(int button);

        int addCube(const pe::Vector3& size);
        void updateCubeTransform(int id, const pe::Transform& transform);
        void updateCubeColor(int id, const pe::Vector3& color);
        void removeCube(int id);

        int addMesh(const pe::Mesh& mesh);
        void updateMeshTransform(int id, const pe::Transform& transform);
        void updateMeshColor(int id, const pe::Vector3& color);
        void removeMesh(int id);
    };

} // namespace pe_core