#pragma once

#include "phys/phys_general.h"
#include "common/mesh.h"
#include "opengl_viewer.h"

namespace pe_intf {

    class Viewer {
    protected:
        static pe::HashMap<int, simple_viewer::ObjType> _obj_map;

        static common::Vector3<float> convertVector3(pe::Vector3 vector);
        static common::Matrix3x3<float> convertMatrix3(pe::Matrix3 matrix);
        static common::Transform<float> convertTransform(pe::Transform transform);
        static common::Mesh<float> convertMesh(const pe::Mesh& mesh);

    public:
        static void open();
        static void close();

        static int getKeyState(char key);
        static int getMouseButtonState(int button);

        static int addCube(const pe::Vector3& size);
        static void updateCubeTransform(int id, const pe::Transform& transform);
        static void updateCubeColor(int id, const pe::Vector3& color);
        static void removeCube(int id);

        static int addMesh(const pe::Mesh& mesh);
        static void updateMeshTransform(int id, const pe::Transform& transform);
        static void updateMeshColor(int id, const pe::Vector3& color);
        static void removeMesh(int id);

        static int addSphere(pe::Real radius);
        static void updateSphereTransform(int id, const pe::Transform& transform);
        static void updateSphereColor(int id, const pe::Vector3& color);
        static void removeSphere(int id);

        static int addCylinder(pe::Real radius, pe::Real height);
        static void updateCylinderTransform(int id, const pe::Transform& transform);
        static void updateCylinderColor(int id, const pe::Vector3& color);
        static void removeCylinder(int id);
    };

} // namespace pe_core