#pragma once

#include "phys/phys_general.h"
#include "common/mesh.h"
#include "phys/shape/shape.h"
#include "opengl_viewer.h"

namespace pe_intf { // interface

    class Viewer {
    protected:
        static pe::HashMap<int, simple_viewer::ObjType> _obj_map;

        static COMMON_FORCE_INLINE common::Vector3<float> convertVector3(const pe::Vector3& vector);
        static COMMON_FORCE_INLINE common::Matrix3<float> convertMatrix3(const pe::Matrix3& matrix);
        static COMMON_FORCE_INLINE common::Transform<float> convertTransform(const pe::Transform& transform);
        static COMMON_FORCE_INLINE common::Mesh<float> convertMesh(const pe::Mesh& mesh);

    public:
        static void open(const std::string& title, int width, int height,
                            const common::Vector3<float>& camera_pos,
                            float camera_yaw, float camera_pitch);
        static void close();
        static bool isOpen();

        static void showAxis(bool show);
        static void showLine(bool show, int width);

        static int getKeyState(char key);
        static int getMouseButtonState(int button);

        static int addCube(const pe::Vector3& size);
        static int addMesh(const pe::Mesh& mesh);
        static int addSphere(pe::Real radius);
        static int addCylinder(pe::Real radius, pe::Real height);

        static void updateTransform(int id, pe_phys_shape::ShapeType type, const pe::Transform& transform);
        static void updateColor(int id, pe_phys_shape::ShapeType type, const pe::Vector3& color);
        static void remove(int id);
    };

} // namespace pe_intf