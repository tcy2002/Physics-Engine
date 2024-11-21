#include "viewer.h"
#include <thread>

namespace pe_intf {

    std::thread* viewer_thread = nullptr;
    pe::HashMap<int, simple_viewer::ObjType> Viewer::_obj_map; // NOLINT

    common::Vector3<float> Viewer::convertVector3(pe::Vector3 vector) {
        return {F(vector.x), F(vector.y), F(vector.z)};
    }

    common::Matrix3x3<float> Viewer::convertMatrix3(pe::Matrix3 matrix) {
        return {
            F(matrix[0][0]), F(matrix[0][1]), F(matrix[0][2]),
            F(matrix[1][0]), F(matrix[1][1]), F(matrix[1][2]),
            F(matrix[2][0]), F(matrix[2][1]), F(matrix[2][2])
        };
    }

    common::Transform<float> Viewer::convertTransform(pe::Transform transform) {
        return {convertMatrix3(transform.getBasis()), convertVector3(transform.getOrigin())};
    }

    common::Mesh<float> Viewer::convertMesh(const pe::Mesh& mesh) {
        common::Mesh<float> result;
        result.vertices.resize(mesh.vertices.size());
        result.faces.resize(mesh.faces.size());
        for (int i = 0; i < mesh.vertices.size(); i++) {
            result.vertices[i].position = convertVector3(mesh.vertices[i].position);
            result.vertices[i].normal = convertVector3(mesh.vertices[i].normal);
        }
        for (int i = 0; i < result.faces.size(); i++) {
            result.faces[i].indices = mesh.faces[i].indices;
            result.faces[i].normal = convertVector3(mesh.faces[i].normal);
        }
        return result;
    }

    void Viewer::open(const std::string& title, int width, int height,
                      const common::Vector3<float>& camera_pos,
                      float camera_yaw, float camera_pitch) {
        simple_viewer::setCamera(camera_pos, camera_yaw, camera_pitch);
        viewer_thread = new std::thread([=]{
            simple_viewer::open(title, width, height);
        });
    }

    void Viewer::close() {
        simple_viewer::close();
        viewer_thread->join();
        delete viewer_thread;
    }

    bool Viewer::isOpen() {
        return simple_viewer::isOpen();
    }

    void Viewer::showAxis(bool show) {
        simple_viewer::showAxis(show);
    }

    void Viewer::showLine(bool show, int width) {
        simple_viewer::showLine(show, width);
    }

    int Viewer::getKeyState(char key) {
        if (!isOpen()) return -1;
        return simple_viewer::getKeyState(key);
    }

    int Viewer::getMouseButtonState(int button) {
        if (!isOpen()) return -1;
        return simple_viewer::getMouseState(button);
    }

    static simple_viewer::ObjType mapShapeType(pe_phys_shape::ShapeType type) {
        switch (type) {
            case pe_phys_shape::ShapeType::Box: return simple_viewer::ObjType::OBJ_CUBE;
            case pe_phys_shape::ShapeType::Sphere: return simple_viewer::ObjType::OBJ_SPHERE;
            case pe_phys_shape::ShapeType::Cylinder: return simple_viewer::ObjType::OBJ_CYLINDER;
            case pe_phys_shape::ShapeType::ConvexMesh: case pe_phys_shape::ShapeType::ConcaveMesh:  
                return simple_viewer::ObjType::OBJ_MESH;
            default: return simple_viewer::ObjType::OBJ_NONE;
        }
    }

    int Viewer::addCube(const pe::Vector3& size) {
        int id = addObj(simple_viewer::ObjInitParam(
                simple_viewer::ObjType::OBJ_CUBE, true,
                F(size.x), F(size.y), F(size.z)));
        _obj_map[id] = simple_viewer::ObjType::OBJ_CUBE;
        return id;
    }

    int Viewer::addMesh(const pe::Mesh& mesh) {
        int id = addObj(simple_viewer::ObjInitParam(
                simple_viewer::ObjType::OBJ_MESH, true,
                convertMesh(mesh)));
        _obj_map[id] = simple_viewer::ObjType::OBJ_MESH;
        return id;
    }

    int Viewer::addSphere(pe::Real radius) {
        int id = addObj(simple_viewer::ObjInitParam(
                simple_viewer::ObjType::OBJ_SPHERE, true,
                F(radius)));
        _obj_map[id] = simple_viewer::ObjType::OBJ_SPHERE;
        return id;
    }

    int Viewer::addCylinder(pe::Real radius, pe::Real height) {
        int id = addObj(simple_viewer::ObjInitParam(
                simple_viewer::ObjType::OBJ_CYLINDER, true,
                F(radius), F(height)));
        _obj_map[id] = simple_viewer::ObjType::OBJ_CYLINDER;
        return id;
    }

    void Viewer::updateTransform(int id, pe_phys_shape::ShapeType type, const pe::Transform &transform) {
        if (_obj_map[id] == mapShapeType(type)) {
            updateObj(simple_viewer::ObjUpdateParam(
                    simple_viewer::ObjUpdateType::OBJ_UPDATE_TRANSFORM,
                    id, mapShapeType(type),
                    convertTransform(transform)));
        }
    }

    void Viewer::updateColor(int id, pe_phys_shape::ShapeType type, const pe::Vector3 &color) {
        if (_obj_map[id] == mapShapeType(type)) {
            updateObj(simple_viewer::ObjUpdateParam(
                    simple_viewer::ObjUpdateType::OBJ_UPDATE_COLOR,
                    id, mapShapeType(type),
                    convertVector3(color)));
        }
    }

    void Viewer::remove(int id) {
        if (_obj_map.find(id) == _obj_map.end()) return;
        updateObj(simple_viewer::ObjUpdateParam(
                simple_viewer::ObjUpdateType::OBJ_DEL,
                id, _obj_map[id]));
        _obj_map.erase(id);
    }

} // namespace pe_intf