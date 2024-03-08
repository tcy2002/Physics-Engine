#include "viewer.h"
#include <thread>

namespace pe_core {

    std::thread* viewer_thread = nullptr;
    pe::HashMap<int, simple_viewer::ObjType> Viewer::_obj_map;

    common::Vector3<float> Viewer::convertVector3(pe::Vector3 vector) {
        return common::Vector3<float>((float)vector.x, (float)vector.y, (float)vector.z);
    }

    common::Matrix3x3<float> Viewer::convertMatrix3(pe::Matrix3 matrix) {
        return common::Matrix3x3<float>((float)matrix[0][0], (float)matrix[0][1], (float)matrix[0][2],
                                        (float)matrix[1][0], (float)matrix[1][1], (float)matrix[1][2],
                                        (float)matrix[2][0], (float)matrix[2][1], (float)matrix[2][2]);
    }

    common::Transform<float> Viewer::convertTransform(pe::Transform transform) {
        return common::Transform<float>(convertMatrix3(transform.getBasis()), convertVector3(transform.getOrigin()));
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

    void Viewer::open() {
        simple_viewer::setCamera(common::Vector3<float>(0, 10, 20), 0, 3.1415 / 6);
        viewer_thread = new std::thread([]{ simple_viewer::open("PhysicEngine", 800, 600); });
    }

    void Viewer::close() {
        simple_viewer::close();
        viewer_thread->join();
        delete viewer_thread;
    }

    int Viewer::getKeyState(char key) {
        return simple_viewer::getKeyState(key);
    }

    int Viewer::getMouseButtonState(int button) {
        return simple_viewer::getMouseState(button);
    }

    int Viewer::addCube(const pe::Vector3& size) {
        int id = simple_viewer::addObj(simple_viewer::ObjInitParam(
                simple_viewer::ObjType::OBJ_CUBE, true,
                (float)size.x, (float)size.y, (float)size.z));
        _obj_map[id] = simple_viewer::ObjType::OBJ_CUBE;
        return id;
    }

    void Viewer::updateCubeTransform(int id, const pe::Transform& transform) {
        if (_obj_map[id] == simple_viewer::ObjType::OBJ_CUBE) {
            simple_viewer::updateObj(simple_viewer::ObjUpdateParam(
                    simple_viewer::ObjUpdateType::OBJ_UPDATE_TRANSFORM,
                    id, simple_viewer::ObjType::OBJ_CUBE,
                    convertTransform(transform)));
        }
    }

    void Viewer::updateCubeColor(int id, const pe::Vector3& color) {
        if (_obj_map[id] == simple_viewer::ObjType::OBJ_CUBE) {
            simple_viewer::updateObj(simple_viewer::ObjUpdateParam(
                    simple_viewer::ObjUpdateType::OBJ_UPDATE_COLOR,
                    id, simple_viewer::ObjType::OBJ_CUBE,
                    convertVector3(color)));
        }
    }

    void Viewer::removeCube(int id) {
        if (_obj_map[id] == simple_viewer::ObjType::OBJ_CUBE) {
            simple_viewer::updateObj(simple_viewer::ObjUpdateParam(
                    simple_viewer::ObjUpdateType::OBJ_DEL,
                    id, simple_viewer::ObjType::OBJ_CUBE));
            _obj_map.erase(id);
        }
    }

    int Viewer::addMesh(const pe::Mesh& mesh) {
        int id = simple_viewer::addObj(simple_viewer::ObjInitParam(
                simple_viewer::ObjType::OBJ_MESH, true,
                convertMesh(mesh)));
        _obj_map[id] = simple_viewer::ObjType::OBJ_MESH;
        return id;
    }

    void Viewer::updateMeshTransform(int id, const pe::Transform& transform) {
        if (_obj_map[id] == simple_viewer::ObjType::OBJ_MESH) {
            simple_viewer::updateObj(simple_viewer::ObjUpdateParam(
                    simple_viewer::ObjUpdateType::OBJ_UPDATE_TRANSFORM,
                    id, simple_viewer::ObjType::OBJ_MESH,
                    convertTransform(transform)));
        }
    }

    void Viewer::updateMeshColor(int id, const pe::Vector3& color) {
        if (_obj_map[id] == simple_viewer::ObjType::OBJ_MESH) {
            simple_viewer::updateObj(simple_viewer::ObjUpdateParam(
                    simple_viewer::ObjUpdateType::OBJ_UPDATE_COLOR,
                    id, simple_viewer::ObjType::OBJ_MESH,
                    convertVector3(color)));
        }
    }

    void Viewer::removeMesh(int id) {
        if (_obj_map[id] == simple_viewer::ObjType::OBJ_MESH) {
            simple_viewer::updateObj(simple_viewer::ObjUpdateParam(
                    simple_viewer::ObjUpdateType::OBJ_DEL,
                    id, simple_viewer::ObjType::OBJ_MESH));
            _obj_map.erase(id);
        }
    }

} // namespace pe_core