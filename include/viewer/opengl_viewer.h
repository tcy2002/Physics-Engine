#pragma once

#include "public_def.h"

namespace simple_viewer {

// object type
enum ObjType {
    OBJ_NONE = 0,
    OBJ_MESH = 1,
    OBJ_LINE = 2
};

// object update command type
enum ObjUpdateType {
    OBJ_UPDATE_NONE = 0,
    OBJ_UPDATE_TRANSFORM,
    OBJ_UPDATE_COLOR,
    OBJ_UPDATE_MESH,
    OBJ_UPDATE_LINE,
    OBJ_UPDATE_LINE_WIDTH,
    OBJ_DEL,
    OBJ_CLEAR_ALL_TYPE
};

// the parameters of function addObj
struct ObjInitParam {
    ObjType type = ObjType::OBJ_NONE;
    bool dynamic = false;
    union {
        SV_Mesh mesh;
        std::vector<SV_Vector3> line;
    };

    ObjInitParam(ObjType _type, bool _dynamic, SV_Mesh _mesh):
        type(_type), dynamic(_dynamic), mesh(std::move(_mesh)) {}
    ObjInitParam(ObjType _type, bool _dynamic, std::vector<SV_Vector3> _line):
        type(_type), dynamic(_dynamic), line(std::move(_line)) {}
    ~ObjInitParam() {}
};

// the parameters of function updateObj
struct ObjUpdateParam {
    ObjUpdateType act_type = ObjUpdateType::OBJ_UPDATE_NONE;
    int obj_id = -1;
    int obj_type = ObjType::OBJ_NONE;
    union {
        SV_Transform transform;
        SV_Vector3 color;
        SV_Mesh mesh;
        std::vector<SV_Vector3> line;
        float line_width;
    };

    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, const SV_Transform& _transform):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), transform(_transform) {}
    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, const SV_Vector3& _color):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), color(_color) {}
    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, SV_Mesh _mesh):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), mesh(std::move(_mesh)) {}
    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, std::vector<SV_Vector3> _line):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), line(std::move(_line)) {}
    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, float _line_width):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), line_width(_line_width) {}
    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), line_width(0) {}
    ~ObjUpdateParam() {}
};

/**
 * @brief A singleton, single-window, reopenable OpenGL viewer
 *
 * Supported objects: Mesh, Line(Strips)
 *
 * @usage
 * - Open the window: OpenglViewer::open("Window Name", Width, Height);\n
 * - Close the window: OpenglViewer::close();\n
 * - Set camera: OpenglViewer::setCamera(Position, Yaw, Pitch);\n
 * - Add objects: OpenglViewer::addObj({ObjType, IsDynamic, TargetParameter});\n
 * - Update objects: OpenglViewer::updateObj({ObjUpdateType, ObjId, ObjType[, TargetParameter]});
 *
 * @note
 * - The camera and objects are independent of the window, so open/close the window will not
 *   affect the camera and objects.\n
 * - The viewer program is thread-safe, which means you can set camera and add/update objects
 *   in any thread, and you can also open/close the window in a different thread.\n
 * - The program will NOT initialize a camera until OpenglViewer::setCamera() is manually
 *   called.\n
 * - When calling OpenglViewer::addObj()/OpenglViewer::updateObj(), the "TargetParameter"
 *   MUST be in consistency with the "ObjType"/"ObjUpdateType", or something not expected will 
 *   happen.
 */
class OpenglViewer {
    //// Window
public:
    static void open(const std::string& name, int width = 800, int height = 600);
    static void close();

    //// Camera
public:
    static void setCamera(const SV_Vector3& position, float yaw, float pitch);

    //// Object
public:
    static int addObj(const ObjInitParam& param);
    static bool updateObj(const ObjUpdateParam& param);

    //// TODO: Text Panel

private:
    OpenglViewer() = default;
    OpenglViewer(const OpenglViewer&) = delete;
    OpenglViewer& operator=(const OpenglViewer&) = delete;
    ~OpenglViewer();
};

} // namespace simple_viewer