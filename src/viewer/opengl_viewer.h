/**
 * @brief A singleton, single-window, reopenable OpenGL viewer
 *
 * Supported objects: Mesh, Cube, Cylinder, Line(Strips), TODO: Char(Ascii)
 *
 * @usage
 * - Open window: OpenglViewer::open("Window Name", Width, Height);\n
 * - Close window: OpenglViewer::close();\n
 * - Set camera: OpenglViewer::setCamera(Position, Yaw, Pitch);\n
 * - Add objects: OpenglViewer::addObj({ObjType, IsDynamic, TargetParameter});\n
 * - Update objects: OpenglViewer::updateObj({ObjUpdateType, ObjId, ObjType[, TargetParameter]});\n
 * - Hide or show axis: OpenglViewer::showAxis(bool show = true);
 *
 * @axis
 * - Consistent with OpenGL, i.e. x is right, y is up, z is backward.\n
 * - A preview of 3 axes is shown in the lower left corner of the window (default shown, you can
 *   call OpenglViewer::showAxis(false) to hide).
 *
 * @camera
 * Dof: position, yaw and pitch\n
 * - left button down and drag: change yaw and move forward/backward
 *   (in horizontal plane)\n
 * - middle button down (or left and right button both down) and drag:
 *   move leftward/rightward/upward/downward (in vertical plane)\n
 * - right button down and drag: change yaw and pitch\n
 * - roll mouse wheel forward/backward: move forward/backward
 *   (relative to camera's direction)\n
 * - any mouse button down and roll mouse wheel forward/backward: change
 *   move speed up/down\n
 * - w/a/s/d/q/e (forward/leftward/backward/rightward/downward/upward):
 *   move camera when any mouse button is down
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

#pragma once

#include "public_include.h"

namespace simple_viewer {

// object type
enum ObjType {
    OBJ_NONE = 0,
    OBJ_MESH = 1,
    OBJ_CUBE = 2,
    OBJ_CYLINDER = 3,
    OBJ_LINE = 4
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
    OBJ_CLEAR_ALL_TYPE,
    OBJ_CLEAR_ALL
};

// the parameters of function addObj
struct ObjInitParam {
    ObjType type = ObjType::OBJ_NONE;
    bool dynamic = false;
    union {
        Mesh mesh;
        Vector3 size;
        std::vector<Vector3> line;
    };
    ObjInitParam(ObjType _type, bool _dynamic, Mesh _mesh):
        type(_type), dynamic(_dynamic), mesh(std::move(_mesh)) {}
    ObjInitParam(ObjType _type, bool _dynamic, const Vector3& _size):
        type(_type), dynamic(_dynamic), size(_size) {}
    ObjInitParam(ObjType _type, bool _dynamic, const std::pair<float, float> _size):
        type(_type), dynamic(_dynamic), size({_size.first, _size.second, 0}) {}
    ObjInitParam(ObjType _type, bool _dynamic, std::vector<Vector3> _line):
        type(_type), dynamic(_dynamic), line(std::move(_line)) {}
    ~ObjInitParam() {}
};

// the parameters of function updateObj
struct ObjUpdateParam {
    ObjUpdateType act_type = ObjUpdateType::OBJ_UPDATE_NONE;
    int obj_id = -1;
    int obj_type = ObjType::OBJ_NONE;
    union {
        Transform transform;
        Vector3 color;
        Mesh mesh;
        std::vector<Vector3> line;
        float line_width;
    };
    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, const Transform& _transform):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), transform(_transform) {}
    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, const Vector3& _color):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), color(_color) {}
    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, Mesh _mesh):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), mesh(std::move(_mesh)) {}
    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, std::vector<Vector3> _line):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), line(std::move(_line)) {}
    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, float _line_width):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), line_width(_line_width) {}
    ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type):
        act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), line_width(0) {}
    explicit ObjUpdateParam(ObjUpdateType _act_type):
        act_type(_act_type), obj_id(-1), obj_type(ObjType::OBJ_NONE), line_width(0) {}
    ~ObjUpdateParam() {}
};

//// Window
void open(const std::string& name, int width = 800, int height = 600);
void close();

//// Camera
void setCamera(const Vector3& position, float yaw, float pitch);

//// Object
int addObj(const ObjInitParam& param);
bool updateObj(const ObjUpdateParam& param);

//// Axis
void showAxis(bool show = true);

} // namespace simple_viewer