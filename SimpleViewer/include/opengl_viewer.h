/**
 * @brief A singleton, single-window, reopenable OpenGL viewer
 *
 * Supported objects: common::Mesh<float>, Cube, Cylinder, Cone, Line(Strips), TODO: Text(Ascii)
 *
 * @axis
 * - Consistent with OpenGL, i.e. x is right, y is up, z is backward.
 * - A preview of 3 axes is shown at the world origin, and you can hide it by
 *   calling simple_viewer::showAxis(false).
 *
 * @camera
 * UE-Styled camera interaction
 * Dof: position, yaw and pitch
 * - (recommend) press any mouse button and press w/a/s/d/q/e: move forward/leftward/
 *   backward/rightward/downward/upward relative to camera's orientation
 * - (recommend) press any mouse button and roll mouse wheel forward/backward: turn move 
 *   speed up / down
 * - press left button and drag: change yaw and move forward/backward (in horizontal plane)
 * - press middle button (or both left and right button) and drag: move leftward/rightward/
 *   upward/downward (in vertical plane)
 * - press right button and drag: change yaw and pitch
 * - roll mouse wheel forward/backward: move forward/backward (in camera's forward direction)
 *
 * @note
 * - The camera and objects are independent of the window, so open/close the window will not
 *   affect the camera and objects.
 * - The viewer program is thread-safe, which means you can set camera and add/update objects
 *   in any thread, and you can also open/close the window in a different thread.
 * - The program will NOT initialize a camera until OpenglViewer::setCamera() is manually
 *   called.
 * - When calling OpenglViewer::addObj() or OpenglViewer::updateObj(), the parameter MUST be 
 *   in consistency with the "ObjType" or "ObjUpdateType", or something not expected will 
 *   happen. Please check the struct ObjInitParam and ObjUpdateParam for more information.
 */

#pragma once

#include <vector>
#include "common/mesh.h"
#include "common/vector3.h"
#include "common/matrix3x3.h"
#include "common/transform.h"

#ifdef _MSC_VER
#define SV_API _declspec(dllexport)
#else
#define SV_API
#endif

namespace simple_viewer {

    // object type
    enum ObjType {
        OBJ_NONE = 0,
        OBJ_MESH = 1,
        OBJ_CUBE = 2,
        OBJ_CYLINDER = 3,
        OBJ_CONE = 4,
        OBJ_LINE = 5
    };

    // object update command type
    enum ObjUpdateType {
        OBJ_UPDATE_NONE = 0,
        OBJ_UPDATE_TRANSFORM,
        OBJ_UPDATE_COLOR,
        OBJ_UPDATE_MESH,
        OBJ_UPDATE_CUBE,
        OBJ_UPDATE_CYLINDER,
        OBJ_UPDATE_CONE,
        OBJ_UPDATE_LINE,
        OBJ_UPDATE_LINE_WIDTH,
        OBJ_DEL,
        OBJ_CLEAR_ALL_TYPE,
        OBJ_CLEAR_ALL
    };

    // object initialize parameter
    struct ObjInitParam {
        ObjType type = ObjType::OBJ_NONE;
        bool dynamic = false;
        union {
            common::Mesh<float> mesh;
            common::Vector3<float> size;
            std::vector<float> line;
        };
        ObjInitParam(ObjType _type, bool _dynamic, common::Mesh<float> _mesh):  // NOLINT
                type(_type), dynamic(_dynamic), mesh(std::move(_mesh)) {}
        ObjInitParam(ObjType _type, bool _dynamic, float x, float y, float z):  // NOLINT
                type(_type), dynamic(_dynamic), size({ x, y, z }) {}
        ObjInitParam(ObjType _type, bool _dynamic, float radius, float height): // NOLINT
                type(_type), dynamic(_dynamic), size({ radius, height, 0 }) {}
        ObjInitParam(ObjType _type, bool _dynamic, std::vector<float> _line): // NOLINT
                type(_type), dynamic(_dynamic), line(std::move(_line)) {}
        ~ObjInitParam() { /* no effect but non-trivial */ }                     // NOLINT
    };

    // object update parameter
    struct ObjUpdateParam {
        ObjUpdateType act_type = ObjUpdateType::OBJ_UPDATE_NONE;
        int obj_id = -1;
        int obj_type = ObjType::OBJ_NONE;
        union {
            common::Transform<float> transform;
            common::Vector3<float> vec;
            common::Mesh<float> mesh;
            std::vector<float> line;
            float line_width;
        };
        ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, const common::Transform<float>& _transform): // NOLINT
            act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), transform(_transform) {}
        ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, const common::Vector3<float>& color):        // NOLINT
            act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), vec(color) {}
        ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, float x, float y, float z):   // NOLINT
            act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), vec({ x, y, z }) {}
        ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, float radius, float height):  // NOLINT
            act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), vec({ radius, height, 0 }) {}
        ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, common::Mesh<float> _mesh):   // NOLINT
            act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), mesh(std::move(_mesh)) {}
        ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, std::vector<float> _line):    // NOLINT
            act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), line(std::move(_line)) {}
        ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type, float _line_width):           // NOLINT
            act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), line_width(_line_width) {}
        ObjUpdateParam(ObjUpdateType _act_type, int _obj_id, int _obj_type):                              // NOLINT
            act_type(_act_type), obj_id(_obj_id), obj_type(_obj_type), line_width(0) {}
        ObjUpdateParam(ObjUpdateType _act_type):                                                          // NOLINT
            act_type(_act_type), obj_id(-1), obj_type(ObjType::OBJ_NONE), line_width(0) {}
        ~ObjUpdateParam() { /* no effect but non-trivial */ }                                             // NOLINT
    };

    //// Window
    /**
     * @brief Open a opengl window
     * @param name window name
     * @param width window width (800 by default)
     * @param height window height (600 by default)
     */
    SV_API void open(const std::string& name, int width = 800, int height = 600);
    /**
     * @brief Close the window
     */
    SV_API void close();

    //// Frame rate
    /**
     * @brief Set the target frame rate (actural frame rate depends on the performance)
     * @param fps frame per second
     */
    SV_API void setTargetFrameRate(int fps);

    //// Camera
    /**
     * @brief Set (initialize) a camera
     * @param position world position
     * @param yaw in radians
     * @param pitch in radians
     */
    SV_API void setCamera(const common::Vector3<float>& position, float yaw, float pitch);
    /**
     * @brief Make the camera movable or stationary
     * @param move whether movable or not
     */
    SV_API void setCameraMovable(bool move = true);

    //// Object
    /**
     * @brief Add a object
     * @param param Object initialize parameter (referring to struct PbjInitParam)
     */
    SV_API int addObj(const ObjInitParam& param);
    /**
     * @brief Update a object
     * @param param Object Update parameter (referring to struct PbjUpdateParam)
     */
    SV_API bool updateObj(const ObjUpdateParam& param);

    //// Axis
    /**
     * @brief Show or hide the 3 axes
     * @param show whether to show or not (true by default)
     */
    SV_API void showAxis(bool show = true);

    //// State
    /**
     * @brief Mouse state: 0 pressed, 1 released, 2 pressing down, 3 releasing up
     * @param button
     * 0/1/2: left/middle/right
     * 3/4: scroll down
     * 5/6: left button pressed and scroll up/down
     * 7/8: middle button pressed and scroll up/down
     * 35/36: right button pressed and scroll up/down
     */
    SV_API int getMouseState(int button);
    /**
     * @brief Key state: 0 pressed, 1 released, 2 pressing down, 3 releasing up
     * @param key char
     */
    SV_API int getKeyState(char key);

} // namespace simple_viewer