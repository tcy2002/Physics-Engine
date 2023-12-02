#pragma once

#include <mutex>
#include <iostream>
#include "def.h"
#include "common/transform.h"

namespace pe_viewer {

/**
 * @brief A UE-stylized 3rd-person camera
 *
 * Thread-safe\n
 *
 * Dof: yaw, pitch and position\n
 *
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
 */
class Camera {
public:
    Camera();

    void mouse(int button, int state, int x, int y);
    void motion(int x, int y);
    void keyboard(unsigned char key, int state);
    void reshape(int width, int height);

    void setPosition(const pe_common::Vector3& position);
    void setYaw(PEReal yaw);
    void setPitch(PEReal pitch);
    void setProj(PEReal fov_degree, PEReal aspect, PEReal near_, PEReal far_);

    const pe_common::Transform& getTransform(int64_t time);
    PEReal getProj(int i) const;

private:
    std::mutex _mutex_trans, _mutex_proj;
    PEReal _yaw, _pitch;
    pe_common::Vector3 _position;
    PEReal _fov, _aspect, _near, _far;
    pe_common::Transform _transform;
    PEReal _proj[4];

    const PEReal MoveSpeedMin = 0.05, MoveSpeedMax = 1., MoveSpeedStep = 0.05;
    PEReal _move_speed = MoveSpeedMin;
    const PEReal RotateSpeed = 0.004;
    PEReal _rotate_speed = RotateSpeed;
    const PEReal WheelStep = 8.0;
    const PEReal KeyStep = 200.0;

    int _state_left = 1, _state_middle = 1, _state_right = 1;
    int _state_w = 1, _state_a = 1, _state_s = 1, _state_d = 1;
    int _state_q = 1, _state_e = 1;
    int _last_x = -1, _last_y = -1;
};

} // namespace pe_viewer