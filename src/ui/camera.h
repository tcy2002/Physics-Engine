#pragma once

#include <mutex>
#include <iostream>
#include "def.h"
#include "cg/transform.h"

namespace pe_ui {

/**
 * @brief A UE-stylized 3rd-person camera
 *
 * dof: yaw, pitch and position
 *
 * - left button down and drag: change yaw and move forward/backward
 *   (in horizontal plane)
 * - middle button down (or left and right button both down) and drag:
 *   move leftward/rightward/upward/downward (in vertical plane)
 * - right button down and drag: change yaw and pitch
 * - roll mouse wheel forward/backward: move forward/backward
 *   (relative to camera's direction)
 * - any mouse button down and roll mouse wheel forward/backward: change
 *   move speed up/down
 *
 * - w/a/s/d/q/e (forward/leftward/backward/rightward/downward/upward):
 *   move camera when any mouse button is down
 */
class Camera {
private:
    std::mutex _mutex_trans, _mutex_proj;
    real _yaw, _pitch;
    pe_cg::Vector3 _position;
    real _fov, _aspect, _near, _far;
    pe_cg::Transform _transform;
    real _proj[4];

    const real MoveSpeedMin = 0.05, MoveSpeedMax = 1., MoveSpeedStep = 0.05;
    real _move_speed = MoveSpeedMin;
    const real RotateSpeed = 0.004;
    real _rotate_speed = RotateSpeed;
    const real WheelStep = 8.0;
    const real KeyStep = 200.0;

    int _state_left = 1;
    int _state_middle = 1;
    int _state_right = 1;
    int _state_w = 1;
    int _state_a = 1;
    int _state_s = 1;
    int _state_d = 1;
    int _state_q = 1;
    int _state_e = 1;
    int _last_x = -1, _last_y = -1;

public:
    Camera();

    void mouse(int button, int state, int x, int y);
    void motion(int x, int y);
    void keyboard(unsigned char key, int state);
    void reshape(int width, int height);

    void setPosition(const pe_cg::Vector3& position);
    void setYaw(real yaw);
    void setPitch(real pitch);
    void setProj(real fov_degree, real aspect, real near, real far);

    const pe_cg::Transform& getTransform(int64_t time);
    real getProj(int i) const;
};

} // namespace pe_ui