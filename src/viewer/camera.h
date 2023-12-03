#pragma once

#include <mutex>
#include "public_include.h"

namespace simple_viewer {

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

    void setPosition(const SV_Vector3& position);
    void setYaw(float yaw);
    void setPitch(float pitch);
    void setProj(float fov_degree, float aspect, float near_, float far_);

    const SV_Transform& getTransform(int64_t time);
    float getProj(int i) const;

private:
    std::mutex _mutex_trans, _mutex_proj;
    float _yaw, _pitch;
    SV_Vector3 _position;
    float _fov, _aspect, _near, _far;
    SV_Transform _transform;
    float _proj[4];

    const float MoveSpeedMin = 0.05, MoveSpeedMax = 1., MoveSpeedStep = 0.05;
    float _move_speed = MoveSpeedMin;
    const float RotateSpeed = 0.004;
    float _rotate_speed = RotateSpeed;
    const float WheelStep = 8.0;
    const float KeyStep = 200.0;

    int _state_left = 1, _state_middle = 1, _state_right = 1;
    int _state_w = 1, _state_a = 1, _state_s = 1, _state_d = 1;
    int _state_q = 1, _state_e = 1;
    int _last_x = -1, _last_y = -1;
};

} // namespace simple_viewer