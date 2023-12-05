#pragma once

#include <mutex>
#include "public_include.h"

namespace simple_viewer {

/**
 * @brief A UE-stylized, thread-safe 3rd-person camera
 */
class Camera {
public:
    Camera();

    void mouse(int button, int state, int x, int y);
    void motion(int x, int y);
    void keyboard(unsigned char key, int state);
    void reshape(int width, int height);

    void setPosition(const Vector3& position);
    void setYaw(float yaw);
    void setPitch(float pitch);
    void setProj(float fov_degree, float aspect, float near_, float far_);

    const Transform& getTransform(long long time);
    float getProj(int i) const;

    void reset();

private:
    std::mutex _mutex_trans, _mutex_proj;
    float _yaw, _pitch;
    Vector3 _position;
    float _fov, _aspect, _near, _far;
    Transform _transform;
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
    long long _last_time = -1;
};

} // namespace simple_viewer