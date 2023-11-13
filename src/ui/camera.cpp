#include "camera.h"

using namespace pe_ui;

Camera::Camera() :
    _yaw(0.), _pitch(0.),
    _position(pe_cg::Vector3::zeros()),
    _fov(M_PI / 2.),
    _transform(pe_cg::Transform::identity()),
    _proj{1., 1., -1.0002, -0.20002} {}

void Camera::mouse(int button, int state, int, int) {
    std::lock_guard<std::mutex> lock(_mutex_trans);

    switch (button) {
        case 0: _state_left = state; break;
        case 1: _state_middle = state; break;
        case 2: _state_right = state; break;
        default: break;
    }

    if (button <= 2 || state == 0) {
        _last_x = -1, _last_y = -1;
        return;
    }

    if (button == 5 || button == 7 || button == 35) {
        _move_speed += MoveSpeedStep;
        if (_move_speed > MoveSpeedMax) _move_speed = MoveSpeedMax;
    } else if (button == 6 || button == 8 || button == 36) {
        _move_speed -= MoveSpeedStep;
        if (_move_speed < MoveSpeedMin) _move_speed = MoveSpeedMin;
    } else {
        auto forward = -_transform.getAxis(2);
        _position += forward * ((button == 3 ? 1 : -1) * _move_speed * WheelStep);
    }
}

void Camera::motion(int x, int y) {
    std::lock_guard<std::mutex> lock(_mutex_trans);

    int dx = _last_x != -1 ? x - _last_x : 0, dy = _last_y != -1 ? y - _last_y : 0;
    _last_x = x, _last_y = y;

    if (_state_left == 0 && _state_right == 1) {
        _yaw -= dx * _rotate_speed;
        if (_yaw > M_PI) _yaw -= 2 * M_PI;
        else if (_yaw < -M_PI) _yaw += 2 * M_PI;
        pe_cg::Vector3 front(sin(_yaw), 0., cos(_yaw));
        _position += front * (dy * _move_speed);
    } else if ((_state_left == 1 && _state_middle == 0) || (_state_left == 0 && _state_right == 0)) {
        real right_yaw = _yaw + M_PI / 2;
        pe_cg::Vector3 right(sin(right_yaw), 0., cos(right_yaw));
        _position += right * (dx * _move_speed);
        _position -= pe_cg::Vector3::up() * (dy * _move_speed);
    } else if (_state_left == 1 && _state_middle == 1 && _state_right == 0) {
        _yaw -= dx * _rotate_speed;
        if (_yaw > M_PI) _yaw -= 2 * M_PI;
        else if (_yaw < -M_PI) _yaw += 2 * M_PI;
        _pitch += dy * _rotate_speed;
        if (_pitch > M_PI / 2) _pitch = M_PI / 2;
        else if (_pitch < -M_PI / 2) _pitch = -M_PI / 2;
    }
}

void Camera::keyboard(unsigned char key, int state) {
    std::lock_guard<std::mutex> lock(_mutex_trans);

    switch (key) {
        case 'w': _state_w = state; break;
        case 'a': _state_a = state; break;
        case 's': _state_s = state; break;
        case 'd': _state_d = state; break;
        case 'q': _state_q = state; break;
        case 'e': _state_e = state; break;
        default: break;
    }
}

void Camera::reshape(int width, int height) {
    std::lock_guard<std::mutex> lock(_mutex_proj);
    _aspect = 1.0 * width / height;
    _proj[0] = _proj[1] / _aspect;
}

void Camera::setPosition(const pe_cg::Vector3& position) {
    std::lock_guard<std::mutex> lock(_mutex_trans);
    _position = position;
}

void Camera::setYaw(real yaw) {
    std::lock_guard<std::mutex> lock(_mutex_trans);
    _yaw = yaw;
}

void Camera::setPitch(real pitch) {
    std::lock_guard<std::mutex> lock(_mutex_trans);
    _pitch = pitch;
}

void Camera::setProj(real fov_degree, real aspect, real near, real far) {
    std::lock_guard<std::mutex> lock(_mutex_proj);
    _fov = fov_degree * M_PI / 180.;
    _aspect = aspect;
    _near = near;
    _far = far;
    _proj[1] = 1. / tan(_fov / 2.);
    _proj[0] = _proj[1] / _aspect;
    _proj[2] = -(_far + _near) / (_far - _near);
    _proj[3] = -2. * _far * _near / (_far - _near);
}

const pe_cg::Transform& Camera::getTransform(int64_t time) {
    static int64_t _last_time = time;
    real dt = (int)(time - _last_time) / 1000.;
    _last_time = time;

    std::lock_guard<std::mutex> lock(_mutex_trans);

    if (_state_left == 0 || _state_right == 0 || _state_middle == 0) {
        auto forward = _transform.getAxis(2);
        auto right = _transform.getAxis(0);
        auto up = _transform.getAxis(1);
        _position += forward * ((_state_w - _state_s) * _move_speed * KeyStep * dt);
        _position -= right * ((_state_d - _state_a) * _move_speed * KeyStep * dt);
        _position += up * ((_state_e - _state_q) * _move_speed * KeyStep * dt);
    }

    _transform.setEulerRotation(-_pitch, _yaw, 0., pe_cg::RotType::S_ZXY);
    _transform.setTranslation(_position);
    return _transform;
}

real Camera::getProj(int i) const {
    return _proj[i];
}
