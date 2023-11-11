Vector3 Transform::operator*(const Vector3& v) const {
    return _basis * v + _origin;
}

Transform Transform::operator*(const Transform& t) const {
    return {_basis * t._basis, _basis * t._origin + _origin};
}

Transform& Transform::operator*=(const Transform& t) {
    _origin += _basis * t._origin;
    _basis *= t._basis;
    return *this;
}

const Matrix3x3& Transform::getBasis() const {
    return _basis;
}

const Vector3& Transform::getOrigin() const {
    return _origin;
}

Vector3 Transform::getAxis(int axis) const {
    return {_basis[0][axis], _basis[1][axis], _basis[2][axis]};
}

void Transform::setRotation(const Vector3& axis, real angle) {
    _basis.setRotation(axis, angle);
}

void Transform::setEulerRotation(real x, real y, real z, RotType type) {
    Matrix3x3 mat_y, mat_z;
    _basis.setRotation(Vector3::right(), x);
    mat_y.setRotation(Vector3::up(), y);
    mat_z.setRotation(Vector3::forward(), z);
    switch (type) {
        case RotType::S_XYZ: case RotType::ZYX:
            _basis = mat_z * mat_y * _basis;
            break;
        case RotType::S_YZX: case RotType::XZY:
            _basis = _basis * mat_z * mat_y;
            break;
        case RotType::S_ZXY: case RotType::YXZ:
            _basis = mat_y * _basis * mat_z;
            break;
        case RotType::S_XZY: case RotType::YZX:
            _basis = mat_y * mat_z * _basis;
            break;
        case RotType::S_ZYX: case RotType::XYZ:
            _basis = _basis * mat_y * mat_z;
            break;
        case RotType::S_YXZ: case RotType::ZXY:
            _basis = mat_z * _basis * mat_y;
            break;
    }
}

void Transform::setTranslation(const Vector3& translation) {
    _origin = translation;
}

void Transform::invert() {
    _basis.transpose();
    _origin = -_basis * _origin;
}

Transform Transform::inverse() const {
    auto new_basis = _basis.transposed();
    return {new_basis, -new_basis * _origin};
}

Vector3 Transform::inverseTransform(const Vector3& v) const {
    return _basis.transposed() * (v - _origin);
}

const Transform& Transform::identity() {
    static Transform identity;
    return identity;
}

std::ostream &operator<<(std::ostream& os, const Transform& t) {
    os << "[" << t._basis << std::endl << t._origin << "]";
    return os;
}
