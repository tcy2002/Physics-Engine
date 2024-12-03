template <typename Scalar>
Vector3<Scalar> Transform<Scalar>::operator*(const Vector3<Scalar>& v) const {
    return _basis * v + _origin;
}

template <typename Scalar>
Transform<Scalar> Transform<Scalar>::operator*(const Transform<Scalar>& t) const {
    return {_basis * t._basis, _basis * t._origin + _origin};
}

template <typename Scalar>
Transform<Scalar>& Transform<Scalar>::operator*=(const Transform<Scalar>& t) {
    _origin += _basis * t._origin;
    _basis *= t._basis;
    return *this;
}

template <typename Scalar>
Vector3<Scalar> Transform<Scalar>::getAxis(int axis) const {
    return _basis.col(axis);
}

template <typename Scalar>
void Transform<Scalar>::setRotation(const Vector3<Scalar>& axis, Scalar angle) {
    _basis = Eigen::AngleAxis<Scalar>(angle, axis.normalized()).toRotationMatrix();
}

template <typename Scalar>
void Transform<Scalar>::setEulerRotation(Scalar x, Scalar y, Scalar z, RotType type) {
    Matrix3<Scalar> mat_y, mat_z;
    _basis = Eigen::AngleAxis<Scalar>(x, Vector3<Scalar>::UnitX()).toRotationMatrix();
    mat_y = Eigen::AngleAxis<Scalar>(y, Vector3<Scalar>::UnitY()).toRotationMatrix();
    mat_z = Eigen::AngleAxis<Scalar>(z, Vector3<Scalar>::UnitZ()).toRotationMatrix();
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

template <typename Scalar>
void Transform<Scalar>::setTranslation(const Vector3<Scalar>& translation) {
    _origin = translation;
}

template <typename Scalar>
void Transform<Scalar>::invert() {
    _basis.transpose();
    _origin = -_basis * _origin;
}

template <typename Scalar>
Transform<Scalar> Transform<Scalar>::inverse() const {
    auto new_basis = _basis.transpose();
    return {new_basis, -new_basis * _origin};
}

template <typename Scalar>
Vector3<Scalar> Transform<Scalar>::inverseTransform(const Vector3<Scalar>& v) const {
    return _basis.transpose() * (v - _origin);
}

template <typename Scalar>
const Transform<Scalar>& Transform<Scalar>::Identity() {
    static Transform identity;
    return identity;
}

template <typename Scalar>
std::ostream &operator<<(std::ostream& os, const Transform<Scalar>& t) {
    os << "[" << t.getBasis() << std::endl << t.getOrigin() << "]";
    return os;
}
