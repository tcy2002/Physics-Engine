template <typename Scalar>
Scalar* Matrix3x3<Scalar>::operator[](int i) {
    return _m[i];
}

template <typename Scalar>
const Scalar* Matrix3x3<Scalar>::operator[](int i) const {
    return _m[i];
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::operator-() const {
    Matrix3x3 res;
    res._m[0][0] = -_m[0][0]; res._m[0][1] = -_m[0][1]; res._m[0][2] = -_m[0][2];
    res._m[1][0] = -_m[1][0]; res._m[1][1] = -_m[1][1]; res._m[1][2] = -_m[1][2];
    res._m[2][0] = -_m[2][0]; res._m[2][1] = -_m[2][1]; res._m[2][2] = -_m[2][2];
    return res;
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::operator+(const Matrix3x3& mat) const {
    Matrix3x3 res;
    res._m[0][0] = _m[0][0] + mat._m[0][0];
    res._m[0][1] = _m[0][1] + mat._m[0][1];
    res._m[0][2] = _m[0][2] + mat._m[0][2];
    res._m[1][0] = _m[1][0] + mat._m[1][0];
    res._m[1][1] = _m[1][1] + mat._m[1][1];
    res._m[1][2] = _m[1][2] + mat._m[1][2];
    res._m[2][0] = _m[2][0] + mat._m[2][0];
    res._m[2][1] = _m[2][1] + mat._m[2][1];
    res._m[2][2] = _m[2][2] + mat._m[2][2];
    return res;
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::operator-(const Matrix3x3& mat) const {
    Matrix3x3 res;
    res._m[0][0] = _m[0][0] - mat._m[0][0];
    res._m[0][1] = _m[0][1] - mat._m[0][1];
    res._m[0][2] = _m[0][2] - mat._m[0][2];
    res._m[1][0] = _m[1][0] - mat._m[1][0];
    res._m[1][1] = _m[1][1] - mat._m[1][1];
    res._m[1][2] = _m[1][2] - mat._m[1][2];
    res._m[2][0] = _m[2][0] - mat._m[2][0];
    res._m[2][1] = _m[2][1] - mat._m[2][1];
    res._m[2][2] = _m[2][2] - mat._m[2][2];
    return res;
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::operator*(Scalar k) const {
    Matrix3x3 res;
    res._m[0][0] = _m[0][0] * k; res._m[0][1] = _m[0][1] * k; res._m[0][2] = _m[0][2] * k;
    res._m[1][0] = _m[1][0] * k; res._m[1][1] = _m[1][1] * k; res._m[1][2] = _m[1][2] * k;
    res._m[2][0] = _m[2][0] * k; res._m[2][1] = _m[2][1] * k; res._m[2][2] = _m[2][2] * k;
    return res;
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::operator/(Scalar k) const {
    Matrix3x3 res;
    res._m[0][0] = _m[0][0] / k;
    res._m[0][1] = _m[0][1] / k;
    res._m[0][2] = _m[0][2] / k;
    res._m[1][0] = _m[1][0] / k;
    res._m[1][1] = _m[1][1] / k;
    res._m[1][2] = _m[1][2] / k;
    res._m[2][0] = _m[2][0] / k;
    res._m[2][1] = _m[2][1] / k;
    res._m[2][2] = _m[2][2] / k;
    return res;
}

template <typename Scalar>
Matrix3x3<Scalar>& Matrix3x3<Scalar>::operator+=(const Matrix3x3& mat) {
    _m[0][0] += mat._m[0][0];
    _m[0][1] += mat._m[0][1];
    _m[0][2] += mat._m[0][2];
    _m[1][0] += mat._m[1][0];
    _m[1][1] += mat._m[1][1];
    _m[1][2] += mat._m[1][2];
    _m[2][0] += mat._m[2][0];
    _m[2][1] += mat._m[2][1];
    _m[2][2] += mat._m[2][2];
    return *this;
}

template <typename Scalar>
Matrix3x3<Scalar>& Matrix3x3<Scalar>::operator-=(const Matrix3x3& mat) {
    _m[0][0] -= mat._m[0][0];
    _m[0][1] -= mat._m[0][1];
    _m[0][2] -= mat._m[0][2];
    _m[1][0] -= mat._m[1][0];
    _m[1][1] -= mat._m[1][1];
    _m[1][2] -= mat._m[1][2];
    _m[2][0] -= mat._m[2][0];
    _m[2][1] -= mat._m[2][1];
    _m[2][2] -= mat._m[2][2];
    return *this;
}

template <typename Scalar>
Matrix3x3<Scalar>& Matrix3x3<Scalar>::operator*=(Scalar k) {
    _m[0][0] *= k; _m[0][1] *= k; _m[0][2] *= k;
    _m[1][0] *= k; _m[1][1] *= k; _m[1][2] *= k;
    _m[2][0] *= k; _m[2][1] *= k; _m[2][2] *= k;
    return *this;
}

template <typename Scalar>
Matrix3x3<Scalar>& Matrix3x3<Scalar>::operator/=(Scalar k) {
    _m[0][0] /= k; _m[0][1] /= k; _m[0][2] /= k;
    _m[1][0] /= k; _m[1][1] /= k; _m[1][2] /= k;
    _m[2][0] /= k; _m[2][1] /= k; _m[2][2] /= k;
    return *this;
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::operator*(const Matrix3x3& mat) const {
    Matrix3x3 res;
    res._m[0][0] = _m[0][0] * mat._m[0][0] + _m[0][1] * mat._m[1][0] + _m[0][2] * mat._m[2][0];
    res._m[0][1] = _m[0][0] * mat._m[0][1] + _m[0][1] * mat._m[1][1] + _m[0][2] * mat._m[2][1];
    res._m[0][2] = _m[0][0] * mat._m[0][2] + _m[0][1] * mat._m[1][2] + _m[0][2] * mat._m[2][2];
    res._m[1][0] = _m[1][0] * mat._m[0][0] + _m[1][1] * mat._m[1][0] + _m[1][2] * mat._m[2][0];
    res._m[1][1] = _m[1][0] * mat._m[0][1] + _m[1][1] * mat._m[1][1] + _m[1][2] * mat._m[2][1];
    res._m[1][2] = _m[1][0] * mat._m[0][2] + _m[1][1] * mat._m[1][2] + _m[1][2] * mat._m[2][2];
    res._m[2][0] = _m[2][0] * mat._m[0][0] + _m[2][1] * mat._m[1][0] + _m[2][2] * mat._m[2][0];
    res._m[2][1] = _m[2][0] * mat._m[0][1] + _m[2][1] * mat._m[1][1] + _m[2][2] * mat._m[2][1];
    res._m[2][2] = _m[2][0] * mat._m[0][2] + _m[2][1] * mat._m[1][2] + _m[2][2] * mat._m[2][2];
    return res;
}

template <typename Scalar>
Matrix3x3<Scalar>& Matrix3x3<Scalar>::operator*=(const Matrix3x3& mat) {
    Scalar tmp[3][3];
    tmp[0][0] = _m[0][0] * mat._m[0][0] + _m[0][1] * mat._m[1][0] + _m[0][2] * mat._m[2][0];
    tmp[0][1] = _m[0][0] * mat._m[0][1] + _m[0][1] * mat._m[1][1] + _m[0][2] * mat._m[2][1];
    tmp[0][2] = _m[0][0] * mat._m[0][2] + _m[0][1] * mat._m[1][2] + _m[0][2] * mat._m[2][2];
    tmp[1][0] = _m[1][0] * mat._m[0][0] + _m[1][1] * mat._m[1][0] + _m[1][2] * mat._m[2][0];
    tmp[1][1] = _m[1][0] * mat._m[0][1] + _m[1][1] * mat._m[1][1] + _m[1][2] * mat._m[2][1];
    tmp[1][2] = _m[1][0] * mat._m[0][2] + _m[1][1] * mat._m[1][2] + _m[1][2] * mat._m[2][2];
    tmp[2][0] = _m[2][0] * mat._m[0][0] + _m[2][1] * mat._m[1][0] + _m[2][2] * mat._m[2][0];
    tmp[2][1] = _m[2][0] * mat._m[0][1] + _m[2][1] * mat._m[1][1] + _m[2][2] * mat._m[2][1];
    tmp[2][2] = _m[2][0] * mat._m[0][2] + _m[2][1] * mat._m[1][2] + _m[2][2] * mat._m[2][2];
    _m[0][0] = tmp[0][0]; _m[0][1] = tmp[0][1]; _m[0][2] = tmp[0][2];
    _m[1][0] = tmp[1][0]; _m[1][1] = tmp[1][1]; _m[1][2] = tmp[1][2];
    _m[2][0] = tmp[2][0]; _m[2][1] = tmp[2][1]; _m[2][2] = tmp[2][2];
    return *this;
}

template <typename Scalar>
Vector3<Scalar> Matrix3x3<Scalar>::operator*(const Vector3<Scalar>& vec) const {
    Vector3<Scalar> res;
    res.x = _m[0][0] * vec.x + _m[0][1] * vec.y + _m[0][2] * vec.z;
    res.y = _m[1][0] * vec.x + _m[1][1] * vec.y + _m[1][2] * vec.z;
    res.z = _m[2][0] * vec.x + _m[2][1] * vec.y + _m[2][2] * vec.z;
    return res;
}

template <typename Scalar>
Vector3<Scalar> Matrix3x3<Scalar>::getColumn(int axis) const {
    return {_m[0][axis], _m[1][axis], _m[2][axis]};
}

template <typename Scalar>
Scalar Matrix3x3<Scalar>::determinant() const {
    return _m[0][0] * (_m[1][1] * _m[2][2] - _m[2][1] * _m[1][2])
           + _m[0][1] * (_m[2][0] * _m[1][2] - _m[1][0] * _m[2][2])
           + _m[0][2] * (_m[1][0] * _m[2][1] - _m[2][0] * _m[1][1]);
}

template <typename Scalar>
Scalar Matrix3x3<Scalar>::trace() const {
    return _m[0][0] + _m[1][1] + _m[2][2];
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::mult(const Matrix3x3& mat) const {
    Matrix3x3<Scalar> res;
    res._m[0][0] = _m[0][0] * mat._m[0][0];
    res._m[0][1] = _m[0][1] * mat._m[0][1];
    res._m[0][2] = _m[0][2] * mat._m[0][2];
    res._m[1][0] = _m[1][0] * mat._m[1][0];
    res._m[1][1] = _m[1][1] * mat._m[1][1];
    res._m[1][2] = _m[1][2] * mat._m[1][2];
    res._m[2][0] = _m[2][0] * mat._m[2][0];
    res._m[2][1] = _m[2][1] * mat._m[2][1];
    res._m[2][2] = _m[2][2] * mat._m[2][2];
    return res;
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::transposed() const {
    Matrix3x3 res;
    res._m[0][0] = _m[0][0]; res._m[1][0] = _m[0][1]; res._m[2][0] = _m[0][2];
    res._m[0][1] = _m[1][0]; res._m[1][1] = _m[1][1]; res._m[2][1] = _m[1][2];
    res._m[0][2] = _m[2][0]; res._m[1][2] = _m[2][1]; res._m[2][2] = _m[2][2];
    return res;
}

template <typename Scalar>
void Matrix3x3<Scalar>::transpose() {
    Scalar tmp;
    tmp = _m[0][1]; _m[0][1] = _m[1][0]; _m[1][0] = tmp;
    tmp = _m[0][2]; _m[0][2] = _m[2][0]; _m[2][0] = tmp;
    tmp = _m[1][2]; _m[1][2] = _m[2][1]; _m[2][1] = tmp;
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::inverse() const {
    Matrix3x3 res;
    Scalar det = determinant();
    if (det == 0) return res;
    res._m[0][0] = (_m[1][1] * _m[2][2] - _m[2][1] * _m[1][2]) / det;
    res._m[0][1] = (_m[2][1] * _m[0][2] - _m[0][1] * _m[2][2]) / det;
    res._m[0][2] = (_m[0][1] * _m[1][2] - _m[1][1] * _m[0][2]) / det;
    res._m[1][0] = (_m[2][0] * _m[1][2] - _m[1][0] * _m[2][2]) / det;
    res._m[1][1] = (_m[0][0] * _m[2][2] - _m[2][0] * _m[0][2]) / det;
    res._m[1][2] = (_m[1][0] * _m[0][2] - _m[0][0] * _m[1][2]) / det;
    res._m[2][0] = (_m[1][0] * _m[2][1] - _m[2][0] * _m[1][1]) / det;
    res._m[2][1] = (_m[2][0] * _m[0][1] - _m[0][0] * _m[2][1]) / det;
    res._m[2][2] = (_m[0][0] * _m[1][1] - _m[1][0] * _m[0][1]) / det;
    return res;
}

template <typename Scalar>
void Matrix3x3<Scalar>::invert() {
    Scalar det = determinant();
    if (det == 0) return;
    Scalar tmp[3][3];
    tmp[0][0] = (_m[1][1] * _m[2][2] - _m[2][1] * _m[1][2]) / det;
    tmp[0][1] = (_m[2][1] * _m[0][2] - _m[0][1] * _m[2][2]) / det;
    tmp[0][2] = (_m[0][1] * _m[1][2] - _m[1][1] * _m[0][2]) / det;
    tmp[1][0] = (_m[2][0] * _m[1][2] - _m[1][0] * _m[2][2]) / det;
    tmp[1][1] = (_m[0][0] * _m[2][2] - _m[2][0] * _m[0][2]) / det;
    tmp[1][2] = (_m[1][0] * _m[0][2] - _m[0][0] * _m[1][2]) / det;
    tmp[2][0] = (_m[1][0] * _m[2][1] - _m[2][0] * _m[1][1]) / det;
    tmp[2][1] = (_m[2][0] * _m[0][1] - _m[0][0] * _m[2][1]) / det;
    tmp[2][2] = (_m[0][0] * _m[1][1] - _m[1][0] * _m[0][1]) / det;
    _m[0][0] = tmp[0][0]; _m[0][1] = tmp[0][1]; _m[0][2] = tmp[0][2];
    _m[1][0] = tmp[1][0]; _m[1][1] = tmp[1][1]; _m[1][2] = tmp[1][2];
    _m[2][0] = tmp[2][0]; _m[2][1] = tmp[2][1]; _m[2][2] = tmp[2][2];
}

template <typename Scalar>
void Matrix3x3<Scalar>::setRotation(const Vector3<Scalar> &axis, Scalar angle) {
    Scalar c = cos(angle), s = sin(angle);
    auto n = axis.normalized();
    Scalar x = n.x, y = n.y, z = n.z;
    _m[0][0] = c + (1 - c) * x * x;
    _m[0][1] = (1 - c) * x * y - s * z;
    _m[0][2] = (1 - c) * x * z + s * y;
    _m[1][0] = (1 - c) * y * x + s * z;
    _m[1][1] = c + (1 - c) * y * y;
    _m[1][2] = (1 - c) * y * z - s * x;
    _m[2][0] = (1 - c) * z * x - s * y;
    _m[2][1] = (1 - c) * z * y + s * x;
    _m[2][2] = c + (1 - c) * z * z;
}

template <typename Scalar>
Matrix3x3<Scalar> Matrix3x3<Scalar>::getAbsolute() const {
    Matrix3x3 res;
    res._m[0][0] = std::abs(_m[0][0]);
    res._m[0][1] = std::abs(_m[0][1]);
    res._m[0][2] = std::abs(_m[0][2]);
    res._m[1][0] = std::abs(_m[1][0]);
    res._m[1][1] = std::abs(_m[1][1]);
    res._m[1][2] = std::abs(_m[1][2]);
    res._m[2][0] = std::abs(_m[2][0]);
    res._m[2][1] = std::abs(_m[2][1]);
    res._m[2][2] = std::abs(_m[2][2]);
    return res;
}

template <typename Scalar>
Vector3<Scalar> Matrix3x3<Scalar>::getDiag() const {
    return {_m[0][0], _m[1][1], _m[2][2]};
}

template <typename Scalar>
const Matrix3x3<Scalar>& Matrix3x3<Scalar>::identity() {
    static Matrix3x3 identity(1, 0, 0, 0, 1, 0, 0, 0, 1);
    return identity;
}

template <typename Scalar>
const Matrix3x3<Scalar>& Matrix3x3<Scalar>::zeros() {
    static Matrix3x3 zeros;
    return zeros;
}

template <typename Scalar>
const Matrix3x3<Scalar>& Matrix3x3<Scalar>::ones() {
    static Matrix3x3 ones(1, 1, 1, 1, 1, 1, 1, 1, 1);
    return ones;
}

template <typename Scalar>
Matrix3x3<Scalar> operator*(Scalar k, const Matrix3x3<Scalar>& mat) {
    return mat * k;
}

template <typename Scalar>
Vector3<Scalar> operator*(const Vector3<Scalar>& vec, const Matrix3x3<Scalar>& mat) {
    Scalar x = mat[0][0] * vec.x + mat[1][0] * vec.y + mat[2][0] * vec.z;
    Scalar y = mat[0][1] * vec.x + mat[1][1] * vec.y + mat[2][1] * vec.z;
    Scalar z = mat[0][2] * vec.x + mat[1][2] * vec.y + mat[2][2] * vec.z;
    return {x, y, z};
}

template <typename Scalar>
std::ostream& operator<<(std::ostream& os, const Matrix3x3<Scalar>& mat) {
    os << "[" << mat[0][0] << " " << mat[0][1] << " " << mat[0][2] << std::endl;
    os << " " << mat[1][0] << " " << mat[1][1] << " " << mat[1][2] << std::endl;
    os << " " << mat[2][0] << " " << mat[2][1] << " " << mat[2][2] << "]";
    return os;
}
