real* Matrix3x3::operator[](int i) {
    return _m[i];
}

const real* Matrix3x3::operator[](int i) const {
    return _m[i];
}

Matrix3x3 Matrix3x3::operator-() const {
    Matrix3x3 res;
    res._m[0][0] = -_m[0][0]; res._m[0][1] = -_m[0][1]; res._m[0][2] = -_m[0][2];
    res._m[1][0] = -_m[1][0]; res._m[1][1] = -_m[1][1]; res._m[1][2] = -_m[1][2];
    res._m[2][0] = -_m[2][0]; res._m[2][1] = -_m[2][1]; res._m[2][2] = -_m[2][2];
    return res;
}

Matrix3x3 Matrix3x3::operator+(const Matrix3x3& mat) const {
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

Matrix3x3 Matrix3x3::operator-(const Matrix3x3& mat) const {
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

Matrix3x3 Matrix3x3::operator*(real k) const {
    Matrix3x3 res;
    res._m[0][0] = _m[0][0] * k; res._m[0][1] = _m[0][1] * k; res._m[0][2] = _m[0][2] * k;
    res._m[1][0] = _m[1][0] * k; res._m[1][1] = _m[1][1] * k; res._m[1][2] = _m[1][2] * k;
    res._m[2][0] = _m[2][0] * k; res._m[2][1] = _m[2][1] * k; res._m[2][2] = _m[2][2] * k;
    return res;
}

Matrix3x3 Matrix3x3::operator/(real k) const {
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

Matrix3x3& Matrix3x3::operator+=(const Matrix3x3& mat) {
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

Matrix3x3& Matrix3x3::operator-=(const Matrix3x3& mat) {
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

Matrix3x3& Matrix3x3::operator*=(real k) {
    _m[0][0] *= k; _m[0][1] *= k; _m[0][2] *= k;
    _m[1][0] *= k; _m[1][1] *= k; _m[1][2] *= k;
    _m[2][0] *= k; _m[2][1] *= k; _m[2][2] *= k;
    return *this;
}

Matrix3x3& Matrix3x3::operator/=(real k) {
    _m[0][0] /= k; _m[0][1] /= k; _m[0][2] /= k;
    _m[1][0] /= k; _m[1][1] /= k; _m[1][2] /= k;
    _m[2][0] /= k; _m[2][1] /= k; _m[2][2] /= k;
    return *this;
}

Matrix3x3 Matrix3x3::operator*(const Matrix3x3& mat) const {
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

Matrix3x3& Matrix3x3::operator*=(const Matrix3x3& mat) {
    real tmp[3][3];
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

Vector3 Matrix3x3::operator*(const Vector3& vec) const {
    Vector3 res;
    res.x = _m[0][0] * vec.x + _m[0][1] * vec.y + _m[0][2] * vec.z;
    res.y = _m[1][0] * vec.x + _m[1][1] * vec.y + _m[1][2] * vec.z;
    res.z = _m[2][0] * vec.x + _m[2][1] * vec.y + _m[2][2] * vec.z;
    return res;
}

real Matrix3x3::determinant() const {
    return _m[0][0] * (_m[1][1] * _m[2][2] - _m[2][1] * _m[1][2])
           + _m[0][1] * (_m[2][0] * _m[1][2] - _m[1][0] * _m[2][2])
           + _m[0][2] * (_m[1][0] * _m[2][1] - _m[2][0] * _m[1][1]);
}

real Matrix3x3::trace() const {
    return _m[0][0] + _m[1][1] + _m[2][2];
}

Matrix3x3 Matrix3x3::transposed() const {
    Matrix3x3 res;
    res._m[0][0] = _m[0][0]; res._m[1][0] = _m[0][1]; res._m[2][0] = _m[0][2];
    res._m[0][1] = _m[1][0]; res._m[1][1] = _m[1][1]; res._m[2][1] = _m[1][2];
    res._m[0][2] = _m[2][0]; res._m[1][2] = _m[2][1]; res._m[2][2] = _m[2][2];
    return res;
}

void Matrix3x3::transpose() {
    real tmp;
    tmp = _m[0][1]; _m[0][1] = _m[1][0]; _m[1][0] = tmp;
    tmp = _m[0][2]; _m[0][2] = _m[2][0]; _m[2][0] = tmp;
    tmp = _m[1][2]; _m[1][2] = _m[2][1]; _m[2][1] = tmp;
}

Matrix3x3 Matrix3x3::inverse() const {
    Matrix3x3 res;
    real det = determinant();
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

void Matrix3x3::invert() {
    real det = determinant();
    if (det == 0) return;
    real tmp[3][3];
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

void Matrix3x3::setRotation(const Vector3 &axis, real angle) {
    real c = cos(angle), s = sin(angle);
    auto n = axis.normalized();
    real x = n.x, y = n.y, z = n.z;
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

const Matrix3x3& Matrix3x3::identity() {
    static Matrix3x3 identity(1, 0, 0, 0, 1, 0, 0, 0, 1);
    return identity;
}

const Matrix3x3& Matrix3x3::zeros() {
    static Matrix3x3 zeros;
    return zeros;
}

const Matrix3x3& Matrix3x3::ones() {
    static Matrix3x3 ones(1, 1, 1, 1, 1, 1, 1, 1, 1);
    return ones;
}

std::ostream& operator<<(std::ostream& os, const Matrix3x3& mat) {
    os << "[" << mat[0][0] << " " << mat[0][1] << " " << mat[0][2] << std::endl;
    os << " " << mat[1][0] << " " << mat[1][1] << " " << mat[1][2] << std::endl;
    os << " " << mat[2][0] << " " << mat[2][1] << " " << mat[2][2] << "]";
    return os;
}
