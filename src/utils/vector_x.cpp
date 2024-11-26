template <typename Scalar, size_t X>
ObjectPool<typename VectorX<Scalar, X>::VectorXData, X * sizeof(Scalar) * 256> VectorX<Scalar, X>::_pool;

template <typename Scalar, size_t X>
VectorX<Scalar, X>::VectorX(Scalar value): _data(_pool.create()) {
    for (size_t i = 0; i < X; i++) {
        _data->data[i] = value;
    }
}

template <typename Scalar, size_t X>
VectorX<Scalar, X>::VectorX(const VectorX& other): _data(_pool.create()) {
    for (size_t i = 0; i < X; i++) {
        _data->data[i] = other[i];
    }
}

template <typename Scalar, size_t X>
VectorX<Scalar, X>::VectorX(VectorX&& other) noexcept : _data(other._data) {
    other._data = nullptr;
}

template <typename Scalar, size_t X>
VectorX<Scalar, X>& VectorX<Scalar, X>::operator=(const VectorX& other) {
    if (this != &other) {
        for (size_t i = 0; i < X; i++) {
            _data->data[i] = other[i];
        }
    }
    return *this;
}

template <typename Scalar, size_t X>
VectorX<Scalar, X>& VectorX<Scalar, X>::operator=(VectorX&& other) noexcept {
    if (this != &other) {
        for (size_t i = 0; i < X; i++) {
            _data->data[i] = other[i];
        }
    }
    return *this;
}

template <typename Scalar, size_t X>
VectorX<Scalar, X> VectorX<Scalar, X>::operator-() const {
    VectorX result;
    for (size_t i = 0; i < X; i++) {
        result[i] = -_data->data[i];
    }
    return std::move(result);
}

template <typename Scalar, size_t X>
VectorX<Scalar, X> VectorX<Scalar, X>::operator+(const VectorX& other) const {
    VectorX result;
    for (size_t i = 0; i < X; i++) {
        result[i] = _data->data[i] + other[i];
    }
    return std::move(result);
}

template <typename Scalar, size_t X>
VectorX<Scalar, X> VectorX<Scalar, X>::operator-(const VectorX& other) const {
    VectorX result;
    for (size_t i = 0; i < X; i++) {
        result[i] = _data->data[i] - other[i];
    }
    return std::move(result);
}

template <typename Scalar, size_t X>
VectorX<Scalar, X> VectorX<Scalar, X>::operator*(Scalar s) const {
    VectorX result;
    for (size_t i = 0; i < X; i++) {
        result[i] = _data->data[i] * s;
    }
    return std::move(result);
}

template <typename Scalar, size_t X>
VectorX<Scalar, X> VectorX<Scalar, X>::operator/(Scalar s) const {
    VectorX result;
    for (size_t i = 0; i < X; i++) {
        result[i] = _data->data[i] / s;
    }
    return std::move(result);
}

template <typename Scalar, size_t X>
VectorX<Scalar, X>& VectorX<Scalar, X>::operator+=(const VectorX& other) {
    for (size_t i = 0; i < X; i++) {
        _data->data[i] += other[i];
    }
    return *this;
}

template <typename Scalar, size_t X>
VectorX<Scalar, X>& VectorX<Scalar, X>::operator-=(const VectorX& other) {
    for (size_t i = 0; i < X; i++) {
        _data->data[i] -= other[i];
    }
    return *this;
}

template <typename Scalar, size_t X>
VectorX<Scalar, X>& VectorX<Scalar, X>::operator*=(Scalar s) {
    for (size_t i = 0; i < X; i++) {
        _data->data[i] *= s;
    }
    return *this;
}

template <typename Scalar, size_t X>
VectorX<Scalar, X>& VectorX<Scalar, X>::operator/=(Scalar s) {
    for (size_t i = 0; i < X; i++) {
        _data->data[i] /= s;
    }
    return *this;
}

template <typename Scalar, size_t X>
Scalar VectorX<Scalar, X>::norm() const {
    Scalar sum = 0;
    for (size_t i = 0; i < X; i++) {
        sum += _data->data[i] * _data->data[i];
    }
    return std::sqrt(sum);
}

template <typename Scalar, size_t X>
Scalar VectorX<Scalar, X>::norm2() const {
    Scalar sum = 0;
    for (size_t i = 0; i < X; i++) {
        sum += _data->data[i] * _data->data[i];
    }
    return sum;
}

template <typename Scalar, size_t X>
VectorX<Scalar, X> VectorX<Scalar, X>::normalized() const {
    Scalar n = norm();
    if (n == 0) {
        return *this;
    }
    return *this / n;
}

template <typename Scalar, size_t X>
void VectorX<Scalar, X>::normalize() {
    Scalar n = norm();
    if (n == 0) {
        return;
    }
    *this /= n;
}

template <typename Scalar, size_t X>
VectorX<Scalar, X> VectorX<Scalar, X>::mult(const VectorX& v) const {
    VectorX result;
    for (size_t i = 0; i < X; i++) {
        result[i] = _data->data[i] * v[i];
    }
    return std::move(result);
}

template <typename Scalar, size_t X>
Scalar VectorX<Scalar, X>::dot(const VectorX& v) const {
    Scalar sum = 0;
    for (size_t i = 0; i < X; i++) {
        sum += _data->data[i] * v[i];
    }
    return sum;
}

template <typename Scalar, size_t X>
const VectorX<Scalar, X>& VectorX<Scalar, X>::zeros() {
    static VectorX zeros(0);
    return zeros;
}

template <typename Scalar, size_t X>
const VectorX<Scalar, X>& VectorX<Scalar, X>::ones() {
    static VectorX ones(1);
    return ones;
}

template <typename Scalar, size_t X>
    std::ostream& operator<<(std::ostream& os, const VectorX<Scalar, X>& v) {
    os << "[";
    for (size_t i = 0; i < X; i++) {
        os << v[i];
        if (i < X - 1) {
            os << " ";
        }
    }
    os << "]";
    return os;
}
