template <typename Scalar>
VectorX<Scalar>::VectorX(size_t size, Scalar value): _data(new Scalar[size]), _size(size) {
    for (size_t i = 0; i < size; i++) {
        _data[i] = value;
    }
}

template <typename Scalar>
VectorX<Scalar>::VectorX(const VectorX& other): _data(new Scalar[other._size]), _size(other._size) {
    for (size_t i = 0; i < _size; i++) {
        _data[i] = other._data[i];
    }
}

template <typename Scalar>
VectorX<Scalar>::VectorX(VectorX&& other) noexcept : _data(other._data), _size(other._size) {
    other._data = nullptr;
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator=(const VectorX& other) {
    if (this != &other) {
        if (_size != other._size) {
            delete[] _data;
            _data = new Scalar[other._size];
            _size = other._size;
        }
        for (size_t i = 0; i < _size; i++) {
            _data[i] = other._data[i];
        }
    }
    return *this;
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator=(VectorX&& other) noexcept {
    if (this != &other) {
        if (_size != other._size) {
            delete[] _data;
            _data = other._data;
            _size = other._size;
            other._data = nullptr;
            return *this;
        }
        for (size_t i = 0; i < _size; i++) {
            _data[i] = other._data[i];
        }
    }
    return *this;
}

template<typename Scalar>
void VectorX<Scalar>::resize(size_t size) {
    if (size != _size) {
        delete[] _data;
        _data = new Scalar[size];
        _size = size;
    }
}

template<typename Scalar>
void VectorX<Scalar>::resize(size_t size, Scalar value) {
    resize(size);
    for (size_t i = 0; i < size; i++) {
        _data[i] = value;
    }
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::operator-() const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = -_data[i];
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::operator+(const VectorX& other) const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = _data[i] + other._data[i];
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::operator-(const VectorX& other) const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = _data[i] - other._data[i];
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::operator*(Scalar s) const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = _data[i] * s;
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::operator/(Scalar s) const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = _data[i] / s;
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator+=(const VectorX& other) {
    for (size_t i = 0; i < _size; i++) {
        _data[i] += other._data[i];
    }
    return *this;
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator-=(const VectorX& other) {
    for (size_t i = 0; i < _size; i++) {
        _data[i] -= other._data[i];
    }
    return *this;
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator*=(Scalar s) {
    for (size_t i = 0; i < _size; i++) {
        _data[i] *= s;
    }
    return *this;
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator/=(Scalar s) {
    for (size_t i = 0; i < _size; i++) {
        _data[i] /= s;
    }
    return *this;
}

template <typename Scalar>
Scalar VectorX<Scalar>::norm() const {
    Scalar sum = 0;
    for (size_t i = 0; i < _size; i++) {
        sum += _data[i] * _data[i];
    }
    return std::sqrt(sum);
}

template <typename Scalar>
Scalar VectorX<Scalar>::norm2() const {
    Scalar sum = 0;
    for (size_t i = 0; i < _size; i++) {
        sum += _data[i] * _data[i];
    }
    return sum;
}

template <typename Scalar>
Scalar VectorX<Scalar>::mean() const {
    if (_size == 0) {
        return 0;
    }
    Scalar sum = 0;
    for (size_t i = 0; i < _size; i++) {
        sum += _data[i];
    }
    return sum / _size;
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::normalized() const {
    Scalar n = norm();
    if (n == 0) {
        return *this;
    }
    return *this / n;
}

template <typename Scalar>
void VectorX<Scalar>::normalize() {
    Scalar n = norm();
    if (n == 0) {
        return;
    }
    *this /= n;
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::mult(const VectorX& v) const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = _data[i] * v[i];
    }
    return std::move(result);
}

template <typename Scalar>
Scalar VectorX<Scalar>::dot(const VectorX& v) const {
    Scalar sum = 0;
    for (size_t i = 0; i < _size; i++) {
        sum += _data[i] * v[i];
    }
    return sum;
}

template <typename Scalar>
const VectorX<Scalar>& VectorX<Scalar>::zeros(size_t size) {
    static VectorX zeros(size, 0);
    return zeros;
}

template <typename Scalar>
const VectorX<Scalar>& VectorX<Scalar>::ones(size_t size) {
    static VectorX ones(size, 1);
    return ones;
}

template <typename Scalar>
    std::ostream& operator<<(std::ostream& os, const VectorX<Scalar>& v) {
    os << "[";
    for (size_t i = 0; i < v.size(); i++) {
        os << v[i];
        if (i < v.size() - 1) {
            os << " ";
        }
    }
    os << "]";
    return os;
}
