template <typename Scalar, size_t M, size_t N>
ObjectPool<typename MatrixMN<Scalar, M, N>::MatrixMNData, M * N * sizeof(Scalar) * 256> MatrixMN<Scalar, M, N>::_pool;

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N>::MatrixMN(Scalar value): _m(_pool.create()) {
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            _m->data[i][j] = value;
        }
    }
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N>::MatrixMN(const MatrixMN& other): _m(_pool.create()) {
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            _m->data[i][j] = other[i][j];
        }
    }
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N>::MatrixMN(MatrixMN&& other) noexcept: _m(other._m) {
    other._m = nullptr;
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N>& MatrixMN<Scalar, M, N>::operator=(const MatrixMN& other) {
    if (this != &other) {
        for (size_t i = 0; i < M; i++) {
            for (size_t j = 0; j < N; j++) {
                _m->data[i][j] = other[i][j];
            }
        }
    }
    return *this;
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N>& MatrixMN<Scalar, M, N>::operator=(MatrixMN&& other) noexcept {
    if (this != &other) {
        for (size_t i = 0; i < M; i++) {
            for (size_t j = 0; j < N; j++) {
                _m->data[i][j] = other[i][j];
            }
        }
    }
    return *this;
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N> MatrixMN<Scalar, M, N>::operator-() const {
    MatrixMN res;
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            res[i][j] = -_m->data[i][j];
        }
    }
    return std::move(res);
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N> MatrixMN<Scalar, M, N>::operator+(const MatrixMN& other) const {
    MatrixMN res;
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            res[i][j] = _m->data[i][j] + other[i][j];
        }
    }
    return std::move(res);
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N> MatrixMN<Scalar, M, N>::operator-(const MatrixMN& other) const {
    MatrixMN res;
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            res[i][j] = _m->data[i][j] - other[i][j];
        }
    }
    return std::move(res);
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N> MatrixMN<Scalar, M, N>::operator*(Scalar s) const {
    MatrixMN res;
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            res[i][j] = _m->data[i][j] * s;
        }
    }
    return std::move(res);
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N> MatrixMN<Scalar, M, N>::operator/(Scalar s) const {
    MatrixMN res;
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            res[i][j] = _m->data[i][j] / s;
        }
    }
    return std::move(res);
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N>& MatrixMN<Scalar, M, N>::operator+=(const MatrixMN& other) {
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            _m->data[i][j] += other[i][j];
        }
    }
    return *this;
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N>& MatrixMN<Scalar, M, N>::operator-=(const MatrixMN& other) {
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            _m->data[i][j] -= other[i][j];
        }
    }
    return *this;
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N>& MatrixMN<Scalar, M, N>::operator*=(Scalar s) {
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            _m->data[i][j] *= s;
        }
    }
    return *this;
}

template <typename Scalar, size_t M, size_t N>
MatrixMN<Scalar, M, N>& MatrixMN<Scalar, M, N>::operator/=(Scalar s) {
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            _m->data[i][j] /= s;
        }
    }
    return *this;
}

template <typename Scalar, size_t M, size_t N>
template <size_t N_OTHER>
MatrixMN<Scalar, M, N_OTHER> MatrixMN<Scalar, M, N>::operator*(const MatrixMN<Scalar, N, N_OTHER>& other) const {
    MatrixMN<Scalar, M, N_OTHER> res(0);
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N_OTHER; j++) {
            for (size_t k = 0; k < N; k++) {
                res[i][j] += _m->data[i][k] * other[k][j];
            }
        }
    }
    return std::move(res);
}

template <typename Scalar, size_t M, size_t N>
VectorX<Scalar, M> MatrixMN<Scalar, M, N>::operator*(const VectorX<Scalar, N>& vec) const {
    VectorX<Scalar, M> res(0);
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < N; j++) {
            res[i] += _m->data[i][j] * vec[j];
        }
    }
    return std::move(res);
}

template <typename Scalar, size_t M, size_t N>
const MatrixMN<Scalar, M, N>& MatrixMN<Scalar, M, N>::identity() {
    static MatrixMN res(0);
    static bool initialized = false;
    if (!initialized) {
        for (size_t i = 0; i < M && i < N; i++) {
            res[i][i] = 1;
        }
        initialized = true;
    }
    return res;
}

template <typename Scalar, size_t M, size_t N>
const MatrixMN<Scalar, M, N>& MatrixMN<Scalar, M, N>::zeros() {
    static MatrixMN res(0);
    return res;
}

template <typename Scalar, size_t M, size_t N>
const MatrixMN<Scalar, M, N>& MatrixMN<Scalar, M, N>::ones() {
    static MatrixMN res(1);
    return res;
}

template <typename Scalar, size_t M, size_t N>
    std::ostream& operator<<(std::ostream& os, const MatrixMN<Scalar, M, N>& mat) {
    os << "[";
    for (size_t i = 0; i < M; i++) {
        if (i != 0) {
            os << " ";
        }
        for (size_t j = 0; j < N; j++) {
            os << mat[i][j];
            if (j < N - 1) {
                os << "\t";
            }
        }
        if (i < M - 1) {
            os << "\n";
        }
    }
    os << "]";
    return os;
}
