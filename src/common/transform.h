#pragma once

#include "vector3.h"
#include "matrix3x3.h"

namespace common {

    template<typename Scalar>
    class Transform {
    protected:
        COMMON_MEMBER_SET_GET(Matrix3x3<Scalar>, basis, Basis)
        COMMON_MEMBER_SET_GET(Vector3<Scalar>, origin, Origin)

    public:
        Transform(): _basis(Matrix3x3<Scalar>::identity()), _origin(Vector3<Scalar>::zeros()) {}
        Transform(const Matrix3x3<Scalar>& basis, const Vector3<Scalar>& origin): _basis(basis), _origin(origin) {}

        enum RotType {
            S_XYZ, S_YZX, S_ZXY, S_XZY, S_ZYX, S_YXZ,
            XYZ,   YZX,   ZXY,   XZY,   ZYX,   YXZ
        };

        COMMON_FORCE_INLINE Vector3<Scalar> operator*(const Vector3<Scalar>&) const;
        COMMON_FORCE_INLINE Transform operator*(const Transform&) const;
        COMMON_FORCE_INLINE Transform& operator*=(const Transform&);

        COMMON_FORCE_INLINE Vector3<Scalar> getAxis(int axis) const;
        COMMON_FORCE_INLINE void setRotation(const Vector3<Scalar>& axis, Scalar angle);
        COMMON_FORCE_INLINE void setEulerRotation(Scalar x, Scalar y, Scalar z, RotType type = RotType::S_YZX);
        COMMON_FORCE_INLINE void setTranslation(const Vector3<Scalar>& translation);
        COMMON_FORCE_INLINE void invert();
        COMMON_FORCE_INLINE Transform inverse() const;
        COMMON_FORCE_INLINE Vector3<Scalar> inverseTransform(const Vector3<Scalar>& v) const;

        COMMON_FORCE_INLINE static Transform const& identity();

    };

    template<typename Scalar>
    COMMON_FORCE_INLINE std::ostream &operator<<(std::ostream& os, const Transform<Scalar>& t);

    #include "transform.cpp"

} // namespace common