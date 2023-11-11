#pragma once

#include "vector3.h"
#include "matrix3x3.h"

namespace pe_cg {

enum RotType {
    S_XYZ, S_YZX, S_ZXY, S_XZY, S_ZYX, S_YXZ,
      XYZ,   YZX,   ZXY,   XZY,   ZYX,   YXZ
};

class Transform {
protected:
    Matrix3x3 _basis;
    Vector3 _origin;

public:
    Transform(): _basis(Matrix3x3::identity()), _origin(Vector3::zeros()) {}
    Transform(const Matrix3x3& basis, const Vector3& origin): _basis(basis), _origin(origin) {}

    PHYS_FORCE_INLINE Vector3 operator*(const Vector3&) const;
    PHYS_FORCE_INLINE Transform operator*(const Transform&) const;
    PHYS_FORCE_INLINE Transform& operator*=(const Transform&);

    PHYS_FORCE_INLINE const Matrix3x3& getBasis() const;
    PHYS_FORCE_INLINE const Vector3& getOrigin() const;
    PHYS_FORCE_INLINE Vector3 getAxis(int axis) const;
    PHYS_FORCE_INLINE void setRotation(const Vector3& axis, real angle);
    PHYS_FORCE_INLINE void setEulerRotation(real x, real y, real z, RotType type = RotType::S_YZX);
    PHYS_FORCE_INLINE void setTranslation(const Vector3& translation);
    PHYS_FORCE_INLINE void invert();
    PHYS_FORCE_INLINE Transform inverse() const;
    PHYS_FORCE_INLINE Vector3 inverseTransform(const Vector3& v) const;

    PHYS_FORCE_INLINE static Transform const& identity();

    PHYS_FORCE_INLINE friend std::ostream &operator<<(std::ostream& os, const Transform& t);
};

#include "transform.inl"

} // namespace data_cg