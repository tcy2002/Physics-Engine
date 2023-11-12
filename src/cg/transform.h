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

    PE_FORCE_INLINE Vector3 operator*(const Vector3&) const;
    PE_FORCE_INLINE Transform operator*(const Transform&) const;
    PE_FORCE_INLINE Transform& operator*=(const Transform&);

    PE_FORCE_INLINE const Matrix3x3& getBasis() const;
    PE_FORCE_INLINE const Vector3& getOrigin() const;
    PE_FORCE_INLINE Vector3 getAxis(int axis) const;
    PE_FORCE_INLINE void setRotation(const Vector3& axis, real angle);
    PE_FORCE_INLINE void setEulerRotation(real x, real y, real z, RotType type = RotType::S_YZX);
    PE_FORCE_INLINE void setTranslation(const Vector3& translation);
    PE_FORCE_INLINE void invert();
    PE_FORCE_INLINE Transform inverse() const;
    PE_FORCE_INLINE Vector3 inverseTransform(const Vector3& v) const;

    PE_FORCE_INLINE static Transform const& identity();

    PE_FORCE_INLINE friend std::ostream &operator<<(std::ostream& os, const Transform& t);
};

#include "transform.inl"

} // namespace pe_cg