#include <cassert>
#include "common/transform.h"
#include "eigen_std.h"
#include "test_def.h"

using namespace pe_common;

void testConstruct() {
    PEReal n[] = {randR(), randR(), randR(),
                  randR(), randR(), randR(),
                  randR(), randR(), randR(),
                  randR(), randR(), randR()};
    Transform a,
              b({n[0], n[1], n[2],
                 n[3], n[4], n[5],
                 n[6], n[7], n[8]},
                {n[9], n[10], n[11]});
    Matrix3x3Std m;
    m << n[0], n[1], n[2], n[3], n[4], n[5], n[6], n[7], n[8];

    ASSERT_MATRIX3_EIGEN_EQUAL(a.getBasis(), Matrix3x3Std::Identity())
    ASSERT_VECTOR3_EIGEN_EQUAL(a.getOrigin(), Vector3Std::Zero())
    ASSERT_MATRIX3_EIGEN_EQUAL(b.getBasis(), m)
    ASSERT_VECTOR3_EIGEN_EQUAL(b.getOrigin(), Vector3Std(n[9], n[10], n[11]))

    ASSERT_VECTOR3_EIGEN_EQUAL(b.getAxis(0), Vector3Std(n[0], n[3], n[6]))
    ASSERT_VECTOR3_EIGEN_EQUAL(b.getAxis(1), Vector3Std(n[1], n[4], n[7]))
    ASSERT_VECTOR3_EIGEN_EQUAL(b.getAxis(2), Vector3Std(n[2], n[5], n[8]))

    ASSERT_MATRIX3_EIGEN_EQUAL(Transform::identity().getBasis(), Matrix3x3Std::Identity())
    ASSERT_VECTOR3_EIGEN_EQUAL(Transform::identity().getOrigin(), Vector3Std::Zero())
}

void testOperator() {
    PEReal n[] = {randR(), randR(), randR(),
                  randR(), randR(), randR(),
                  randR(), randR(), randR(),
                  randR(), randR(), randR(),
                  randR(), randR(), randR(),
                  randR(), randR(), randR(),
                  randR(), randR(), randR(),
                  randR(), randR(), randR(),
                  randR(), randR(), randR()};
    Transform a,
              b({n[0], n[1], n[2],
                 n[3], n[4], n[5],
                 n[6], n[7], n[8]},
                {n[9], n[10], n[11]}),
              c({n[12], n[13], n[14],
                 n[15], n[16], n[17],
                 n[18], n[19], n[20]},
                {n[21], n[22], n[23]});
    Vector3 v(n[24], n[25], n[26]);

    ASSERT_VECTOR3_EQUAL(a * v, v)
    ASSERT_VECTOR3_EQUAL(c * v, c.getBasis() * v + c.getOrigin())

    ASSERT_TRANSFORM_EQUAL(a * b, b.getBasis(), b.getOrigin())
    ASSERT_TRANSFORM_EQUAL(a *= b, b.getBasis(), b.getOrigin())
    auto m_basis = c.getBasis() * b.getBasis();
    auto m_origin = c.getBasis() * b.getOrigin() + c.getOrigin();
    ASSERT_TRANSFORM_EQUAL(c * b, m_basis, m_origin)
    ASSERT_TRANSFORM_EQUAL(c *= b, m_basis, m_origin)
}

void testGeometry() {
    PEReal n[] = {randNeg(), randR(), randR(), randR(),
                  randR(), randR(), randR()};
    Transform a;
    a.setTranslation({n[4], n[5], n[6]});
    ASSERT_VECTOR3_EQUAL(a.getOrigin(), Vector3(n[4], n[5], n[6]))

    a.setRotation({n[0], n[1], n[2]}, n[3]);
    auto rot = AngleAxisStd(n[3], Eigen::Vector3d(n[0], n[1], n[2]).normalized());
    ASSERT_MATRIX3_EIGEN_EQUAL(a.getBasis(), rot.toRotationMatrix())

    auto rot_x = AngleAxisStd(n[0], Eigen::Vector3d::UnitX()).toRotationMatrix();
    auto rot_y = AngleAxisStd(n[1], Eigen::Vector3d::UnitY()).toRotationMatrix();
    auto rot_z = AngleAxisStd(n[2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
    a.setEulerRotation(n[0], n[1], n[2], RotType::S_XYZ);
    ASSERT_MATRIX3_EIGEN_EQUAL(a.getBasis(), rot_z * rot_y * rot_x)
    a.setEulerRotation(n[0], n[1], n[2], RotType::S_YZX);
    ASSERT_MATRIX3_EIGEN_EQUAL(a.getBasis(), rot_x * rot_z * rot_y)
    a.setEulerRotation(n[0], n[1], n[2], RotType::S_ZXY);
    ASSERT_MATRIX3_EIGEN_EQUAL(a.getBasis(), rot_y * rot_x * rot_z)
    a.setEulerRotation(n[0], n[1], n[2], RotType::S_XZY);
    ASSERT_MATRIX3_EIGEN_EQUAL(a.getBasis(), rot_y * rot_z * rot_x)
    a.setEulerRotation(n[0], n[1], n[2], RotType::S_ZYX);
    ASSERT_MATRIX3_EIGEN_EQUAL(a.getBasis(), rot_x * rot_y * rot_z)
    a.setEulerRotation(n[0], n[1], n[2], RotType::S_YXZ);
    ASSERT_MATRIX3_EIGEN_EQUAL(a.getBasis(), rot_z * rot_x * rot_y)
}

void testMath() {
    PEReal n[] = {randPos(), randR(), randR(), randR(),
                  randR(), randR(), randR(),
                  randR(), randR(), randR()};
    Transform a;
    a.setRotation({n[0], n[1], n[2]}, n[3]);
    a.setTranslation({n[4], n[5], n[6]});
    auto m_basis = AngleAxisStd(n[3], Eigen::Vector3d(n[0], n[1], n[2]).normalized()).toRotationMatrix();
    auto m_origin = Vector3Std(n[4], n[5], n[6]);
    Vector3 v(n[7], n[8], n[9]);
    Vector3Std ve(n[7], n[8], n[9]);

    ASSERT_VECTOR3_EIGEN_EQUAL(a.inverseTransform(v),
                               m_basis.transpose() * (ve - m_origin))
    ASSERT_TRANSFORM_EIGEN_EQUAL(a.inverse(), m_basis.transpose(), -m_basis.transpose() * m_origin)
    a.invert();
    ASSERT_TRANSFORM_EIGEN_EQUAL(a, m_basis.transpose(), -m_basis.transpose() * m_origin)
}

int main() {
    randInit();
    for (int i = 0; i < 10; i++) {
        testConstruct();
        testOperator();
        testGeometry();
        testMath();
    }
    return 0;
}
