#include <cassert>
#include "common/matrix3x3.h"
#include "eigen_std.h"
#include "test_general.h"

void testConstruct() {
    Real n[] = {randR(), randR(), randR(),
                randR(), randR(), randR(),
                randR(), randR(), randR()};
    Matrix3Test a,
                b(n[0], n[1], n[2],
                  n[3], n[4], n[5],
                  n[6], n[7], n[8]);
    Matrix3x3Std ae, be;
    ae.setZero();
    be << n[0], n[1], n[2], n[3], n[4], n[5], n[6], n[7], n[8];

    ASSERT_MATRIX3_EIGEN_EQUAL(a, ae);
    ASSERT_MATRIX3_EIGEN_EQUAL(b, be);

    ASSERT_MATRIX3_EIGEN_EQUAL(Matrix3Test::identity(), Matrix3x3Std::Identity());
    ASSERT_MATRIX3_EIGEN_EQUAL(Matrix3Test::zeros(), Matrix3x3Std::Zero());
    ASSERT_MATRIX3_EIGEN_EQUAL(Matrix3Test::ones(), Matrix3x3Std::Ones());
}

void testOperator() {
    Real n[] = {randPos(), randNeg(), randR(),
                randR(), randR(), randR(),
                randR(), randR(), randR(),
                randR(), randR(), randR(),
                randR(), randR(), randR(),
                randR(), randR(), randR(),
                randR(), randR(), randR()};
    Matrix3Test a,
                b(n[0], n[1], n[2],
                  n[3], n[4], n[5],
                  n[6], n[7], n[8]),
                c(n[9], n[10], n[11],
                  n[12], n[13], n[14],
                  n[15], n[16], n[17]);
    Matrix3x3Std ae, be, ce;
    ae.setZero();
    be << n[0], n[1], n[2], n[3], n[4], n[5], n[6], n[7], n[8];
    ce << n[9], n[10], n[11], n[12], n[13], n[14], n[15], n[16], n[17];
    Vector3Test v(n[18], n[19], n[20]);
    Vector3Std ve(n[18], n[19], n[20]);

    ASSERT_MATRIX3_EIGEN_EQUAL(a + b, ae + be);
    ASSERT_MATRIX3_EIGEN_EQUAL(a - b, ae - be);
    ASSERT_MATRIX3_EIGEN_EQUAL(c + b, ce + be);
    ASSERT_MATRIX3_EIGEN_EQUAL(c - b, ce - be);

    ASSERT_MATRIX3_EIGEN_EQUAL(n[0] * a, n[0] * ae);
    ASSERT_MATRIX3_EIGEN_EQUAL(n[0] * b, n[0] * be);
    ASSERT_MATRIX3_EIGEN_EQUAL(a * n[0], ae * n[0]);
    ASSERT_MATRIX3_EIGEN_EQUAL(b * n[0], be * n[0]);
    ASSERT_MATRIX3_EIGEN_EQUAL(a * n[1], ae * n[1]);
    ASSERT_MATRIX3_EIGEN_EQUAL(b * n[1], be * n[1]);

    ASSERT_MATRIX3_EIGEN_EQUAL(a / n[0], ae / n[0]);
    ASSERT_MATRIX3_EIGEN_EQUAL(b / n[0], be / n[0]);
    ASSERT_MATRIX3_EIGEN_EQUAL(a / n[1], ae / n[1]);
    ASSERT_MATRIX3_EIGEN_EQUAL(b / n[1], be / n[1]);

    ASSERT_MATRIX3_EIGEN_EQUAL(a += b, ae += be);
    ASSERT_MATRIX3_EIGEN_EQUAL(a -= b, ae -= be);
    ASSERT_MATRIX3_EIGEN_EQUAL(c += b, ce += be);
    ASSERT_MATRIX3_EIGEN_EQUAL(c -= b, ce -= be);

    ASSERT_MATRIX3_EIGEN_EQUAL(a *= n[0], ae *= n[0]);
    ASSERT_MATRIX3_EIGEN_EQUAL(b *= n[0], be *= n[0]);
    ASSERT_MATRIX3_EIGEN_EQUAL(a *= n[1], ae *= n[1]);
    ASSERT_MATRIX3_EIGEN_EQUAL(b *= n[1], be *= n[1]);

    ASSERT_MATRIX3_EIGEN_EQUAL(a /= n[0], ae /= n[0]);
    ASSERT_MATRIX3_EIGEN_EQUAL(b /= n[0], be /= n[0]);
    ASSERT_MATRIX3_EIGEN_EQUAL(a /= n[1], ae /= n[1]);
    ASSERT_MATRIX3_EIGEN_EQUAL(b /= n[1], be /= n[1]);

    ASSERT_MATRIX3_EIGEN_EQUAL(-a, -ae);
    ASSERT_MATRIX3_EIGEN_EQUAL(-b, -be);

    ASSERT_MATRIX3_EIGEN_EQUAL(a * b, ae * be);
    ASSERT_MATRIX3_EIGEN_EQUAL(c * b, ce * be);
    ASSERT_MATRIX3_EIGEN_EQUAL(a *= b, ae *= be);
    ASSERT_MATRIX3_EIGEN_EQUAL(c *= b, ce *= be);

    ASSERT_VECTOR3_EIGEN_EQUAL(a * v, ae * ve);
    ASSERT_VECTOR3_EIGEN_EQUAL(b * v, be * ve);
    ASSERT_VECTOR3_EIGEN_EQUAL(v * a, ve.transpose() * ae);
    ASSERT_VECTOR3_EIGEN_EQUAL(v * b, ve.transpose() * be);
}

void testMathGeometry() {
    Real n[] = {randPos(), randNeg(), randR(),
                randR(), randR(), randR(),
                randR(), randR(), randR()};
    Matrix3Test a,
                b(n[0], n[1], n[2],
                  n[3], n[4], n[5],
                  n[6], n[7], n[8]);
    Matrix3x3Std ae, be;
    ae.setZero();
    be << n[0], n[1], n[2], n[3], n[4], n[5], n[6], n[7], n[8];

    ASSERT_MATRIX3_EQUAL(a.mult(b), Matrix3Test::zeros());
    Matrix3Test b_mult_b = Matrix3Test(n[0] * n[0], n[1] * n[1], n[2] * n[2],
                                       n[3] * n[3], n[4] * n[4], n[5] * n[5],
                                       n[6] * n[6], n[7] * n[7], n[8] * n[8]);
    ASSERT_MATRIX3_EQUAL(b.mult(b), b_mult_b);

    ASSERT_EQUAL(a.trace(), ae.trace())
    ASSERT_EQUAL(b.trace(), be.trace())

    ASSERT_MATRIX3_EIGEN_EQUAL(b.transposed(), be.transpose());
    b.transpose();
    be << be.transpose().eval();
    ASSERT_MATRIX3_EIGEN_EQUAL(b, be);

    ASSERT_EQUAL(a.determinant(), ae.determinant())
    ASSERT_EQUAL(b.determinant(), be.determinant())

    ASSERT_MATRIX3_EIGEN_EQUAL(b.inverse(), be.inverse());
    b.invert();
    be << be.inverse().eval();
    ASSERT_MATRIX3_EIGEN_EQUAL(b, be);

    a.setRotation({n[0], n[1], n[2]}, n[3]);
    ae = AngleAxisStd(n[3], Eigen::Vector3d(n[0], n[1], n[2]).normalized()).toRotationMatrix();
    ASSERT_MATRIX3_EIGEN_EQUAL(a, ae);
}

int main() {
    randInit();
    for (int i = 0; i < 10; i++) {
        testConstruct();
        testOperator();
        testMathGeometry();
    }
    return 0;
}
