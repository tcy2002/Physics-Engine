#include <cassert>
#include "common/vector3.h"
#include "../eigen_std.h"
#include "test_general.h"

void testConstruct() {
    Real n[] = {randR(), randR(), randR()};
    Vector3Test a, b(n[0], n[1], n[2]);
    Vector3Std ae(0, 0, 0), be(n[0], n[1], n[2]);

    ASSERT_VECTOR3_EIGEN_EQUAL(a, ae);
    ASSERT_VECTOR3_EIGEN_EQUAL(b, be);

    ASSERT_VECTOR3_EIGEN_EQUAL(Vector3Test::zeros(), Vector3Std::Zero());
    ASSERT_VECTOR3_EIGEN_EQUAL(Vector3Test::ones(), Vector3Std::Ones());
    ASSERT_VECTOR3_EIGEN_EQUAL(Vector3Test::forward(), Vector3Std::UnitZ());
    ASSERT_VECTOR3_EIGEN_EQUAL(Vector3Test::up(), Vector3Std::UnitY());
    ASSERT_VECTOR3_EIGEN_EQUAL(Vector3Test::right(), Vector3Std::UnitX());
}

void testOperator() {
    Real n[] = {randPos(), randNeg(), randR(), randR(), randR(), randR()};
    Vector3Test a, b(n[0], n[1], n[2]), c(n[3], n[4], n[5]);
    Vector3Std ae(0, 0 ,0), be(n[0], n[1], n[2]), ce(n[3], n[4], n[5]);

    ASSERT_VECTOR3_EIGEN_EQUAL(a + b, ae + be);
    ASSERT_VECTOR3_EIGEN_EQUAL(a - b, ae - be);
    ASSERT_VECTOR3_EIGEN_EQUAL(c + b, ce + be);
    ASSERT_VECTOR3_EIGEN_EQUAL(c - b, ce - be);

    ASSERT_VECTOR3_EIGEN_EQUAL(n[0] * a, n[0] * ae);
    ASSERT_VECTOR3_EIGEN_EQUAL(n[0] * b, n[0] * be);
    ASSERT_VECTOR3_EIGEN_EQUAL(a * n[0], ae * n[0]);
    ASSERT_VECTOR3_EIGEN_EQUAL(b * n[0], be * n[0]);
    ASSERT_VECTOR3_EIGEN_EQUAL(a * n[1], ae * n[1]);
    ASSERT_VECTOR3_EIGEN_EQUAL(b * n[1], be * n[1]);

    ASSERT_VECTOR3_EIGEN_EQUAL(a / n[0], ae / n[0]);
    ASSERT_VECTOR3_EIGEN_EQUAL(b / n[0], be / n[0]);
    ASSERT_VECTOR3_EIGEN_EQUAL(a / n[1], ae / n[1]);
    ASSERT_VECTOR3_EIGEN_EQUAL(b / n[1], be / n[1]);

    ASSERT_VECTOR3_EIGEN_EQUAL(a += b, ae += be);
    ASSERT_VECTOR3_EIGEN_EQUAL(a -= b, ae -= be);
    ASSERT_VECTOR3_EIGEN_EQUAL(c += b, ce += be);
    ASSERT_VECTOR3_EIGEN_EQUAL(c -= b, ce -= be);

    ASSERT_VECTOR3_EIGEN_EQUAL(a *= n[0], ae *= n[0]);
    ASSERT_VECTOR3_EIGEN_EQUAL(b *= n[0], be *= n[0]);
    ASSERT_VECTOR3_EIGEN_EQUAL(a *= n[1], ae *= n[1]);
    ASSERT_VECTOR3_EIGEN_EQUAL(b *= n[1], be *= n[1]);

    ASSERT_VECTOR3_EIGEN_EQUAL(a /= n[0], ae /= n[0]);
    ASSERT_VECTOR3_EIGEN_EQUAL(b /= n[0], be /= n[0]);
    ASSERT_VECTOR3_EIGEN_EQUAL(a /= n[1], ae /= n[1]);
    ASSERT_VECTOR3_EIGEN_EQUAL(b /= n[1], be /= n[1]);

    ASSERT_VECTOR3_EIGEN_EQUAL(-a, -ae);
    ASSERT_VECTOR3_EIGEN_EQUAL(-b, -be);
}

void testMath() {
    Real n[] = {randPos(), randNeg(), randR(), randR(), randR(), randR()};
    Vector3Test a, b(n[0], n[1], n[2]), c(n[3], n[4], n[5]);
    Vector3Std ae(0, 0 ,0), be(n[0], n[1], n[2]), ce(n[3], n[4], n[5]);

    ASSERT_EQUAL(a.norm(), ae.norm())
    ASSERT_EQUAL(b.norm(), be.norm())

    ASSERT_VECTOR3_EQUAL(a.mult(b), Vector3Test(0, 0, 0));
    ASSERT_VECTOR3_EQUAL(b.mult(c), Vector3Test(n[0] * n[3], n[1] * n[4], n[2] * n[5]));

    ASSERT_VECTOR3_EIGEN_EQUAL(b.normalized(), be.normalized());
    c.normalize(), ce.normalize();
    ASSERT_VECTOR3_EIGEN_EQUAL(c, ce);

    ASSERT_EQUAL(a.dot(b), ae.dot(be));
    ASSERT_EQUAL(c.dot(b), ce.dot(be));
    ASSERT_VECTOR3_EIGEN_EQUAL(a.cross(b), ae.cross(be));
    ASSERT_VECTOR3_EIGEN_EQUAL(c.cross(b), ce.cross(be));
}

void testGeometry() {
    Vector3Test a, b(3, 5, 4), c(0, 1, 0);
    Vector3Std ae(0, 0, 0);

    Vector3Test a_p_c = a.project(c);
    ASSERT_EQUAL(a_p_c.x, 0);
    ASSERT_EQUAL(a_p_c.y, 0);
    ASSERT_EQUAL(a_p_c.z, 0);

    Vector3Test b_p_c = b.project(c);
    ASSERT_EQUAL(b_p_c.x, 0);
    ASSERT_EQUAL(b_p_c.y, 5);
    ASSERT_EQUAL(b_p_c.z, 0);

    Vector3Test a_r_c = a.reflect(c);
    ASSERT_EQUAL(a_r_c.x, 0);
    ASSERT_EQUAL(a_r_c.y, 0);
    ASSERT_EQUAL(a_r_c.z, 0);

    Vector3Test b_r_c = b.reflect(c);
    ASSERT_EQUAL(b_r_c.x, -3);
    ASSERT_EQUAL(b_r_c.y, 5);
    ASSERT_EQUAL(b_r_c.z, -4);

    Vector3Test a_ro_c = a.rotate(c, M_PI / 2);
    ASSERT_EQUAL(a_ro_c.x, 0);
    ASSERT_EQUAL(a_ro_c.y, 0);
    ASSERT_EQUAL(a_ro_c.z, 0);

    Vector3Test b_ro_c = b.rotate(c, M_PI / 2);
    ASSERT_EQUAL(b_ro_c.x, 4);
    ASSERT_EQUAL(b_ro_c.y, 5);
    ASSERT_EQUAL(b_ro_c.z, -3);

    Real angle = b.angle(b);
    ASSERT_EQUAL(angle, 0);
    angle = b.angle(c);
    ASSERT_EQUAL(angle, M_PI / 4);
}

int main() {
    randInit();
    for (int i = 0; i < 10; i++) {
        testConstruct();
        testOperator();
        testMath();
    }
    testGeometry();

    return 0;
}
