#include "phys/shape/cylinder_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "test_general.h"
#include "phys/shape/default_mesh.h"
#include <sstream>

using namespace pe_phys_shape;

void testConstruct() {
    CylinderShape cyl(0.5, 1.0);

    ASSERT_EQUAL_INT(cyl.getType(), ShapeType::ST_Cylinder)
    ASSERT_EQUAL_INT(cyl.isConvex(), true)
    ASSERT_EQUAL(cyl.getRadius(), 0.5);
    ASSERT_EQUAL(cyl.getHeight(), 1.0);
}

void testAABB() {
    CylinderShape cyl(0.5, 1.0);
    pe::Vector3 min, max;

    cyl.getAABB(pe::Transform::Identity(), min, max);
    ASSERT_EQUAL(min.x(), -0.5)
    ASSERT_EQUAL(min.y(), -0.5)
    ASSERT_EQUAL(min.z(), -0.5)
    ASSERT_EQUAL(max.x(), 0.5)
    ASSERT_EQUAL(max.y(), 0.5)
    ASSERT_EQUAL(max.z(), 0.5)

    pe::Transform transform;
    transform.setTranslation(pe::Vector3(1., 2., 3.));
    cyl.getAABB(transform, min, max);
    ASSERT_EQUAL(min.x(), 0.5)
    ASSERT_EQUAL(min.y(), 1.5)
    ASSERT_EQUAL(min.z(), 2.5)
    ASSERT_EQUAL(max.x(), 1.5)
    ASSERT_EQUAL(max.y(), 2.5)
    ASSERT_EQUAL(max.z(), 3.5)

    transform.setRotation(pe::Vector3::UnitZ(), PE_PI / 6);
    cyl.getAABB(transform, min, max);
    ASSERT_EQUAL(min.x(), 0.317)
    ASSERT_EQUAL(min.y(), 1.317)
    ASSERT_EQUAL(min.z(), 2.5)
    ASSERT_EQUAL(max.x(), 1.683)
    ASSERT_EQUAL(max.y(), 2.683)
    ASSERT_EQUAL(max.z(), 3.5)
}

void testIsInside() {
    CylinderShape cyl(0.5, 1.0);

    ASSERT_EQUAL(cyl.localIsInside(pe::Vector3(0., 0., 0.)), true)
    ASSERT_EQUAL(cyl.localIsInside(pe::Vector3(0.353, 0.499, 0.353)), true)
    ASSERT_EQUAL(cyl.localIsInside(pe::Vector3(0.354, 0.501, 0.354)), false)
}

void testProject() {
    CylinderShape cyl(0.5, 1.0);
    pe::Real min, max;
    pe::Vector3 minPoint, maxPoint;

    cyl.project(pe::Transform::Identity(), pe::Vector3::UnitY(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_EQUAL(minPoint.y(), -0.5)
    ASSERT_EQUAL(maxPoint.y(), 0.5)

    cyl.project(pe::Transform::Identity(), pe::Vector3::UnitX(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_EQUAL(minPoint.x(), -0.5)
    ASSERT_EQUAL(maxPoint.x(), 0.5)

    cyl.project(pe::Transform::Identity(), pe::Vector3::UnitZ(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_EQUAL(minPoint.z(), -0.5)
    ASSERT_EQUAL(maxPoint.z(), 0.5)

    pe::Transform transform;
    transform.setRotation({0, 0, 1}, PE_PI / 6);
    transform.setTranslation({2, 2, 2});
    cyl.project(transform, pe::Vector3::UnitY(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.317)
    ASSERT_EQUAL(max, 2.683)
    ASSERT_VECTOR3_EQUAL(minPoint, pe::Vector3(1.817, 1.317, 2));
    ASSERT_VECTOR3_EQUAL(maxPoint, pe::Vector3(2.183, 2.683, 2));

    cyl.project(transform, pe::Vector3::UnitX(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.317)
    ASSERT_EQUAL(max, 2.683)
    ASSERT_VECTOR3_EQUAL(minPoint, pe::Vector3(1.317, 2.183, 2));
    ASSERT_VECTOR3_EQUAL(maxPoint, pe::Vector3(2.683, 1.817, 2));

    cyl.project(transform, pe::Vector3::UnitZ(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.5)
    ASSERT_EQUAL(max, 2.5)
    ASSERT_EQUAL(minPoint.z(), 1.5)
    ASSERT_EQUAL(maxPoint.z(), 2.5)
}

int main() {
    testConstruct();
    testAABB();
    testIsInside();
    testProject();
}
