#include "phys/shape/box_shape.h"
#include "test_def.h"
#include "mesh_to_obj.h"

using namespace pe_phys_shape;

void testConstruct() {
    BoxShape box(pe::Vector3(1., 2., 3.));

    ASSERT_EQUAL_INT(box.getType(), ShapeType::BOX)
    ASSERT_EQUAL_INT(box.isConvex(), true)

    auto& mesh = box.getMesh();
    ASSERT_EQUAL_INT(mesh.vertices.size(), 24)
    ASSERT_EQUAL_INT(mesh.faces.size(), 6)
    meshToObj(mesh, SHAPE_TEST_SOURCE_DIR "/box_shape/box.obj");
}

void testAABB() {
    BoxShape box(pe::Vector3(1., 2., 3.));
    pe::Vector3 min, max;

    box.getAABB(pe::Transform::identity(), min, max);
    ASSERT_EQUAL(min.x, -0.5)
    ASSERT_EQUAL(min.y, -1.0)
    ASSERT_EQUAL(min.z, -1.5)
    ASSERT_EQUAL(max.x, 0.5)
    ASSERT_EQUAL(max.y, 1.0)
    ASSERT_EQUAL(max.z, 1.5)

    pe::Transform transform;
    transform.setTranslation(pe::Vector3(1., 2., 3.));
    box.getAABB(transform, min, max);
    ASSERT_EQUAL(min.x, 0.5)
    ASSERT_EQUAL(min.y, 1.0)
    ASSERT_EQUAL(min.z, 1.5)
    ASSERT_EQUAL(max.x, 1.5)
    ASSERT_EQUAL(max.y, 3.0)
    ASSERT_EQUAL(max.z, 4.5)

    transform.setRotation(pe::Vector3::up(), M_PI / 4);
    transform.setTranslation(pe::Vector3(0., 0., 0.));
    box.getAABB(transform, min, max);
    ASSERT_EQUAL(min.x, -sqrt(2.))
    ASSERT_EQUAL(min.y, -1)
    ASSERT_EQUAL(min.z, -sqrt(2.))
    ASSERT_EQUAL(max.x, sqrt(2.))
    ASSERT_EQUAL(max.y, 1)
    ASSERT_EQUAL(max.z, sqrt(2.))
}

void testIsInside() {
    BoxShape box(pe::Vector3(1., 2., 3.));

    ASSERT_EQUAL(box.isInside(pe::Transform::identity(),
                              pe::Vector3(0., 0., 0.)),
                 true)
    ASSERT_EQUAL(box.isInside(pe::Transform::identity(),
                              pe::Vector3(0.499, 0.999, 1.499)),
                 true)
    ASSERT_EQUAL(box.isInside(pe::Transform::identity(),
                              pe::Vector3(0.501, 1.001, 1.501)),
                 false)

    pe::Transform transform;
    transform.setRotation({0, 1, 0}, M_PI / 4);
    transform.setTranslation({2, 2, 2});
    ASSERT_EQUAL(box.isInside(transform,
                              pe::Vector3(2., 2., 2.)),
                 true)
    ASSERT_EQUAL(box.isInside(transform,
                              pe::Vector3(1.999 + sqrt(1.999), 2.999, 1.999 + sqrt(1.999) / 2)),
                 true)
    ASSERT_EQUAL(box.isInside(transform,
                              pe::Vector3(2.001 + sqrt(2.001), 3.001, 2.001 + sqrt(2.001) / 2)),
                 false)
}

void testProject() {
    BoxShape box(pe::Vector3(1., 2., 3.));
    pe::Real min, max;

    box.project(pe::Transform::identity(), pe::Vector3::up(), min, max);
    ASSERT_EQUAL(min, -1.)
    ASSERT_EQUAL(max, 1.)

    box.project(pe::Transform::identity(), pe::Vector3::right(), min, max);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)

    box.project(pe::Transform::identity(), pe::Vector3::forward(), min, max);
    ASSERT_EQUAL(min, -1.5)
    ASSERT_EQUAL(max, 1.5)

    pe::Transform transform;
    transform.setRotation({0, 1, 0}, M_PI / 4);
    transform.setTranslation({2, 2, 2});
    box.project(transform, pe::Vector3::up(), min, max);
    ASSERT_EQUAL(min, 1.)
    ASSERT_EQUAL(max, 3.)

    box.project(transform, pe::Vector3::right(), min, max);
    ASSERT_EQUAL(min, 2. - sqrt(2.))
    ASSERT_EQUAL(max, 2. + sqrt(2.))

    box.project(transform, pe::Vector3::forward(), min, max);
    ASSERT_EQUAL(min, 2. - sqrt(2.))
    ASSERT_EQUAL(max, 2. + sqrt(2.))
}

int main() {
    testConstruct();
    testAABB();
    testIsInside();
    testProject();
}
