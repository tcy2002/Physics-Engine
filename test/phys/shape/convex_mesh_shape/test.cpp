#include "phys/shape/box_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "test_def.h"
#include "mesh_to_obj.h"

using namespace pe_phys_shape;

void testConstruct() {
    BoxShape box(pe_common::Vector3(1., 2., 3.));
    ConvexMeshShape meshShape(box.getMesh());

    ASSERT_EQUAL_INT(meshShape.getType(), ShapeType::MESH)
    ASSERT_EQUAL_INT(meshShape.isConvex(), true)

    auto& mesh = meshShape.getMesh();
    ASSERT_EQUAL_INT(mesh.vertices.size(), 24)
    ASSERT_EQUAL_INT(mesh.faces.size(), 6)
    meshToObj(mesh, SHAPE_TEST_SOURCE_DIR "/convex_mesh_shape/mesh.obj");
}

void testAABB() {
    BoxShape box(pe_common::Vector3(1., 2., 3.));
    ConvexMeshShape meshShape(box.getMesh());
    pe_common::Vector3 min, max;

    meshShape.getAABB(pe_common::Transform::identity(), min, max);
    ASSERT_EQUAL(min.x, -0.5)
    ASSERT_EQUAL(min.y, -1.0)
    ASSERT_EQUAL(min.z, -1.5)
    ASSERT_EQUAL(max.x, 0.5)
    ASSERT_EQUAL(max.y, 1.0)
    ASSERT_EQUAL(max.z, 1.5)

    pe_common::Transform transform;
    transform.setTranslation(pe_common::Vector3(1., 2., 3.));
    meshShape.getAABB(transform, min, max);
    ASSERT_EQUAL(min.x, 0.5)
    ASSERT_EQUAL(min.y, 1.0)
    ASSERT_EQUAL(min.z, 1.5)
    ASSERT_EQUAL(max.x, 1.5)
    ASSERT_EQUAL(max.y, 3.0)
    ASSERT_EQUAL(max.z, 4.5)

    transform.setRotation(pe_common::Vector3::up(), M_PI / 4);
    transform.setTranslation(pe_common::Vector3(0., 0., 0.));
    meshShape.getAABB(transform, min, max);
    ASSERT_EQUAL(min.x, -sqrt(2.))
    ASSERT_EQUAL(min.y, -1)
    ASSERT_EQUAL(min.z, -sqrt(2.))
    ASSERT_EQUAL(max.x, sqrt(2.))
    ASSERT_EQUAL(max.y, 1)
    ASSERT_EQUAL(max.z, sqrt(2.))
}

void testIsInside() {
    BoxShape box(pe_common::Vector3(1., 2., 3.));
    ConvexMeshShape meshShape(box.getMesh());

    ASSERT_EQUAL(meshShape.isInside(pe_common::Transform::identity(),
                                    pe_common::Vector3(0., 0., 0.)),
                 true)
    ASSERT_EQUAL(meshShape.isInside(pe_common::Transform::identity(),
                                    pe_common::Vector3(0.499, 0.999, 1.499)),
                 true)
    ASSERT_EQUAL(meshShape.isInside(pe_common::Transform::identity(),
                                    pe_common::Vector3(0.501, 1.001, 1.501)),
                 false)

    pe_common::Transform transform;
    transform.setRotation({0, 1, 0}, M_PI / 4);
    transform.setTranslation({2, 2, 2});
    ASSERT_EQUAL(meshShape.isInside(transform,
                                    pe_common::Vector3(2., 2., 2.)),
                 true)
    ASSERT_EQUAL(meshShape.isInside(transform,
                                    pe_common::Vector3(1.999 + sqrt(1.999), 2.999, 1.999 + sqrt(1.999) / 2)),
                 true)
    ASSERT_EQUAL(meshShape.isInside(transform,
                                    pe_common::Vector3(2.001 + sqrt(2.001), 3.001, 2.001 + sqrt(2.001) / 2)),
                 false)
}

void testProject() {
    BoxShape box(pe_common::Vector3(1., 2., 3.));
    ConvexMeshShape meshShape(box.getMesh());
    PEReal min, max;

    meshShape.project(pe_common::Transform::identity(), pe_common::Vector3::up(), min, max);
    ASSERT_EQUAL(min, -1.)
    ASSERT_EQUAL(max, 1.)

    meshShape.project(pe_common::Transform::identity(), pe_common::Vector3::right(), min, max);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)

    meshShape.project(pe_common::Transform::identity(), pe_common::Vector3::forward(), min, max);
    ASSERT_EQUAL(min, -1.5)
    ASSERT_EQUAL(max, 1.5)

    pe_common::Transform transform;
    transform.setRotation({0, 1, 0}, M_PI / 4);
    transform.setTranslation({2, 2, 2});
    meshShape.project(transform, pe_common::Vector3::up(), min, max);
    ASSERT_EQUAL(min, 1.)
    ASSERT_EQUAL(max, 3.)

    meshShape.project(transform, pe_common::Vector3::right(), min, max);
    ASSERT_EQUAL(min, 2. - sqrt(2.))
    ASSERT_EQUAL(max, 2. + sqrt(2.))

    meshShape.project(transform, pe_common::Vector3::forward(), min, max);
    ASSERT_EQUAL(min, 2. - sqrt(2.))
    ASSERT_EQUAL(max, 2. + sqrt(2.))
}

int main() {
    testConstruct();
    testAABB();
    testIsInside();
    testProject();
}
