#include "phys/shape/cylinder_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "test_general.h"
#include "phys/shape/default_mesh.h"
#include <sstream>

using namespace pe_phys_shape;

void testConstruct() {
    CylinderShape cyl(0.5, 1.0);

    ASSERT_EQUAL_INT(cyl.getType(), ShapeType::Cylinder)
    ASSERT_EQUAL_INT(cyl.isConvex(), true)
    ASSERT_EQUAL(cyl.getRadius(), 0.5);
    ASSERT_EQUAL(cyl.getHeight(), 1.0);
}

void testAABB() {
    CylinderShape cyl(0.5, 1.0);
    pe::Vector3 min, max;

    cyl.getAABB(pe::Transform::identity(), min, max);
    ASSERT_EQUAL(min.x, -0.5)
    ASSERT_EQUAL(min.y, -0.5)
    ASSERT_EQUAL(min.z, -0.5)
    ASSERT_EQUAL(max.x, 0.5)
    ASSERT_EQUAL(max.y, 0.5)
    ASSERT_EQUAL(max.z, 0.5)

    pe::Transform transform;
    transform.setTranslation(pe::Vector3(1., 2., 3.));
    cyl.getAABB(transform, min, max);
    ASSERT_EQUAL(min.x, 0.5)
    ASSERT_EQUAL(min.y, 1.5)
    ASSERT_EQUAL(min.z, 2.5)
    ASSERT_EQUAL(max.x, 1.5)
    ASSERT_EQUAL(max.y, 2.5)
    ASSERT_EQUAL(max.z, 3.5)

    transform.setRotation(pe::Vector3::forward(), PE_PI / 6);
    cyl.getAABB(transform, min, max);
    ASSERT_EQUAL(min.x, 0.317)
    ASSERT_EQUAL(min.y, 1.317)
    ASSERT_EQUAL(min.z, 2.5)
    ASSERT_EQUAL(max.x, 1.683)
    ASSERT_EQUAL(max.y, 2.683)
    ASSERT_EQUAL(max.z, 3.5)
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

    cyl.project(pe::Transform::identity(), pe::Vector3::up(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_EQUAL(minPoint.y, -0.5)
    ASSERT_EQUAL(maxPoint.y, 0.5)

    cyl.project(pe::Transform::identity(), pe::Vector3::right(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_EQUAL(minPoint.x, -0.5)
    ASSERT_EQUAL(maxPoint.x, 0.5)

    cyl.project(pe::Transform::identity(), pe::Vector3::forward(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_EQUAL(minPoint.z, -0.5)
    ASSERT_EQUAL(maxPoint.z, 0.5)

    pe::Transform transform;
    transform.setRotation({0, 0, 1}, PE_PI / 6);
    transform.setTranslation({2, 2, 2});
    cyl.project(transform, pe::Vector3::up(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.317)
    ASSERT_EQUAL(max, 2.683)
    ASSERT_VECTOR3_EQUAL(minPoint, pe::Vector3(1.817, 1.317, 2));
    ASSERT_VECTOR3_EQUAL(maxPoint, pe::Vector3(2.183, 2.683, 2));

    cyl.project(transform, pe::Vector3::right(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.317)
    ASSERT_EQUAL(max, 2.683)
    ASSERT_VECTOR3_EQUAL(minPoint, pe::Vector3(1.317, 2.183, 2));
    ASSERT_VECTOR3_EQUAL(maxPoint, pe::Vector3(2.683, 1.817, 2));

    cyl.project(transform, pe::Vector3::forward(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.5)
    ASSERT_EQUAL(max, 2.5)
    ASSERT_EQUAL(minPoint.z, 1.5)
    ASSERT_EQUAL(maxPoint.z, 2.5)
}

std::string getUniqueEdges() {
    pe_phys_shape::ConvexMeshShape mesh;
    mesh.setMesh(pe_phys_shape::_cylinder_mesh);
    std::stringstream str;
    str << "const pe::Array<pe::Vector3> _cylinder_unique_edges = { //NOLINT\n";
    auto& edges = mesh.getUniqueEdges();
    for (int i = 0; i < edges.size();) {
        for (int j = 0; j < 2; j++) {
            str << "{" << edges[i].x << ", " << edges[i].y << ", " << edges[i].z << "}, ";
            i++;
        }
        str << "\n";
    }
    str << "};\n";
    return std::move(str.str());
}

int main() {
    std::cout << getUniqueEdges() << std::endl;
//    testConstruct();
//    testAABB();
//    testIsInside();
//    testProject();
}
