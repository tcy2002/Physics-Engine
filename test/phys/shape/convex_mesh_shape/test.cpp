#include "phys/shape/convex_mesh_shape.h"
#include "common/eigen_std.h"
#include "test_general.h"

using namespace pe_phys_shape;

void meshToObj(const pe::Mesh &mesh, const std::string &filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return;
    }

    for (auto& point : mesh.vertices) {
        auto& p = point.position;
        auto& n = point.normal;
        file << "v " << p.x << " " << p.y << " " << p.z << std::endl;
        file << "vn " << n.x << " " << n.y << " " << n.z << std::endl;
    }
    for (auto& face: mesh.faces) {
        file << "f";
        for (auto& index : face.indices) {
            file << " " << index + 1 << "//" << index + 1;
        }
        file << std::endl;
    }
    file.close();
}

const pe::Mesh _cylinder_mesh = { //NOLINT
        {
                {{0.5, 0.5, 0}, {0, 1, 0}},
                {{0.475528, 0.5, -0.154508}, {0, 1, 0}},
                {{0.404508, 0.5, -0.293893}, {0, 1, 0}},
                {{0.293893, 0.5, -0.404508}, {0, 1, 0}},
                {{0.154508, 0.5, -0.475528}, {0, 1, 0}},
                {{0, 0.5, -0.5}, {0, 1, 0}},
                {{-0.154508, 0.5, -0.475528}, {0, 1, 0}},
                {{-0.293893, 0.5, -0.404508}, {0, 1, 0}},
                {{-0.404508, 0.5, -0.293893}, {0, 1, 0}},
                {{-0.475528, 0.5, -0.154508}, {0, 1, 0}},
                {{-0.5, 0.5, 0}, {0, 1, 0}},
                {{-0.475528, 0.5, 0.154508}, {0, 1, 0}},
                {{-0.404508, 0.5, 0.293893}, {0, 1, 0}},
                {{-0.293893, 0.5, 0.404508}, {0, 1, 0}},
                {{-0.154508, 0.5, 0.475528}, {0, 1, 0}},
                {{0, 0.5, 0.5}, {0, 1, 0}},
                {{0.154508, 0.5, 0.475528}, {0, 1, 0}},
                {{0.293893, 0.5, 0.404508}, {0, 1, 0}},
                {{0.404508, 0.5, 0.293893}, {0, 1, 0}},
                {{0.475528, 0.5, 0.154508}, {0, 1, 0}},
                {{0.493844, -0.5, -0.078217}, {0, -1, 0}},
                {{0.445503, -0.5, -0.226995}, {0, -1, 0}},
                {{0.353553, -0.5, -0.353553}, {0, -1, 0}},
                {{0.226995, -0.5, -0.445503}, {0, -1, 0}},
                {{0.078217, -0.5, -0.493844}, {0, -1, 0}},
                {{-0.078217, -0.5, -0.493844}, {0, -1, 0}},
                {{-0.226995, -0.5, -0.445503}, {0, -1, 0}},
                {{-0.353553, -0.5, -0.353553}, {0, -1, 0}},
                {{-0.445503, -0.5, -0.226995}, {0, -1, 0}},
                {{-0.493844, -0.5, -0.078217}, {0, -1, 0}},
                {{-0.493844, -0.5, 0.078217}, {0, -1, 0}},
                {{-0.445503, -0.5, 0.226995}, {0, -1, 0}},
                {{-0.353553, -0.5, 0.353553}, {0, -1, 0}},
                {{-0.226995, -0.5, 0.445503}, {0, -1, 0}},
                {{-0.078217, -0.5, 0.493844}, {0, -1, 0}},
                {{0.078217, -0.5, 0.493844}, {0, -1, 0}},
                {{0.226995, -0.5, 0.445503}, {0, -1, 0}},
                {{0.353553, -0.5, 0.353553}, {0, -1, 0}},
                {{0.445503, -0.5, 0.226995}, {0, -1, 0}},
                {{0.493844, -0.5, 0.078217}, {0, -1, 0}},
                {{0.5, 0.5, 0}, {1, 0, 0}},
                {{0.475528, 0.5, -0.154508}, {0.951056, 0, -0.309017}},
                {{0.404508, 0.5, -0.293893}, {0.809017, 0, -0.587785}},
                {{0.293893, 0.5, -0.404508}, {0.587785, 0, -0.809017}},
                {{0.154508, 0.5, -0.475528}, {0.309017, 0, -0.951056}},
                {{0, 0.5, -0.5}, {0, 0, -1}},
                {{-0.154508, 0.5, -0.475528}, {-0.309017, 0, -0.951056}},
                {{-0.293893, 0.5, -0.404508}, {-0.587785, 0, -0.809017}},
                {{-0.404508, 0.5, -0.293893}, {-0.809017, 0, -0.587785}},
                {{-0.475528, 0.5, -0.154508}, {-0.951056, 0, -0.309017}},
                {{-0.5, 0.5, 0}, {-1, 0, 0}},
                {{-0.475528, 0.5, 0.154508}, {-0.951056, 0, 0.309017}},
                {{-0.404508, 0.5, 0.293893}, {-0.809017, 0, 0.587785}},
                {{-0.293893, 0.5, 0.404508}, {-0.587785, 0, 0.809017}},
                {{-0.154508, 0.5, 0.475528}, {-0.309017, 0, 0.951056}},
                {{0, 0.5, 0.5}, {0, 0, 1}},
                {{0.154508, 0.5, 0.475528}, {0.309017, 0, 0.951056}},
                {{0.293893, 0.5, 0.404508}, {0.587785, 0, 0.809017}},
                {{0.404508, 0.5, 0.293893}, {0.809017, 0, 0.587785}},
                {{0.475528, 0.5, 0.154508}, {0.951056, 0, 0.309017}},
                {{0.493844, -0.5, -0.078217}, {0.987688, 0, -0.156434}},
                {{0.445503, -0.5, -0.226995}, {0.891006, 0, -0.453990}},
                {{0.353553, -0.5, -0.353553}, {0.707107, 0, -0.707107}},
                {{0.226995, -0.5, -0.445503}, {0.453990, 0, -0.891006}},
                {{0.078217, -0.5, -0.493844}, {0.156434, 0, -0.987688}},
                {{-0.078217, -0.5, -0.493844}, {-0.156434, 0, -0.987688}},
                {{-0.226995, -0.5, -0.445503}, {-0.453990, 0, -0.891006}},
                {{-0.353553, -0.5, -0.353553}, {-0.707107, 0, -0.707107}},
                {{-0.445503, -0.5, -0.226995}, {-0.891006, 0, -0.453990}},
                {{-0.493844, -0.5, -0.078217}, {-0.987688, 0, -0.156434}},
                {{-0.493844, -0.5, 0.078217}, {-0.987688, 0, 0.156434}},
                {{-0.445503, -0.5, 0.226995}, {-0.891006, 0, 0.453990}},
                {{-0.353553, -0.5, 0.353553}, {-0.707107, 0, 0.707107}},
                {{-0.226995, -0.5, 0.445503}, {-0.453990, 0, 0.891006}},
                {{-0.078217, -0.5, 0.493844}, {-0.156434, 0, 0.987688}},
                {{0.078217, -0.5, 0.493844}, {0.156434, 0, 0.987688}},
                {{0.226995, -0.5, 0.445503}, {0.453990, 0, 0.891006}},
                {{0.353553, -0.5, 0.353553}, {0.707107, 0, 0.707107}},
                {{0.445503, -0.5, 0.226995}, {0.891006, 0, 0.453990}},
                {{0.493844, -0.5, 0.078217}, {0.987688, 0, 0.156434}},
        },
        {
                {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}, {0, 1, 0}},
                {{39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20}, {0, -1, 0}},
                {{60, 41, 40}, {0.987670, 0.006156, -0.156432}},
                {{61, 42, 41}, {0.890990, 0.006156, -0.453982}},
                {{62, 43, 42}, {0.707093, 0.006156, -0.707093}},
                {{63, 44, 43}, {0.453982, 0.006156, -0.890990}},
                {{64, 45, 44}, {0.156432, 0.006156, -0.987670}},
                {{65, 46, 45}, {-0.156432, 0.006156, -0.987670}},
                {{66, 47, 46}, {-0.493982, 0.006156, -0.890990}},
                {{67, 48, 47}, {-0.707093, 0.006156, -0.707093}},
                {{68, 49, 48}, {-0.890990, 0.006156, -0.453982}},
                {{69, 50, 49}, {-0.987670, 0.006156, -0.156432}},
                {{70, 51, 50}, {-0.987670, 0.006156, 0.156432}},
                {{71, 52, 51}, {-0.890990, 0.006156, 0.453982}},
                {{72, 53, 52}, {-0.707093, 0.006156, 0.707093}},
                {{73, 54, 53}, {-0.453982, 0.006156, 0.890990}},
                {{74, 55, 54}, {-0.156432, 0.006156, 0.987670}},
                {{75, 56, 55}, {0.156432, 0.006156, 0.987670}},
                {{76, 57, 56}, {0.453982, 0.006156, 0.890990}},
                {{77, 58, 57}, {0.707093, 0.006156, 0.707093}},
                {{78, 59, 58}, {0.890990, 0.006156, 0.453982}},
                {{79, 40, 59}, {0.987670, 0.006156, 0.156432}},
                {{40, 79, 60}, {0.999981, -0.006156, 0}},
                {{41, 60, 61}, {0.951038, -0.006156, -0.309011}},
                {{42, 61, 62}, {0.809002, -0.006156, -0.587774}},
                {{43, 62, 63}, {0.587774, -0.006156, -0.809002}},
                {{44, 63, 64}, {0.309011, -0.006156, -0.951038}},
                {{45, 64, 65}, {0, -0.006156, -0.999981}},
                {{46, 65, 66}, {-0.309011, -0.006156, -0.951038}},
                {{47, 66, 67}, {-0.587774, -0.006156, -0.809002}},
                {{48, 67, 68}, {-0.809002, -0.006156, -0.587774}},
                {{49, 68, 69}, {-0.951038, -0.006156, -0.309011}},
                {{50, 69, 70}, {-0.999981, -0.006156, 0}},
                {{51, 70, 71}, {-0.951038, -0.006156, 0.309011}},
                {{52, 71, 72}, {-0.809002, -0.006156, 0.587774}},
                {{53, 72, 73}, {-0.587774, -0.006156, 0.809002}},
                {{54, 73, 74}, {-0.309011, -0.006156, 0.951038}},
                {{55, 74, 75}, {0, -0.006156, 0.999981}},
                {{56, 75, 76}, {0.309011, -0.006156, 0.951038}},
                {{57, 76, 77}, {0.587774, -0.006156, 0.809002}},
                {{58, 77, 78}, {0.809002, -0.006156, 0.587774}},
                {{59, 78, 79}, {0.951038, -0.006156, 0.309011}},
        },
};

void testConstruct() {
    ConvexMeshShape meshShape(_cylinder_mesh);

    ASSERT_EQUAL_INT(meshShape.getType(), ShapeType::MESH)
    ASSERT_EQUAL_INT(meshShape.isConvex(), true)

    auto& mesh = meshShape.getMesh();
    ASSERT_EQUAL_INT(mesh.vertices.size(), 80)
    ASSERT_EQUAL_INT(mesh.faces.size(), 42)
    meshToObj(mesh, SHAPE_TEST_SOURCE_DIR "/convex_mesh_shape/mesh.obj");
}

void testAABB() {
    ConvexMeshShape meshShape(_cylinder_mesh);
    pe::Vector3 min, max;

    meshShape.getAABB(pe::Transform::identity(), min, max);
    ASSERT_EQUAL(min.x, -0.5)
    ASSERT_EQUAL(min.y, -0.5)
    ASSERT_EQUAL(min.z, -0.5)
    ASSERT_EQUAL(max.x, 0.5)
    ASSERT_EQUAL(max.y, 0.5)
    ASSERT_EQUAL(max.z, 0.5)

    pe::Transform transform;
    transform.setRotation(pe::Vector3::up(), M_PI / 4);
    transform.setTranslation(pe::Vector3(1., 2., 3.));
    meshShape.getAABB(transform, min, max);
    ASSERT_EQUAL(min.x, 0.5)
    ASSERT_EQUAL(min.y, 1.5)
    ASSERT_EQUAL(min.z, 2.5)
    ASSERT_EQUAL(max.x, 1.5)
    ASSERT_EQUAL(max.y, 2.5)
    ASSERT_EQUAL(max.z, 3.5)
}

void testIsInside() {
    ConvexMeshShape meshShape(_cylinder_mesh);

    ASSERT_EQUAL(meshShape.isInside(pe::Transform::identity(),
                                    pe::Vector3(0., 0., 0.)),
                 true)
    ASSERT_EQUAL(meshShape.isInside(pe::Transform::identity(),
                                    pe::Vector3(0.34, 0.499, 0.34)),
                 true)
    ASSERT_EQUAL(meshShape.isInside(pe::Transform::identity(),
                                    pe::Vector3(0.36, 0.501, 0.36)),
                 false)

    pe::Transform transform;
    transform.setRotation({0, 1, 0}, M_PI / 4);
    transform.setTranslation({2, 2, 2});
    ASSERT_EQUAL(meshShape.isInside(transform,
                                    pe::Vector3(2., 2., 2.)),
                 true)
    ASSERT_EQUAL(meshShape.isInside(transform,
                                    pe::Vector3(2.34, 2.499, 2.34)),
                 true)
    ASSERT_EQUAL(meshShape.isInside(transform,
                                    pe::Vector3(2.36, 2.501, 2.36)),
                 false)
}

void testProject() {
    ConvexMeshShape meshShape(_cylinder_mesh);
    pe::Real min, max;
    pe::Vector3 minPoint, maxPoint;

    meshShape.project(pe::Transform::identity(), pe::Vector3::up(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_VECTOR3_EQUAL(minPoint, pe::Vector3(0, -0.5, 0));
    ASSERT_VECTOR3_EQUAL(maxPoint, pe::Vector3(0, 0.5, 0));

    meshShape.project(pe::Transform::identity(), pe::Vector3::right(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_VECTOR3_EQUAL(minPoint, pe::Vector3(-0.5, 0, 0));
    ASSERT_VECTOR3_EQUAL(maxPoint, pe::Vector3(0.5, 0, 0));

    meshShape.project(pe::Transform::identity(), pe::Vector3::forward(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_VECTOR3_EQUAL(minPoint, pe::Vector3(0, 0, -0.5));
    ASSERT_VECTOR3_EQUAL(maxPoint, pe::Vector3(0, 0, 0.5));

    pe::Transform transform;
    transform.setRotation({0, 1, 0}, M_PI / 4);
    transform.setTranslation({2, 2, 2});
    meshShape.project(transform, pe::Vector3::up(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.5)
    ASSERT_EQUAL(max, 2.5)
    ASSERT_VECTOR3_EQUAL(minPoint, pe::Vector3(0, 1.5, 0));
    ASSERT_VECTOR3_EQUAL(maxPoint, pe::Vector3(0, 2.5, 0));

    meshShape.project(transform, pe::Vector3::right(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.5)
    ASSERT_EQUAL(max, 2.5)
    ASSERT_VECTOR3_EQUAL(minPoint, pe::Vector3(1.5, 0, 0));
    ASSERT_VECTOR3_EQUAL(maxPoint, pe::Vector3(2.5, 0, 0));

    meshShape.project(transform, pe::Vector3::forward(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.5)
    ASSERT_EQUAL(max, 2.5)
    ASSERT_VECTOR3_EQUAL(minPoint, pe::Vector3(0, 0, 1.5));
    ASSERT_VECTOR3_EQUAL(maxPoint, pe::Vector3(0, 0, 2.5));
}

int main() {
    testConstruct();
    testAABB();
    testIsInside();
    testProject();
}
