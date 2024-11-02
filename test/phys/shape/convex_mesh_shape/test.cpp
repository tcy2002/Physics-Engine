#include <fstream>
#include <sstream>
#include "phys/shape/box_shape.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/default_mesh.h"
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

// void objToMesh(pe::Mesh& mesh, const std::string &filename) {
//     std::ifstream file(filename);
//     if (!file.is_open()) {
//         std::cerr << "Failed to open file " << filename << std::endl;
//         return;
//     }
//
//     std::string c;
//     while (file >> c) {
//         if (c == "v") {
//             pe::Real x, y, z;
//             file >> x >> y >> z;
//             mesh.vertices.push_back({{x, y, z}, {0, 0, 0}});
//         } else if (c == "f") {
//             int i;
//             pe::Mesh::Face face;
//             while (file >> i) {
//                 face.indices.push_back(i - 1);
//             }
//             mesh.faces.push_back(face);
//             file.clear();
//         }
//     }
//
//     pe::Mesh::perFaceNormal(mesh);
//     for (auto& face : mesh.faces) {
//         for (auto i : face.indices) {
//             mesh.vertices[i].normal = face.normal;
//         }
//     }
//
//     file.close();
// }

void objToMesh(pe::Mesh& mesh, const std::string& filename, pe::Real size) {
    std::fstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return;
    }

    char buf[1024];
    while (file.getline(buf, 1024)) {
        std::stringstream ss(buf);
        std::string str;
        ss >> str;
        if (str == "v") {
            pe::Real x, y, z;
            ss >> x >> y >> z;
            mesh.vertices.push_back({ {x * size, y * size, z * size}, {0, 0, 0} });
        }
        else if (str == "f") {
            std::string vert;
            pe::Mesh::Face face;
            while (ss >> vert) {
                int vi = std::atoi(vert.substr(0, vert.find_first_of('/')).c_str());
                face.indices.push_back(vi - 1);
            }
            mesh.faces.push_back(face);
        }
    }

    pe::Mesh::perFaceNormal(mesh);
    pe::Mesh::perVertexNormal(mesh);
    file.close();
}

void testConstruct() {
    ConvexMeshShape meshShape;
    meshShape.setMesh(PE_CYLINDER_DEFAULT_MESH);

    ASSERT_EQUAL_INT(meshShape.getType(), ShapeType::ConvexMesh)
    ASSERT_EQUAL_INT(meshShape.isConvex(), true)

    auto& edges = meshShape.getUniqueEdges();
    ASSERT_EQUAL_INT(edges.size(), 80)

    auto& mesh = meshShape.getMesh();
    ASSERT_EQUAL_INT(mesh.vertices.size(), 80)
    ASSERT_EQUAL_INT(mesh.faces.size(), 42)
    meshToObj(mesh, CURRENT_TEST_SOURCE_DIR "/mesh.obj");
}

void testAABB() {
    ConvexMeshShape meshShape;
    meshShape.setMesh(PE_CYLINDER_DEFAULT_MESH);
    pe::Vector3 min, max;

    meshShape.getAABB(pe::Transform::identity(), min, max);
    ASSERT_EQUAL(min.x, -0.5)
    ASSERT_EQUAL(min.y, -0.5)
    ASSERT_EQUAL(min.z, -0.5)
    ASSERT_EQUAL(max.x, 0.5)
    ASSERT_EQUAL(max.y, 0.5)
    ASSERT_EQUAL(max.z, 0.5)

    pe::Transform transform;
    transform.setRotation(pe::Vector3::up(), PE_PI / 4);
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
    ConvexMeshShape meshShape;
    meshShape.setMesh(PE_CYLINDER_DEFAULT_MESH);

    ASSERT_EQUAL(meshShape.localIsInside(pe::Vector3(0., 0., 0.)),
                 true)
    ASSERT_EQUAL(meshShape.localIsInside(pe::Vector3(0.34, 0.499, 0.34)),
                 true)
    ASSERT_EQUAL(meshShape.localIsInside(pe::Vector3(0.36, 0.501, 0.36)),
                 false)
}

void testProject() {
    ConvexMeshShape meshShape;
    meshShape.setMesh(PE_CYLINDER_DEFAULT_MESH);
    pe::Real min, max;
    pe::Vector3 minPoint, maxPoint;

    meshShape.project(pe::Transform::identity(), pe::Vector3::up(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_EQUAL(minPoint.y, -0.5)
    ASSERT_EQUAL(maxPoint.y, 0.5)

    meshShape.project(pe::Transform::identity(), pe::Vector3::right(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_EQUAL(minPoint.x, -0.5)
    ASSERT_EQUAL(maxPoint.x, 0.5)

    meshShape.project(pe::Transform::identity(), pe::Vector3::forward(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, -0.5)
    ASSERT_EQUAL(max, 0.5)
    ASSERT_EQUAL(minPoint.z, -0.5)
    ASSERT_EQUAL(maxPoint.z, 0.5)

    pe::Transform transform;
    transform.setRotation({0, 1, 0}, PE_PI / 4);
    transform.setTranslation({2, 2, 2});
    meshShape.project(transform, pe::Vector3::up(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.5)
    ASSERT_EQUAL(max, 2.5)
    ASSERT_EQUAL(minPoint.y, 1.5)
    ASSERT_EQUAL(maxPoint.y, 2.5)

    meshShape.project(transform, pe::Vector3::right(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.5)
    ASSERT_EQUAL(max, 2.5)
    ASSERT_EQUAL(minPoint.x, 1.5)
    ASSERT_EQUAL(maxPoint.x, 2.5)

    meshShape.project(transform, pe::Vector3::forward(), min, max, minPoint, maxPoint);
    ASSERT_EQUAL(min, 1.5)
    ASSERT_EQUAL(max, 2.5)
    ASSERT_EQUAL(minPoint.z, 1.5)
    ASSERT_EQUAL(maxPoint.z, 2.5)
}

int main() {
    testConstruct();
    testAABB();
    testIsInside();
    testProject();
}
