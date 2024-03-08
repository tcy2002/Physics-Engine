#include "test_general.h"
#include "phys/collision/collision_algorithm/mesh_mesh_collision_algorithm.h"
#include "phys/collision/collision_algorithm/box_box_collision_algorithm.h"
#include "phys/object/rigidbody.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/box_shape.h"
#include "phys/fracture/fracture_utils/default_mesh.h"
#include <fstream>

using namespace pe_phys_collision;

void objToMesh(pe::Mesh& mesh, const std::string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return;
    }

    std::string c;
    pe::Real y_min = PE_REAL_MAX;
    while (file >> c) {
        if (c == "v") {
            pe::Real x, y, z;
            file >> x >> y >> z;
            y_min = std::min(y_min, y);
            mesh.vertices.push_back({{x, y, z}, {0, 0, 0}});
        } else if (c == "f") {
            int i;
            pe::Mesh::Face face;
            while (file >> i) {
                face.indices.push_back(i - 1);
            }
            mesh.faces.push_back(face);
            file.clear();
        }
    }
    std::cout << "y_min: " << y_min << std::endl;

    pe::Mesh::perFaceNormal(mesh);
    for (auto& face : mesh.faces) {
        for (auto i : face.indices) {
            mesh.vertices[i].normal = face.normal;
        }
    }

    file.close();
}

pe::Mesh resizeCylinder(pe::Real radius, pe::Real height) {
    pe::Mesh result = pe_phys_fracture::_cylinder_mesh;
    for (auto& v : result.vertices) {
        v.position.y *= height;
        v.position.x *= (radius / 0.5);
        v.position.z *= (radius / 0.5);
    }
    return std::move(result);
}

pe::Mesh resizeBox(const pe::Vector3& size) {
    pe::Mesh result = pe_phys_fracture::_box_mesh;
    for (auto& v : result.vertices) {
        v.position = v.position * size;
    }
    return std::move(result);
}

void testMeshMesh() {
    pe::Mesh mesh;
    objToMesh(mesh, CURRENT_TEST_SOURCE_DIR "/mesh1.obj");
    auto rb1 = new pe_phys_object::RigidBody();
//    rb1->setCollisionShape(new pe_phys_shape::ConvexMeshShape(resizeBox(pe::Vector3(1, 1, 1))));
    rb1->setCollisionShape(new pe_phys_shape::ConvexMeshShape(mesh));
//    rb1->setCollisionShape(new pe_phys_shape::BoxShape({1, 1, 1}));
//    pe::Transform trans;
//    trans.setRotation({0, 0, 1}, 3.14159265 / 12);
//    trans.setOrigin({0.3536, 0.6123, 0});
//    rb1->setTransform(trans);
    rb1->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -1.4986, 0)));
    auto rb2 = new pe_phys_object::RigidBody();
    rb2->setCollisionShape(new pe_phys_shape::ConvexMeshShape(resizeBox(pe::Vector3(20, 1, 20))));
//    rb2->setCollisionShape(new pe_phys_shape::BoxShape({20, 1, 20}));
    rb2->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -0.5, 0)));

    auto alg = new MeshMeshCollisionAlgorithm();
//    auto alg = new BoxBoxCollisionAlgorithm();
    ContactResult result;
    pe::Vector3 overlap_min, overlap_max;
    alg->processCollision(rb1, rb2, result, overlap_min, overlap_max);

    std::cout << result.getPointSize() << std::endl;
    for (int i = 0; i < result.getPointSize(); i++) {
        auto& p = result.getContactPoint(i);
        std::cout << p.getDistance() << " ";
        std::cout << p.getAppliedImpulse();
        std::cout << p.getWorldPos();
        std::cout << p.getWorldNormal();
        std::cout << p.getLocalPosA();
        std::cout << p.getLocalPosB() << std::endl;
    }
}

int main() {
    testMeshMesh();
}