#include "test_general.h"
#include "phys/collision/collision_algorithm/convex_convex_collision_algorithm.h"
#include "phys/object/rigidbody.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/default_mesh.h"
#include <fstream>
#include <sstream>

using namespace pe_phys_collision;

void objToMesh(pe::Mesh& mesh, const std::string& filename) {
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
            mesh.vertices.push_back({ {x, y, z}, {0, 0, 0} });
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
    for (auto& face : mesh.faces) {
        for (auto i : face.indices) {
            mesh.vertices[i].normal = face.normal;
        }
    }

    file.close();
}

pe::Mesh resizeCylinder(pe::Real radius, pe::Real height) {
    pe::Mesh result = PE_CYLINDER_DEFAULT_MESH;
    for (auto& v : result.vertices) {
        v.position.y *= height;
        v.position.x *= (radius / 0.5);
        v.position.z *= (radius / 0.5);
    }
    return std::move(result);
}

pe::Mesh resizeBox(const pe::Vector3& size) {
    pe::Mesh result = PE_BOX_DEFAULT_MESH;
    for (auto& v : result.vertices) {
        v.position = v.position * size;
    }
    return std::move(result);
}

void testMeshMesh() {
    pe::Mesh mesh;
    objToMesh(mesh, CURRENT_TEST_SOURCE_DIR "/mesh.obj");
    auto rb1 = new pe_phys_object::RigidBody();
    auto shape = new pe_phys_shape::ConvexMeshShape();
    shape->setMesh(mesh);
    rb1->setCollisionShape(shape);
    rb1->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0, 0)));
    auto rb2 = new pe_phys_object::RigidBody();
    shape = new pe_phys_shape::ConvexMeshShape();
    shape->setMesh(resizeBox({20, 1, 20}));
    rb2->setCollisionShape(shape);
    rb2->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -0.5, 0)));

    auto alg = new ConvexConvexCollisionAlgorithm();
    ContactResult result;
    result.setObjectA(rb1);
	result.setObjectB(rb2);
    alg->processCollision(rb1->getCollisionShape(), rb2->getCollisionShape(), rb1->getTransform(), rb2->getTransform(), result);
    result.sortContactPoints();

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