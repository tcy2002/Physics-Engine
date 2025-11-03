#include "physics/physics.h"
#include "physics/object/rigidbody.h"
#include "physics/collision/collision_algorithm/box_capsule_collision_algorithm.h"
#include "physics/shape/box_shape.h"
#include "physics/shape/capsule_shape.h"
#include "physics/fracture/fracture_utils/fracture_data_manager.h"
#include <fstream>
#include <sstream>

using namespace pe_physics_collision;

pe_physics_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
                                                 const pe::Vector3& size, pe::Real mass) {
    /* This function creates a box-shaped rigidbody */

    auto rb = new pe_physics_object::RigidBody();
    rb->setMass(mass);
    auto shape = new pe_physics_shape::BoxShape(size);
    rb->setCollisionShape(shape);
    rb->setTransform(trans);
    rb->setFrictionCoeff(PE_R(0.5)); // friction coefficient
    rb->setRestitutionCoeff(PE_R(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
    rb->setAngularDamping(PE_R(0.8)); // angular damping parameter (slows down the rotation speed)
    return rb;
}

pe_physics_object::RigidBody* createCapsuleRigidBody(const pe::Transform& trans,
                                                     const pe::Real& radius, const pe::Real& height, pe::Real mass) {
    /* This function creates a box-shaped rigidbody */

    auto rb = new pe_physics_object::RigidBody();
    rb->setMass(mass);
    auto shape = new pe_physics_shape::CapsuleShape(radius, height);
    rb->setCollisionShape(shape);
    rb->setTransform(trans);
    rb->setFrictionCoeff(PE_R(0.5)); // friction coefficient
    rb->setRestitutionCoeff(PE_R(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
    rb->setAngularDamping(PE_R(0.8)); // angular damping parameter (slows down the rotation speed)
    return rb;
}

void load_obj_with_normal(const std::string& filepath, pe::Mesh& mesh) {
    std::fstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file." << filepath << std::endl;
        return;
    }

    char buf[1024];
    pe::Array<pe::Vector3> temp_vertices;
    pe::Array<pe::Vector3> temp_normals;
    while (file.getline(buf, 1024)) {
        std::stringstream ss(buf);
        std::string str;
        ss >> str;
        if (str == "v") {
            pe::Real x, y, z;
            ss >> x >> y >> z;
            temp_vertices.emplace_back(x, y, z);
        } else if (str == "vn") {
            pe::Real x, y, z;
            ss >> x >> y >> z;
            temp_normals.emplace_back(x, y, z);
        } else if (str == "f") {
            std::string vert;
            pe::Mesh::Face face;
            while (ss >> vert) {
                int vi = std::atoi(vert.substr(0, vert.find_first_of('/')).c_str());
                face.indices.push_back(vi - 1);
            }
            mesh.faces.push_back(face);
        }
    }

    for (int i = 0; i < temp_vertices.size(); i++) {
        mesh.vertices.push_back({temp_vertices[i], temp_normals[i]});
    }

    pe::Mesh::perFaceNormal(mesh);
}

void output_txt(const std::string& filepath, const pe::Mesh& mesh) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file." << filepath << std::endl;
        return;
    }

    file << "{\\\n";
    file << "    {\\\n";
    for (int i = 0; i < mesh.vertices.size(); i++) {
        auto& p = mesh.vertices[i].position;
        auto& n = mesh.vertices[i].normal;
        file << "        {";
        file << "{PE_R(" << p.x << "), PE_R(" << p.y << "), PE_R(" << p.z << ")}";
        file << ", ";
        file << "{PE_R(" << n.x << "), PE_R(" << n.y << "), PE_R(" << n.z << ")}";
        file << (i == mesh.vertices.size() - 1 ? "}\\\n" : "},\\\n");
    }
    file << "    },\\\n";
    file << "    {\\\n";
    for (int i = 0; i < mesh.faces.size(); i++) {
        auto& f = mesh.faces[i].indices;
        auto& fn = mesh.faces[i].normal;
        file << "        {{";
        for (int j = 0; j < f.size(); j++) {
            file << f[j];
            if (j != f.size() - 1) file << ", ";
        }
        file << "}, {PE_R(" << fn.x << "), PE_R(" << fn.y << "), PE_R(" << fn.z << ")}}";
        file << (i == mesh.faces.size() - 1 ? "\\\n" : ",\\\n");
    }
    file << "    }\\\n";
    file << "}\n";
}

void testMeshMesh() {
    pe::Mesh mesh;
    load_obj_with_normal("../../capsule.obj", mesh);
    pe_physics_fracture::FractureDataManager fdm;
    fdm.import_from_mesh(mesh);
    pe::Mesh new_mesh;
    fdm.export_to_mesh(new_mesh);
    pe::Mesh::saveToObj("../../capsule_processed.obj", new_mesh, pe::Vector3::ones());
    output_txt("../../capsule.txt", new_mesh);

 //    // add a ground
 //    auto rb2 = createBoxRigidBody(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), PE_PI / PE_R(2.0)), pe::Vector3(-5, 0, 0)),
 //                                  pe::Vector3(10, 10, 10), 10);
 //    rb2->setKinematic(true);
 //
 //    // add a capsule
 //    auto rb1 = createCapsuleRigidBody(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), -PE_PI / PE_R(1000.0)), pe::Vector3(PE_R(0.45), 0, 0)),
 //                                      PE_R(0.5), PE_R(1.0), PE_R(1.0));
 //
 //    auto alg = new BoxCapsuleCollisionAlgorithm();
 //    ContactResult result;
 //    result.setObjectA(rb1);
	// result.setObjectB(rb2);
 //    pe::Real refScale = (rb1->getAABBScale() + rb2->getAABBScale()) * PE_DIST_REF_RADIO;
 //    alg->processCollision(rb1->getCollisionShape(), rb2->getCollisionShape(), rb1->getTransform(), rb2->getTransform(), refScale, result);
 //    result.sortContactPoints();
 //
 //    std::cout << result.getPointSize() << std::endl;
 //    for (int i = 0; i < result.getPointSize(); i++) {
 //        auto& p = result.getContactPoint(i);
 //        std::cout << p.getDistance() << " ";
 //        std::cout << p.getWorldPos();
 //        std::cout << p.getWorldNormal();
 //        std::cout << p.getLocalPosA();
 //        std::cout << p.getLocalPosB() << std::endl;
 //    }
}

int main() {
    testMeshMesh();
}