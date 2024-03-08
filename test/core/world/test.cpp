#include "core/world.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/fracture/fracture_solver/fracture_solver.h"
#include "phys/object/fracturable_object.h"
#include "phys/fracture/fracture_utils/default_mesh.h"
#include "core/viewer.h"
#include <fstream>

void objToMesh(pe::Mesh& mesh, const std::string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return;
    }

    std::string c;
    while (file >> c) {
        if (c == "v") {
            pe::Real x, y, z;
            file >> x >> y >> z;
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

    pe::Mesh::perFaceNormal(mesh);
    for (auto& face : mesh.faces) {
        for (auto i : face.indices) {
            mesh.vertices[i].normal = face.normal;
        }
    }

    file.close();
}

pe::Mesh resizeBox(const pe::Vector3& size) {
    pe::Mesh result = pe_phys_fracture::_box_mesh;
    for (auto& v : result.vertices) {
        v.position = v.position * size;
    }
    return std::move(result);
}

pe_phys_object::RigidBody* createMeshRigidBody(const pe::Vector3& pos, const pe::Vector3& size,
                                               const std::string& filename = "") {
    auto rb = new pe_phys_object::RigidBody();
    rb->setMass(1.0);
    rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
    rb->setFrictionCoeff(0.5);
    rb->setRestitutionCoeff(0.8);
    if (filename.empty()) {
        rb->setCollisionShape(new pe_phys_shape::ConvexMeshShape(resizeBox(size)));
        pe::Real x = size.x, y = size.y, z = size.z;
        rb->setLocalInertia({(y * y + z * z) / 12, 0, 0,
                             0, (x * x + z * z) / 12, 0,
                             0, 0, (x * x + y * y) / 12});
    } else {
        pe::Mesh mesh;
        objToMesh(mesh, filename);
        auto shape = new pe_phys_shape::ConvexMeshShape(mesh);
        rb->setCollisionShape(shape);
        rb->setLocalInertia(shape->calcLocalInertia(1.0));
    }
    return rb;
}

pe_phys_object::RigidBody* createRigidBody(const pe::Vector3& pos, const pe::Vector3& size) {
    auto rb = new pe_phys_object::RigidBody();
    rb->setMass(1.0);
    auto shape = new pe_phys_shape::BoxShape(size);
    rb->setCollisionShape(shape);
    rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
    rb->setLocalInertia(shape->calcLocalInertia(1.0));
    rb->setFrictionCoeff(0.5);
    rb->setRestitutionCoeff(0.8);
    return rb;
}

pe_phys_object::FracturableObject* createFracturableObject(const pe::Vector3& pos, const pe::Vector3& size, pe::Real th) {
    auto rb = new pe_phys_object::FracturableObject();
    rb->setMass(1.0);
    auto shape = new pe_phys_shape::BoxShape(size);
    rb->setCollisionShape(shape);
    rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
    rb->setLocalInertia(shape->calcLocalInertia(1.0));
    rb->setFrictionCoeff(0.5);
    rb->setRestitutionCoeff(0.8);
    rb->setThreshold(th);
    return rb;
}

void testWorld() {
    int rb_num = 0;
    auto world = new pe_core::World();
    world->setDt(0.01);
    world->setGravity(pe::Vector3(0, -9.8, 0));

    // open viewer
    pe_core::Viewer::open();

    // create rigid bodies
    auto rb1 = createMeshRigidBody(pe::Vector3(0, -0.5, 0), pe::Vector3(20, 1, 20));
    rb1->setKinematic(true);
//    auto rb2 = createRigidBody(pe::Vector3(0, 10, 0), pe::Vector3(1, 1, 1));
//    auto rb2 = createMeshRigidBody(pe::Vector3(0, 50, 0), pe::Vector3(1, 1, 1), CURRENT_TEST_SOURCE_DIR "/mesh.obj");
//    pe::Transform trans;
//    trans.setOrigin(pe::Vector3(0, 50, 0));
//    trans.setRotation({0, 1, 0}, 3.14159265 / 2);
//    rb2->setTransform(trans);
    auto rb2 = createFracturableObject(pe::Vector3(0, 2.1, 0), pe::Vector3(4, 4, 4), 1);
    auto fs = new pe_phys_fracture::FractureSolver();
    pe_phys_fracture::FractureSource src;
    src.type = pe_phys_fracture::FractureType::Sphere;
    src.position = pe::Vector3(1.5, 3.6, 1.5);
    src.intensity = pe::Vector3(0.5, 0.5, 0.5);
    fs->setFracturableObject(rb2);
    fs->solve({src});

    // add to world
//    world->addRigidBody(rb2);
    world->addRigidBody(rb1);
    pe::Array<pe_phys_object::RigidBody*> rbs;
    for (int i = 0; i < rb_num; i++) {
        auto rb = createRigidBody(pe::Vector3(0, 1 + i * 1.1, 0), pe::Vector3(1, 1, 1));
        rbs.push_back(rb);
        world->addRigidBody(rb);
    }
    for (auto rb : fs->getFragments()) {
        rbs.push_back(rb);
        world->addRigidBody(rb);
    }

    // add to viewer
    int id1 = pe_core::Viewer::addCube(pe::Vector3(20, 1, 20));
    pe_core::Viewer::updateCubeColor(id1, pe::Vector3(0.3, 0.3, 0.8));
    pe_core::Viewer::updateCubeTransform(id1, rb1->getTransform());
//    int id2 = pe_core::Viewer::addCube(pe::Vector3(1, 1, 1));
//    pe_core::Viewer::updateCubeColor(id2, pe::Vector3(0.3, 0.8, 0.3));
//    pe_core::Viewer::updateCubeTransform(id2, rb2->getTransform());
//    pe::Mesh mesh;
//    objToMesh(mesh, CURRENT_TEST_SOURCE_DIR "/mesh.obj");
//    int id2 = pe_core::Viewer::addMesh(mesh);
//    pe_core::Viewer::updateMeshColor(id2, pe::Vector3(0.3, 0.8, 0.3));
//    pe_core::Viewer::updateMeshTransform(id2, rb2->getTransform());
    pe::Array<int> ids;
    for (int i = 0; i < rb_num; i++) {
        int id = pe_core::Viewer::addCube(pe::Vector3(1, 1, 1));
        pe_core::Viewer::updateCubeColor(id, pe::Vector3(0.8, 0.3, 0.3));
        pe_core::Viewer::updateCubeTransform(id, rbs[i]->getTransform());
        ids.push_back(id);
    }
    for (auto rb : fs->getFragments()) {
        auto shape = (pe_phys_shape::ConvexMeshShape*)(rb->getCollisionShape());
        auto id = pe_core::Viewer::addMesh(shape->getMesh());
        pe_core::Viewer::updateMeshColor(id, pe::Vector3(0.3, 0.8, 0.3));
        pe_core::Viewer::updateMeshTransform(id, rb->getTransform());
        ids.push_back(id);
    }

    // main loop
    int frame = 0;
    while (pe_core::Viewer::getKeyState('q') != 0) {
        frame++;
        while (pe_core::Viewer::getKeyState('r') != 0);
        auto t = COMMON_GetTickCount();
//        pe_core::Viewer::updateCubeTransform(id2, rb2->getTransform());
//        pe_core::Viewer::updateMeshTransform(id2, rb2->getTransform());
        for (int i = 0; i < ids.size(); i++) {
            pe_core::Viewer::updateCubeTransform(ids[i], rbs[i]->getTransform());
            pe_core::Viewer::updateMeshTransform(ids[i], rbs[i]->getTransform());
        }
        world->step();
        COMMON_Sleep(10 - (int)(COMMON_GetTickCount() - t));
    }

    pe_core::Viewer::close();
}

int main() {
    testWorld();
}