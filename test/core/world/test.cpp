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

pe_phys_object::RigidBody* createMeshRigidBody(const pe::Vector3& pos, const pe::Vector3& size, pe::Real mass,
                                               const std::string& filename = "") {
    auto rb = new pe_phys_object::RigidBody();
    rb->setMass(mass);
    rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
    rb->setFrictionCoeff(0.5);
    rb->setRestitutionCoeff(0.8);
    if (filename.empty()) {
        auto shape = new pe_phys_shape::ConvexMeshShape();
        shape->setMesh(resizeBox(size));
        rb->setCollisionShape(shape);
        rb->setLocalInertia(shape->calcLocalInertia(1.0));
        rb->setMargin(size * 0.005);
    } else {
        pe::Mesh mesh;
        objToMesh(mesh, filename);
        auto shape = new pe_phys_shape::ConvexMeshShape();
        shape->setMesh(mesh);
        rb->setCollisionShape(shape);
        rb->setLocalInertia(shape->calcLocalInertia(1.0));
        pe::Vector3 aabb_min, aabb_max;
        shape->getAABB(pe::Transform::identity(), aabb_min, aabb_max);
        rb->setMargin((aabb_max - aabb_min) * 0.005);
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
    rb->setMargin(size * 0.005);
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
    rb->setMargin(size * 0.001);
    return rb;
}

//#define TEST_SINGLE
#define TEST_FRAC
#define TEST_SECOND_GROUND

void testWorld() {
    int rb_num = 0;
    const auto filename = CURRENT_TEST_SOURCE_DIR "/test4.obj";
    auto world = new pe_core::World();
    world->setDt(0.01);
    world->setGravity(pe::Vector3(0, -9.8, 0));

    // open viewer
    pe_core::Viewer::open();

    // create rigid bodies
    auto rb1 = createMeshRigidBody(pe::Vector3(0, -0.5, 0), pe::Vector3(20, 1, 20), 400);
    rb1->setKinematic(true);
#ifdef TEST_SECOND_GROUND
    auto rb4 = createMeshRigidBody(pe::Vector3(0, 1.5, 0), pe::Vector3(10, 1, 10), 100);
#endif
#ifdef TEST_SINGLE
    auto rb2 = createMeshRigidBody(pe::Vector3(0, 10, 0), pe::Vector3(1, 1, 1), filename);
    pe::Transform trans, trans2;
    trans.setOrigin(rb2->getTransform().getOrigin());
    trans.setRotation({0, 0, 1}, -0.90);
    trans2.setRotation({1, 0, 0}, 0.05);
    trans.setBasis(trans2.getBasis() * trans.getBasis());
    rb2->setTransform(trans);
#endif
#ifdef TEST_FRAC
    auto rb3 = createFracturableObject(pe::Vector3(0, 5, 0), pe::Vector3(4, 4, 4), 1);
    auto fs = new pe_phys_fracture::FractureSolver();
    pe_phys_fracture::FractureSource src;
    src.type = pe_phys_fracture::FractureType::Sphere;
    src.position = pe::Vector3(1.5, 6.5, 1.5);
    src.intensity = pe::Vector3(0.5, 0.5, 0.5);
    fs->setFracturableObject(rb3);
    fs->solve({src});
#endif

    // add to world
    world->addRigidBody(rb1);
#ifdef TEST_SECOND_GROUND
    world->addRigidBody(rb4);
#endif
#ifdef TEST_SINGLE
    world->addRigidBody(rb2);
#endif
    pe::Array<pe_phys_object::RigidBody*> rbs;
    for (int i = 0; i < rb_num; i++) {
        auto rb = createRigidBody(pe::Vector3(0, 5 + i * 1.1, 0), pe::Vector3(1, 1, 1));
        rbs.push_back(rb);
        world->addRigidBody(rb);
    }
#ifdef TEST_FRAC
    for (auto rb : fs->getFragments()) {
        rbs.push_back(rb);
        world->addRigidBody(rb);
    }
#endif

    // add to viewer
    int id1 = pe_core::Viewer::addCube(pe::Vector3(20, 1, 20));
    pe_core::Viewer::updateCubeColor(id1, pe::Vector3(0.3, 0.3, 0.8));
    pe_core::Viewer::updateCubeTransform(id1, rb1->getTransform());
#ifdef TEST_SECOND_GROUND
    int id4 = pe_core::Viewer::addCube(pe::Vector3(10, 1, 10));
    pe_core::Viewer::updateCubeColor(id4, pe::Vector3(0.3, 0.3, 0.8));
    pe_core::Viewer::updateCubeTransform(id4, rb4->getTransform());
#endif
#ifdef TEST_SINGLE
    int id2 = pe_core::Viewer::addMesh(((pe_phys_shape::ConvexMeshShape*)(rb2->getCollisionShape()))->getMesh());
    pe_core::Viewer::updateMeshColor(id2, pe::Vector3(0.3, 0.8, 0.3));
    pe_core::Viewer::updateMeshTransform(id2, rb2->getTransform());
#endif
    pe::Array<int> ids;
    for (int i = 0; i < rb_num; i++) {
        int id = pe_core::Viewer::addCube(pe::Vector3(1, 1, 1));
        pe_core::Viewer::updateCubeColor(id, pe::Vector3(0.8, 0.3, 0.3));
        pe_core::Viewer::updateCubeTransform(id, rbs[i]->getTransform());
        ids.push_back(id);
    }
#ifdef TEST_FRAC
    for (auto rb : fs->getFragments()) {
        auto shape = (pe_phys_shape::ConvexMeshShape*)(rb->getCollisionShape());
        auto id = pe_core::Viewer::addMesh(shape->getMesh());
        pe_core::Viewer::updateMeshColor(id, pe::Vector3(0.3, 0.8, 0.3));
        pe_core::Viewer::updateMeshTransform(id, rb->getTransform());
        ids.push_back(id);
    }
#endif

    // main loop
    int frame = 0, th = 50000;
    while (true) {
        frame++;
        while (pe_core::Viewer::getKeyState('r') != 0) {
            if (pe_core::Viewer::getKeyState('q') == 0) goto ret;
        }
        auto t = COMMON_GetTickCount();
#   ifdef TEST_SINGLE
        pe_core::Viewer::updateCubeTransform(id2, rb2->getTransform());
        pe_core::Viewer::updateMeshTransform(id2, rb2->getTransform());
#   endif
#   ifdef TEST_SECOND_GROUND
        pe_core::Viewer::updateCubeTransform(id4, rb4->getTransform());
#   endif
        for (int i = 0; i < ids.size(); i++) {
            pe_core::Viewer::updateCubeTransform(ids[i], rbs[i]->getTransform());
            pe_core::Viewer::updateMeshTransform(ids[i], rbs[i]->getTransform());
        }
        world->step();
        if (++frame > th) while (pe_core::Viewer::getKeyState('r') != 1);
        COMMON_Sleep(10 - (int)(COMMON_GetTickCount() - t));
    }

    ret:
    pe_core::Viewer::close();
    delete world;
}

int main() {
    testWorld();
}