#include "intf/simulator.h"
#include "phys/constraint/constraint_solver/sequential_impulse_solver.h"
#include <random>

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class ConcaveSimulator : public pe_intf::Simulator {
public:
    ConcaveSimulator() {}
    virtual ~ConcaveSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */
        auto solver = new pe_phys_constraint::SequentialImpulseSolver;
        solver->setIteration(30);
        _world.setConstraintSolver(solver);

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, PE_R(-9.8), 0));
        // _world.setSleepLinVel2Threshold(R(0.01)); // linear velocity threshold for sleep
        // _world.setSleepAngVel2Threshold(R(0.01)); // angular velocity threshold for sleep
        // _world.setSleepTimeThreshold(R(1.0));     // sleep time threshold

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, -0.5, 0)),
                                      pe::Vector3(30, 1, 30), 10000, 0.3, 0.5);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add a concave rigidbody (stanford dragon)
        // auto rb = createConcaveRigidBody(CONCAVE_DEMO_SOURCE_DIR "/dragon.obj",
            // pe::Transform(pe::Matrix3::identity(), pe::Vector3(1, 5, 0)), 100, 6);
        // _world.addRigidBody(rb);

        // add some concave rigidbodies (stanford bunny)
        int c = 1;
        int m = (c - 1) * 5;
        for (int i = 0; i < c; i++) {
            for (int j = 0; j < c; j++) {
                auto rb = createConcaveRigidBody(CONCAVE_DEMO_SOURCE_DIR "/bunny.obj",
                pe::Transform(pe::Matrix3::Identity(), pe::Vector3(i * 10 - m, 1, j * 10 - m)), 100, 3);
                _world.addRigidBody(rb);
            }
        }

        c = 10;
        m = c / 2;
        int h = 3;
        for (int i = 0; i < c; i++) {
            for (int j = 0; j < c; j++) {
                for (int k = 0; k < h; k++) {
                    pe_phys_object::RigidBody* rb = nullptr;
                    if ((i + j + k) % 3 == 0) {
                        // ice density: 0.9 g/cm^3
                        rb = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(i - m + random(0.001), 12 + k, j - m + random(0.001))),
                        pe::Vector3(PE_R(0.8), PE_R(0.8), PE_R(0.8)), 0.4608, 0.02, 0.5);
                    } else if ((i + j + k) % 3 == 1) {
                        // metal density: 7.8 g/cm^3
                        rb = createSphereRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(i - m + random(0.001), 12 + k, j - m + random(0.001))),
                        PE_R(0.4), 2.091, 0.5, 0.55);
                    } else {
                        // wood density: 0.6 g/cm^3
                        rb = createCylinderRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(i - m + random(0.001), 12 + k, j - m + random(0.001))),
                            PE_R(0.3), 1.0, 0.1696, 0.8, 0.6);
                    }
                    _world.addRigidBody(rb);
                }
            }
        }

        //saveScene("");
    }

protected:
    static pe::Real random(pe::Real Scalar) {
        static std::default_random_engine e(COMMON_GetTickCount());
        static std::uniform_real_distribution<pe::Real> r(-0.5, 0.5);
        return Scalar * r(e);
    }
    static pe_phys_object::RigidBody* createConcaveRigidBody(const std::string& obj_path, const pe::Transform& trans, pe::Real mass, pe::Real size) {
        static pe::Mesh mesh;
        if (mesh.empty())
            pe::Mesh::loadFromObj(obj_path, mesh, pe::Vector3(size, size, size));
        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::ConcaveMeshShape();
        shape->setMeshPath(obj_path);
        shape->setScale(pe::Vector3(3, 3, 3));
        shape->setMesh(mesh);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setKinematic(true);
        return rb;
    }

    static pe_phys_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
                                                         const pe::Vector3& size, pe::Real mass, pe::Real friction, pe::Real restitution) {
        /* This function creates a box-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(friction); // friction coefficient
        rb->setRestitutionCoeff(restitution); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(0); // angular damping parameter (slows down the rotation speed)
        return rb;
    }

    static pe_phys_object::RigidBody* createSphereRigidBody(const pe::Transform& trans,
                                                            pe::Real radius, pe::Real mass, pe::Real friction, pe::Real restitution) {
        /* This function creates a sphere-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::SphereShape(radius);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(friction);
        rb->setRestitutionCoeff(restitution);
        rb->setAngularDamping(0.6);
        return rb;
    }

    static pe_phys_object::RigidBody* createCylinderRigidBody(const pe::Transform& trans,
                                                              pe::Real radius, pe::Real height, pe::Real mass, pe::Real friction, pe::Real restitution) {
        /* This function creates a cylinder-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::CylinderShape(radius, height);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(friction);
        rb->setRestitutionCoeff(restitution);
        rb->setAngularDamping(0.4);
        return rb;
    }
};

// Simulator class, Target frame rate
PE_CUSTOM_MAIN(ConcaveSimulator, 200)
