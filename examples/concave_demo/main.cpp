#include "intf/simulator.h"

// pe_intf::UseViewer::True/False: simulate with/without viewer
// If using viewer, press `x` to start simulation
// See SimpleViewer/include/opengl_viewer.h to learn the view control
class ConcaveSimulator : public pe_intf::Simulator {
public:
    ConcaveSimulator() {}
    virtual ~ConcaveSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, pe::Real(-9.8), 0));
        _world.setSleepLinVel2Threshold(pe::Real(0.01)); // linear velocity threshold for sleep
        _world.setSleepAngVel2Threshold(pe::Real(0.01)); // angular velocity threshold for sleep
        _world.setSleepTimeThreshold(pe::Real(1.0));     // sleep time threshold

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -5, 0)),
                                      pe::Vector3(30, 10, 30), 10000);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add a concave rigidbody (stanford dragon)
        // auto rb = createConcaveRigidBody(CONCAVE_DEMO_SOURCE_DIR "/dragon.obj",
            // pe::Transform(pe::Matrix3::identity(), pe::Vector3(1, 5, 0)), 100, 6);
        // _world.addRigidBody(rb);

        // add some concave rigidbodies (stanford bunny)
        int c = 1;
        int m = std::round((c - 1) / 2.0 * 10);
        for (int i = 0; i < c; i++) {
            for (int j = 0; j < c; j++) {
                auto rb = createConcaveRigidBody(CONCAVE_DEMO_SOURCE_DIR "/bunny.obj",
                pe::Transform(pe::Matrix3::identity(), pe::Vector3(i * 10 - m, 1, j * 10 - m)), 100, 3);
                _world.addRigidBody(rb);
            }
        }

        c = 15;
        m = c / 2;
        int h = 3;
        for (int i = 0; i < c; i++) {
            for (int j = 0; j < c; j++) {
                for (int k = 0; k < h; k++) {
                    pe_phys_object::RigidBody* rb = nullptr;
                    if ((i + j + k) % 3 == 0) {
                        rb = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(i - m, 12 + k, j - m)),
                        pe::Vector3(0.8, 0.8, 0.8), 1);
                    } else {
                        rb = createSphereRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(i - m, 12 + k, j - m)),
                        0.4, 1);
                    }
                    _world.addRigidBody(rb);
                }
            }
        }

        //saveScene("");
    }

protected:
    static pe_phys_object::RigidBody* createConcaveRigidBody(const std::string& obj_path, const pe::Transform& trans, pe::Real mass, pe::Real size) {
        static pe::Mesh mesh;
        if (mesh.empty())
            pe::Mesh::loadFromObj(obj_path, mesh, pe::Vector3(size, size, size));
        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::ConcaveMeshShape();
        shape->setMeshPath("./obj/bunny.obj");
        shape->setScale(pe::Vector3(3, 3, 3));
        shape->setMesh(mesh);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(pe::Real(0.5));
        rb->setRestitutionCoeff(pe::Real(0.5));
        rb->setKinematic(true);
        return rb;
    }

    static pe_phys_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
                                                         const pe::Vector3& size, pe::Real mass) {
        /* This function creates a box-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(pe::Real(0.5)); // friction coefficient
        rb->setRestitutionCoeff(pe::Real(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(pe::Real(0.8)); // angular damping parameter (slows down the rotation speed)
        return rb;
    }

    static pe_phys_object::RigidBody* createSphereRigidBody(const pe::Transform& trans,
                                                            pe::Real radius, pe::Real mass) {
        /* This function creates a sphere-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::SphereShape(radius);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(pe::Real(0.5));
        rb->setRestitutionCoeff(pe::Real(0.5));
        rb->setAngularDamping(pe::Real(0.8));
        return rb;
    }
};

// Simulator class, Delta time, Max frame
PE_CUSTOM_MAIN(ConcaveSimulator, 60)
