#include "intf/simulator.h"
#include "phys/object/fracturable_object.h"
#include "phys/fracture/fracture_solver/fracture_solver.h"

// true/false: simulate with/without viewer
// If using viewer, press `r` to start simulation
// See SimpleViewer/include/opengl_viewer.h to learn the view control
class FractureSimulator : public pe_intf::Simulator<true> {
public:
    FractureSimulator() {}
    virtual ~FractureSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, -9.8, 0));

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -0.5, 0)),
                                      pe::Vector3(1000, 1, 1000), 8);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add a fracturable box and solve the fracture result
        auto rb3 = createFracturableObject(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 2, 0)),
                                           pe::Vector3(4, 4, 4), 1);
        auto fs = new pe_phys_fracture::FractureSolver();
        pe_phys_fracture::FractureSource src;
        src.type = pe_phys_fracture::FractureType::Sphere;
        src.position = pe::Vector3(1.5, 3.5, 1.5);
        src.intensity = pe::Vector3(1.5, 1.5, 1.5);
        fs->setFracturableObject(rb3);
        fs->solve({src});
        for (auto rb : fs->getFragments()) {
            _world.addRigidBody(rb);
        }

        // add some other dynamic objects
        pe::Array<pe_phys_object::RigidBody*> rbs;
        for (int i = 0; i < 0; i++) {
            pe_phys_object::RigidBody* rb;
            if (i % 3 == 0) {
                rb = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 10 + i * 1.1, 0)),
                                        pe::Vector3(1, 1, 1), 1.0);
            } else if (i % 3 == 1) {
                rb = createSphereRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 10 + i * 1.1, 0)),
                                           0.5, 1.0);
            } else {
                rb = createCylinderRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 10 + i * 1.1, 0)),
                                             0.5, 1.0, 1.0);
            }
            _world.addRigidBody(rb);
        }
    }

protected:
    static pe_phys_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
                                                         const pe::Vector3& size, pe::Real mass) {
        /* This function creates a box-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setLocalInertia(shape->calcLocalInertia(mass)); // inertia tensor matrix
        rb->setFrictionCoeff(0.5); // friction coefficient
        rb->setRestitutionCoeff(0.5); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(0.8); // angular damping parameter (slows down the rotation speed)
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
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }

    static pe_phys_object::RigidBody* createCylinderRigidBody(const pe::Transform& trans,
                                                              pe::Real radius, pe::Real height, pe::Real mass) {
        /* This function creates a cylinder-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::CylinderShape(radius, height);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }

    static pe_phys_object::FracturableObject* createFracturableObject(const pe::Transform& trans,
                                                                      const pe::Vector3& size, pe::Real th) {
        /* This function creates a fracturable object */

        auto rb = new pe_phys_object::FracturableObject();
        rb->setMass(1.0);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setLocalInertia(shape->calcLocalInertia(1.0));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        rb->setThreshold(th);
        return rb;
    }
};

// Simulator class, Delta time, Max frame
PE_SIM_MAIN(FractureSimulator, 0.016, 1000000)