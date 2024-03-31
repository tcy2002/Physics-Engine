#include <iostream>
#include "intf/simulator.h"
#include "phys/object/rigidbody.h"
#include "phys/object/fracturable_object.h"
#include "phys/fracture/fracture_solver/fracture_solver.h"

//#define PE_SECOND_GROUND

class FractureSimulator : public pe_intf::Simulator<true> {
public:
    FractureSimulator() {}
    virtual ~FractureSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity
        _world.setGravity(pe::Vector3(0, -9.8, 0));

        // create a ground
        auto rb1 = createBoxRigidBody(pe::Vector3(0, -0.5, 0),
                                      pe::Vector3(1000, 1, 1000), 8);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1);

#   ifdef PE_SECOND_GROUND
        // create a second ground
        auto rb2 = createBoxRigidBody(pe::Vector3(0, 0.5, 0),
                                      pe::Vector3(15, 1, 15), 8);
        _world.addRigidBody(rb2);
#   endif

        // create a fracturable box and solve it
        auto rb3 = createFracturableObject(pe::Vector3(0, 5, 0),
                                           pe::Vector3(4, 4, 4), 1);
        auto fs = new pe_phys_fracture::FractureSolver();
        pe_phys_fracture::FractureSource src;
        src.type = pe_phys_fracture::FractureType::Sphere;
        src.position = pe::Vector3(1.5, 6.5, 1.5);
        src.intensity = pe::Vector3(0.5, 0.5, 0.5);
        fs->setFracturableObject(rb3);
        fs->solve({src});
        for (auto rb : fs->getFragments()) {
            _world.addRigidBody(rb);
        }

        // create some other dynamic objects
        pe::Array<pe_phys_object::RigidBody*> rbs;
        for (int i = 0; i < 99; i++) {
            pe_phys_object::RigidBody* rb;
            if (i % 3 == 0) {
                rb = createBoxRigidBody(pe::Vector3(0, 10 + i * 1.1, 0),
                                        pe::Vector3(1, 1, 1), 1.0);
            } else if (i % 3 == 1) {
                rb = createSphereRigidBody(pe::Vector3(0, 10 + i * 1.1, 0),
                                           0.5, 1.0);
            } else {
                rb = createCylinderRigidBody(pe::Vector3(0, 10 + i * 1.1, 0),
                                             0.5, 1.0, 1.0);
            }
            _world.addRigidBody(rb);
        }
    }

protected:
    static pe_phys_object::RigidBody* createBoxRigidBody(const pe::Vector3& pos,
                                                         const pe::Vector3& size, pe::Real mass) {
        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }

    static pe_phys_object::RigidBody* createSphereRigidBody(const pe::Vector3& pos,
                                                            pe::Real radius, pe::Real mass) {
        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::SphereShape(radius);
        rb->setCollisionShape(shape);
        rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }

    static pe_phys_object::RigidBody* createCylinderRigidBody(const pe::Vector3& pos,
                                                              pe::Real radius, pe::Real height, pe::Real mass) {
        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::CylinderShape(radius, height);
        rb->setCollisionShape(shape);
        rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }

    static pe_phys_object::FracturableObject* createFracturableObject(const pe::Vector3& pos,
                                                                      const pe::Vector3& size, pe::Real th) {
        auto rb = new pe_phys_object::FracturableObject();
        rb->setMass(1.0);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
        rb->setLocalInertia(shape->calcLocalInertia(1.0));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        rb->setThreshold(th);
        return rb;
    }
};

int main() {
    FractureSimulator simulator;
    simulator.run(0.01, 10000);
    return 0;
}