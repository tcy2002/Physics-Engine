#include "intf/simulator.h"

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class FractureSimulator : public pe_intf::Simulator {
public:
    FractureSimulator() {}
    virtual ~FractureSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, PE_R(-9.8), 0));
        _world.setSleepLinVel2Threshold(PE_R(0.01)); // linear velocity threshold for sleep
        _world.setSleepAngVel2Threshold(PE_R(0.01)); // angular velocity threshold for sleep
        _world.setSleepTimeThreshold(PE_R(1.0));     // sleep time threshold

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, -5, 0)),
                                      pe::Vector3(30, 10, 30), 10000);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add a fracturable box
        auto rb3 = createFracturableObject(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, 2, 0)),
                                           pe::Vector3(4, 4, 4), 1);
        _world.addRigidBody(rb3);
        pe_phys_fracture::FractureSource src;
        src.type = pe_phys_fracture::FractureType::Sphere;
        src.position = pe::Vector3(PE_R(1.5), PE_R(3.5), PE_R(1.5));
        src.intensity = pe::Vector3(PE_R(1.5), PE_R(1.5), PE_R(1.5));
        _world.addFractureSource(src);

        // add some other dynamic objects
        pe::Array<pe_phys_object::RigidBody*> rbs;
        for (int i = 0; i < 30; i++) {
            pe_phys_object::RigidBody* rb;
            if (i % 3 == 0) {
                rb = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, 6 + i * PE_R(1.1), 0)),
                                        pe::Vector3(1, 1, 1), PE_R(1.0));
            } else if (i % 3 == 1) {
                rb = createSphereRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, 6 + i * PE_R(1.1), 0)),
                    PE_R(0.5), PE_R(1.0));
            } else {
                rb = createCylinderRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, 6 + i * PE_R(1.1), 0)),
                    PE_R(0.45), PE_R(1.0), PE_R(1.0));
            }
            _world.addRigidBody(rb);
        }

        //saveScene();
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
        rb->setFrictionCoeff(PE_R(0.5)); // friction coefficient
        rb->setRestitutionCoeff(PE_R(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(PE_R(0.8)); // angular damping parameter (slows down the rotation speed)
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
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setAngularDamping(PE_R(0.8));
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
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setAngularDamping(PE_R(0.8));
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
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setAngularDamping(PE_R(0.8));
        rb->setThreshold(th);
        return rb;
    }
};

// Simulator class, Target frame rate
PE_CUSTOM_MAIN(FractureSimulator, 100)
