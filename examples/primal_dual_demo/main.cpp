#include "intf/simulator.h"

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class PrimalDualSimulator : public pe_intf::Simulator {
public:
    PrimalDualSimulator() {}
    virtual ~PrimalDualSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */
        use_gui = true;

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: outward screen)
        _world.setGravity(pe::Vector3(0, -9.81, 0));
        // _world.setSleepLinVel2Threshold(R(0.01)); // linear velocity threshold for sleep
        // _world.setSleepAngVel2Threshold(R(0.01)); // angular velocity threshold for sleep
        // _world.setSleepTimeThreshold(R(1.0));     // sleep time threshold

        // add a ground
        auto rb = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(),
                                                   pe::Vector3(0, -1, 0)),
                                     pe::Vector3(2, 2, 2), 4);
        rb->setKinematic(true);
        _world.addRigidBody(rb); // a rigidbody must be added into the _world to perform physical effects

        /*rb = createBoxRigidBody(pe::Transform(Eigen::AngleAxis<pe::Real>(PE_PI / 5, pe::Vector3::UnitZ()).toRotationMatrix(),
            pe::Vector3(-0.1, 3, -0.1)),
            pe::Vector3(1, 1, 1), 1);
        _world.addRigidBody(rb);

        pe::Matrix3 mat;
        pe::Vector3 vec;
        mat << 0.985692, -0.168557, 0, 0.168557, 0.985692, 0, 0, 0, 1;
        vec << 0.252675, 1, -0.1;
        rb = createBoxRigidBody(pe::Transform(mat, vec), pe::Vector3(1, 1, 1), 1);*/
        
        _world.addRigidBody(rb);

        // cube tower
        addPyramidCubes();
        //addUniformCubes();
    }

    void step() override {
        //PE_LOG_DEBUG << _world.getRigidBodies()[1]->getTransform() << std::endl;
    }

    void addPyramidCubes() {
        // add box1
        auto rb = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(),
                                                   pe::Vector3(0, 1.116, 0)),
                                pe::Vector3(0.5, 0.5, 0.5), 1);
        _world.addRigidBody(rb);

        // add box2
        rb = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(),
                                              pe::Vector3(0, 1.752, 0)),
                                pe::Vector3(0.7071, 0.7071, 0.7071), 4.243);
        _world.addRigidBody(rb);

        // add box3
        rb = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(),
                                              pe::Vector3(0, 2.635, 0)),
                                pe::Vector3(1, 1, 1), 9);
        _world.addRigidBody(rb);

        // add box4
        rb = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(),
                                              pe::Vector3(0, 3.91, 0)),
                                pe::Vector3(1.4142, 1.4142, 1.4142), 76.368);
        _world.addRigidBody(rb);

        // add box5
        rb = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(),
                                              pe::Vector3(0, 5.674, 0)),
                                pe::Vector3(2, 2, 2), 648);
        _world.addRigidBody(rb);
    }

    void addUniformCubes() {
        for (int i = 0; i < 5; i++) {
            auto rb = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(),
                                                       pe::Vector3(0, 2.6 + 1.2 * i, 0)),
                                        pe::Vector3(1.2, 1.2, 1.2), 1);
            _world.addRigidBody(rb);
        }
    }

protected:
    static pe_phys_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
                                                         const pe::Vector3& size, pe::Real mass) {
        /* This function creates a box-shaped rigid body */

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
};

// Simulator class, Target frame rate
PE_CUSTOM_MAIN(PrimalDualSimulator, 100)
