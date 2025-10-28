#include "intf/simulator.h"

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class FractureSimulator : public pe_intf::Simulator {
public:
    FractureSimulator() {}
    virtual ~FractureSimulator() {}

    void step() override {
        // auto& rbs = _world.getRigidBodies();
        // for (auto rb : rbs) {
        //     if (!rb->isSleep()) {
        //         std::cout << rb->getGlobalId() << " " << rb->getLinearVelocity() << std::endl;
        //     }
        // }
    }

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, R(-9.8), 0));
        _world.setSleepLinVel2Threshold(R(0.01)); // linear velocity threshold for sleep
        _world.setSleepAngVel2Threshold(R(0.01)); // angular velocity threshold for sleep
        _world.setSleepTimeThreshold(R(1.0));     // sleep time threshold

        // add a ground
        auto ground = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -5, 0)),
                                      pe::Vector3(100, 10, 100), 1000);
        ground->setKinematic(true);
        _world.addRigidBody(ground); // a rigidbody must be added into the _world to perform physical effects

        // add a ceiling
        auto ceiling = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 55, 0)),
                                      pe::Vector3(100, 10, 100), 1000);
        ceiling->setKinematic(true);
        _world.addRigidBody(ceiling);

        // add a wall
        auto wall1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(-55, 25, 0)),
                                      pe::Vector3(10, 60, 100), 1000);
        wall1->setKinematic(true);
        _world.addRigidBody(wall1);

        // add a wall
        auto wall2 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(55, 25, 0)),
                                      pe::Vector3(10, 60, 100), 1000);
        wall2->setKinematic(true);
        _world.addRigidBody(wall2);

        // add a wall
        auto wall3 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 25, -55)),
                                      pe::Vector3(100, 60, 10), 1000);
        wall3->setKinematic(true);
        _world.addRigidBody(wall3);

        // add a wall
        auto wall4 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 25, 55)),
                                      pe::Vector3(100, 60, 10), 1000);
        wall4->setKinematic(true);
        _world.addRigidBody(wall4);

        // pe::Mesh mesh;
        // pe::Mesh::loadFromObj("D:\\ClionProjects\\Physics-Engine-clean\\Physics-Engine-84da628\\test.obj", mesh, pe::Vector3::ones());
        // auto shape = new pe_phys_shape::ConvexMeshShape();
        // auto offset = shape->setMesh(mesh);
        // pe::Transform offsetTrans(pe::Matrix3::identity(), offset);
        // pe::Transform trans(pe::Matrix3::fromRotation(pe::Vector3::forward(), -PE_PI / 3.8), pe::Vector3(0, 1.2, 0));
        // // pe::Transform trans(pe::Matrix3(1, -0, 0, 0, 3.96461e-13, 1, -0, -1, 3.96461e-13),
        // //     pe::Vector3(-0.812162, 0.777112, -0.516398));
        // // pe::Transform trans(pe::Matrix3(1, -0, 0, 0, 3.96461e-13, 1, -0, -1, 3.96461e-13),
        // //     pe::Vector3(-0.812162, 0.777112, -0.516398));
        // auto rb2 = new pe_phys_object::RigidBody();
        // rb2->setMass(1.0);
        // rb2->setCollisionShape(shape);
        // rb2->setTransform(trans * offsetTrans);
        // _world.addRigidBody(rb2);

        for (int i = 0; i < 10; i++) {
            // add a fracturable box
            auto rb = createFracturableObject(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, i + 0.5, 0)),
                                               pe::Vector3(4, 1, 4), 1);
            _world.addRigidBody(rb);

            pe_phys_fracture::FractureSource src;
            src.type = pe_phys_fracture::FractureType::Sphere;
            src.position = pe::Vector3(R(1.5), R(i + 0.5), R(1.5));
            src.intensity = pe::Vector3(R(1.5), R(1.5), R(1.5));
            _world.addFractureSource(src);
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
        rb->setFrictionCoeff(R(0.5)); // friction coefficient
        rb->setRestitutionCoeff(R(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(R(0.8)); // angular damping parameter (slows down the rotation speed)
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
        rb->setFrictionCoeff(R(0.5));
        rb->setRestitutionCoeff(R(0.5));
        rb->setAngularDamping(R(0.8));
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
        rb->setFrictionCoeff(R(0.5));
        rb->setRestitutionCoeff(R(0.5));
        rb->setAngularDamping(R(0.8));
        rb->setThreshold(th);
        return rb;
    }
};

// Simulator class, Target frame rate
PE_CUSTOM_MAIN(FractureSimulator, 100)
