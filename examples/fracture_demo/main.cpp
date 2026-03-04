#include "interface/simulator.h"
#include "interface/viewer.h"
#include "physics/shape/box_shape.h"
#include "physics/shape/sphere_shape.h"
#include "physics/shape/convex_mesh_shape.h"
#include "physics/object/fracturable_object.h"
#include "physics/shape/capsule_shape.h"

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class FractureSimulator : public pe_interface::Simulator {
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
    }

    void step() override {
        /* This function is called before each simulation step */

        static int frame = 0;
        frame++;
        if (frame == 1 || frame == 201 || frame == 401) {
            int i = frame / 200;
            // add a fracturable box
            auto rb = createFracturableObject(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, i + 0.5, 0)),
                pe::Vector3(4, 1, 4), 1);
            _world.addRigidBody(rb);
            //std::cout << "Added fracturable object: " << rb->getGlobalId() << std::endl;

            pe_physics_fracture::FractureSource src;
            src.type = pe_physics_fracture::FractureType::Sphere;
            src.position = pe::Vector3(PE_R(1.5), PE_R(i + 0.5), PE_R(1.5));
            src.intensity = pe::Vector3(PE_R(1.5), PE_R(1.5), PE_R(1.5));
            _world.addFractureSource(src);
        }
    }

protected:
    static pe_physics_object::RigidBody* createCapsuleRigidBody(const pe::Transform& trans,
                                                                pe::Real radius, pe::Real height, pe::Real mass) {
        /* This function creates a capsule-shaped rigidbody */

        auto rb = new pe_physics_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_physics_shape::CapsuleShape(radius, height);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setAngularDamping(PE_R(0.8));
        return rb;
    }

    static pe_physics_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
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

    static pe_physics_object::RigidBody* createSphereRigidBody(const pe::Transform& trans,
                                                               pe::Real radius, pe::Real mass) {
        /* This function creates a sphere-shaped rigidbody */

        auto rb = new pe_physics_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_physics_shape::SphereShape(radius);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setAngularDamping(PE_R(0.8));
        return rb;
    }

    static pe_physics_object::FracturableObject* createFracturableObject(const pe::Transform& trans,
                                                                         const pe::Vector3& size, pe::Real th) {
        /* This function creates a fracturable object */

        auto rb = new pe_physics_object::FracturableObject();
        rb->setMass(1.0);
        auto shape = new pe_physics_shape::BoxShape(size);
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
