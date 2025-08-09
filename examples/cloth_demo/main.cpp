#include "intf/simulator.h"
#include "rigid/constraint/constraint_solver/sequential_impulse_solver.h"
#include "cloth/object/pbd_cloth.h"

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class ClothSimulator : public pe_intf::Simulator {
public:
    ClothSimulator() {}
    virtual ~ClothSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */
        use_gui = true;
        //max_frame = 500;
        //saving = true;

        auto solver = new pe_phys_constraint::SequentialImpulseSolver;
        solver->setIteration(10);
        _world.setConstraintSolver(solver);

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: outward screen)
        _world.setGravity(pe::Vector3(0, PE_R(-9.81), 0));
        //_world.setSleepLinVel2Threshold(PE_R(0.01)); // linear velocity threshold for sleep
        //_world.setSleepAngVel2Threshold(PE_R(0.01)); // angular velocity threshold for sleep
        //_world.setSleepTimeThreshold(PE_R(1.0));     // sleep time threshold

        auto pbd_cloth = new pe_phys_object::PBDCloth();
        pbd_cloth->loadFromObj(PE_DEMO_PATH "modified_cloth.obj");
        pbd_cloth->setFixedPlane(21);
        pbd_cloth->setEnableSelfCollision(true);
        pbd_cloth->setEnableBending(false);
        _world.addClothObject(pbd_cloth);
        std::cout << "cloth num: " << _world.getClothObjects().size() << std::endl;
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
PE_CUSTOM_MAIN(ClothSimulator, 100)
