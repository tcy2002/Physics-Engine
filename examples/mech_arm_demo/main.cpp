#include "intf/simulator.h"

class MechanicalArm {
protected:
    struct Component {
        pe::Transform local_transform;
        pe_phys_object::RigidBody* rigidbody;
        pe::Array<Component*> children;
    };

    Component *bottom;

    pe::Transform _transform = pe::Transform::Identity();

    const pe::Real _bottom_radius = PE_R(0.5);
    const pe::Real _bottom_height = PE_R(1.8);
    const pe::Real _bottom_neck_radius = PE_R(0.5);
    const pe::Real _bottom_neck_height = PE_R(1.6);
    const pe::Real _lower_arm_radius = PE_R(0.4);
    const pe::Real _lower_arm_height = PE_R(4);
    const pe::Real _middle_neck_radius = PE_R(0.4);
    const pe::Real _middle_neck_height = PE_R(1.8);
    const pe::Real _upper_arm_radius = PE_R(0.3);
    const pe::Real _upper_arm_height = PE_R(3.8);
    const pe::Real _top_neck_radius = PE_R(0.36);
    const pe::Real _top_neck_height = PE_R(1.2);
    const pe::Real _hand_radius = PE_R(0.36);
    const pe::Real _hand_height = PE_R(1.5);
    const pe::Real _palm_radius = PE_R(0.42);
    const pe::Vector3 _upper_finger_size = {PE_R(0.4), PE_R(1.3), PE_R(0.3)};
    const pe::Vector3 _lower_finger_size = {PE_R(0.4), PE_R(1.5), PE_R(0.3)};
    const pe::Real _knuckle_radius = PE_R(0.3);
    const pe::Real _finger_tip_radius = PE_R(0.3);

    pe::Transform _bottom_neck_transform;
    pe::Transform _lower_arm_transform;
    pe::Transform _middle_neck_transform;
    pe::Transform _upper_arm_transform;
    pe::Transform _top_neck_transform;
    pe::Transform _hand_transform;
    pe::Transform _palm_transform;
    pe::Transform _lower_finger_transform;
    pe::Transform _knuckle_transform;
    pe::Transform _upper_finger_transform;
    pe::Transform _finger_tip_transform;

    // pre-rotation
    pe::Real _bottom_rotation = 0; // 0 ~ 2pi
    pe::Real _bottom_neck_rotation = PE_PI / 2; // 0 ~ pi
    pe::Real _middle_neck_rotation = PE_PI * PE_R(0.75); // -pi/4 ~ 5pi/4
    pe::Real _top_neck_rotation = 0; // 0 ~ 3pi/4
    pe::Real _lower_finger_rotation = PE_PI / 4; // pi/9 ~ pi/3
    pe::Real _upper_finger_rotation = PE_PI / 2; // 0 ~ pi/2

    void updateKinematic(Component* component, const pe::Transform& transform) {
        if (component == nullptr) return;
        auto current_transform = transform * component->local_transform;
        component->rigidbody->setTransform(current_transform);
        for (auto child : component->children) {
            updateKinematic(child, current_transform);
        }
    }

    void initializeParams() {
        // arm
        _bottom_neck_transform.setRotation(-pe::Vector3::UnitZ(), PE_PI / 2);
        _bottom_neck_transform.setOrigin({_bottom_neck_height / 2, 0, 0});
        _lower_arm_transform.setRotation(-pe::Vector3::UnitX(), PE_PI / 2);
        _lower_arm_transform.setOrigin({0, _bottom_neck_height / 4, -_lower_arm_height / 2});
        _middle_neck_transform.setRotation(-pe::Vector3::UnitX(), PE_PI / 2);
        _middle_neck_transform.setOrigin({0, _lower_arm_height / 2, -_middle_neck_height / 5});
        _upper_arm_transform.setRotation(pe::Vector3::UnitZ(), PE_PI / 2);
        _upper_arm_transform.setOrigin({-_upper_arm_height / 2, _middle_neck_height / 4, 0});
        _top_neck_transform.setRotation(pe::Vector3::UnitZ(), PE_PI / 2);
        _top_neck_transform.setOrigin({-_middle_neck_height / 8, _upper_arm_height / 2, 0});
        _hand_transform.setRotation(pe::Vector3::UnitZ(), PE_PI / 2);
        _hand_transform.setOrigin({_hand_height / 8, _top_neck_height / 2, 0});

        // hand
        _palm_transform.setRotation(pe::Vector3::UnitX(), PE_PI);
        _palm_transform.setOrigin({0,-_hand_height / 2,0});
        _lower_finger_transform.setBasis(pe::Matrix3::Identity());
        _lower_finger_transform.setOrigin({0,_lower_finger_size.y() / 2,0});
        _knuckle_transform.setBasis(pe::Matrix3::Identity());
        _knuckle_transform.setOrigin({0,_lower_finger_size.y() / 2,0});
        _upper_finger_transform.setBasis(pe::Matrix3::Identity());
        _upper_finger_transform.setOrigin({0, _upper_finger_size.y() / 2, 0});
        _finger_tip_transform.setBasis(pe::Matrix3::Identity());
        _finger_tip_transform.setOrigin({0, _upper_finger_size.y() / 2, 0});
    }

    void updateRotation() {
        // arm
        pe::Transform tmp = pe::Transform::Identity();
        bottom->local_transform.setOrigin(pe::Vector3(0, _bottom_height / 2, 0));
        bottom->local_transform.setRotation(pe::Vector3::UnitY(), _bottom_rotation);
        auto bottom_neck = bottom->children[0];
        tmp.setRotation(pe::Vector3::UnitY(), _bottom_rotation);
        bottom_neck->local_transform = tmp * _bottom_neck_transform;
        auto lower_arm = bottom_neck->children[0];
        tmp.setRotation(pe::Vector3::UnitY(), _bottom_neck_rotation);
        lower_arm->local_transform = tmp * _lower_arm_transform;
        auto middle_neck = lower_arm->children[0];
        middle_neck->local_transform = _middle_neck_transform;
        auto upper_arm = middle_neck->children[0];
        tmp.setRotation(pe::Vector3::UnitY(), _middle_neck_rotation);
        upper_arm->local_transform = tmp * _upper_arm_transform;
        auto top_neck = upper_arm->children[0];
        top_neck->local_transform = _top_neck_transform;
        auto hand = top_neck->children[0];
        tmp.setRotation(-pe::Vector3::UnitY(), _top_neck_rotation);
        hand->local_transform = tmp * _hand_transform;

        // hand
        auto palm = hand->children[0];
        palm->local_transform = _palm_transform;
        pe::Transform tmp2;
        tmp.setRotation(pe::Vector3::UnitX(), _lower_finger_rotation);
        auto lower_finger1 = palm->children[0];
        lower_finger1->local_transform = tmp * _lower_finger_transform;
        auto lower_finger2 = palm->children[1];
        tmp2.setRotation(pe::Vector3::UnitY(), PE_PI / 2);
        lower_finger2->local_transform = tmp2 * tmp * _lower_finger_transform;
        auto lower_finger3 = palm->children[2];
        tmp2.setRotation(pe::Vector3::UnitY(), PE_PI);
        lower_finger3->local_transform = tmp2 * tmp * _lower_finger_transform;
        auto lower_finger4 = palm->children[3];
        tmp2.setRotation(pe::Vector3::UnitY(), -PE_PI / 2);
        lower_finger4->local_transform = tmp2 * tmp * _lower_finger_transform;
        auto knuckle1 = lower_finger1->children[0];
        knuckle1->local_transform = _knuckle_transform;
        auto knuckle2 = lower_finger2->children[0];
        knuckle2->local_transform = _knuckle_transform;
        auto knuckle3 = lower_finger3->children[0];
        knuckle3->local_transform = _knuckle_transform;
        auto knuckle4 = lower_finger4->children[0];
        knuckle4->local_transform = _knuckle_transform;
        tmp.setRotation(-pe::Vector3::UnitX(), _upper_finger_rotation);
        auto upper_finger1 = knuckle1->children[0];
        upper_finger1->local_transform = tmp * _upper_finger_transform;
        auto upper_finger2 = knuckle2->children[0];
        upper_finger2->local_transform = tmp * _upper_finger_transform;
        auto upper_finger3 = knuckle3->children[0];
        upper_finger3->local_transform = tmp * _upper_finger_transform;
        auto upper_finger4 = knuckle4->children[0];
        upper_finger4->local_transform = tmp * _upper_finger_transform;
        auto finger_tip1 = upper_finger1->children[0];
        finger_tip1->local_transform = _finger_tip_transform;
        auto finger_tip2 = upper_finger2->children[0];
        finger_tip2->local_transform = _finger_tip_transform;
        auto finger_tip3 = upper_finger3->children[0];
        finger_tip3->local_transform = _finger_tip_transform;
        auto finger_tip4 = upper_finger4->children[0];
        finger_tip4->local_transform = _finger_tip_transform;
    }

    void initializeArm(pe_intf::World* world) {
        // arm
        bottom = new Component();
        bottom->rigidbody = createCylinderRigidBody(pe::Transform::Identity(), _bottom_radius, _bottom_height, 1);
        bottom->rigidbody->setKinematic(true);
        bottom->rigidbody->setTag("color:0.3,0.3,0.8");
        world->addRigidBody(bottom->rigidbody);
        auto bottom_neck = new Component();
        bottom_neck->rigidbody = createCylinderRigidBody(pe::Transform::Identity(), _bottom_neck_radius, _bottom_neck_height, 1);
        bottom_neck->rigidbody->setKinematic(true);
        bottom_neck->rigidbody->setTag("color:0.3,0.3,0.8");
        world->addRigidBody(bottom_neck->rigidbody);
        bottom->children.push_back(bottom_neck);
        auto lower_arm = new Component();
        lower_arm->rigidbody = createCylinderRigidBody(pe::Transform::Identity(), _lower_arm_radius, _lower_arm_height, 1);
        lower_arm->rigidbody->setKinematic(true);
        lower_arm->rigidbody->setTag("color:0.3,0.3,0.8");
        world->addRigidBody(lower_arm->rigidbody);
        bottom_neck->children.push_back(lower_arm);
        auto middle_neck = new Component();
        middle_neck->rigidbody = createCylinderRigidBody(pe::Transform::Identity(), _middle_neck_radius, _middle_neck_height, 1);
        middle_neck->rigidbody->setKinematic(true);
        middle_neck->rigidbody->setTag("color:0.3,0.3,0.8");
        world->addRigidBody(middle_neck->rigidbody);
        lower_arm->children.push_back(middle_neck);
        auto upper_arm = new Component();
        upper_arm->rigidbody = createCylinderRigidBody(pe::Transform::Identity(), _upper_arm_radius, _upper_arm_height, 1);
        upper_arm->rigidbody->setKinematic(true);
        upper_arm->rigidbody->setTag("color:0.3,0.3,0.8");
        world->addRigidBody(upper_arm->rigidbody);
        middle_neck->children.push_back(upper_arm);
        auto top_neck = new Component();
        top_neck->rigidbody = createCylinderRigidBody(pe::Transform::Identity(), _top_neck_radius, _top_neck_height, 1);
        top_neck->rigidbody->setKinematic(true);
        top_neck->rigidbody->setTag("color:0.8,0.3,0.3");
        world->addRigidBody(top_neck->rigidbody);
        upper_arm->children.push_back(top_neck);
        auto hand = new Component();
        hand->rigidbody = createCylinderRigidBody(pe::Transform::Identity(), _hand_radius, _hand_height, 1);
        hand->rigidbody->setKinematic(true);
        hand->rigidbody->setTag("color:0.8,0.3,0.3");
        world->addRigidBody(hand->rigidbody);
        top_neck->children.push_back(hand);

        // hand
        auto palm = new Component();
        palm->rigidbody = createSphereRigidBody(pe::Transform::Identity(), _palm_radius, 1);
        palm->rigidbody->setKinematic(true);
        palm->rigidbody->setTag("color:0.8,0.8,0.3");
        world->addRigidBody(palm->rigidbody);
        hand->children.push_back(palm);
        auto lower_finger1 = new Component();
        lower_finger1->rigidbody = createBoxRigidBody(pe::Transform::Identity(), _lower_finger_size, 1);
        lower_finger1->rigidbody->setKinematic(true);
        lower_finger1->rigidbody->setTag("color:0.8,0.3,0.3");
        world->addRigidBody(lower_finger1->rigidbody);
        palm->children.push_back(lower_finger1);
        auto lower_finger2 = new Component();
        lower_finger2->rigidbody = createBoxRigidBody(pe::Transform::Identity(), _lower_finger_size, 1);
        lower_finger2->rigidbody->setKinematic(true);
        lower_finger2->rigidbody->setTag("color:0.8,0.3,0.3");
        world->addRigidBody(lower_finger2->rigidbody);
        palm->children.push_back(lower_finger2);
        auto lower_finger3 = new Component();
        lower_finger3->rigidbody = createBoxRigidBody(pe::Transform::Identity(), _lower_finger_size, 1);
        lower_finger3->rigidbody->setKinematic(true);
        lower_finger3->rigidbody->setTag("color:0.8,0.3,0.3");
        world->addRigidBody(lower_finger3->rigidbody);
        palm->children.push_back(lower_finger3);
        auto lower_finger4 = new Component();
        lower_finger4->rigidbody = createBoxRigidBody(pe::Transform::Identity(), _lower_finger_size, 1);
        lower_finger4->rigidbody->setKinematic(true);
        lower_finger4->rigidbody->setTag("color:0.8,0.3,0.3");
        world->addRigidBody(lower_finger4->rigidbody);
        palm->children.push_back(lower_finger4);
        auto knuckle1 = new Component();
        knuckle1->rigidbody = createSphereRigidBody(pe::Transform::Identity(), _knuckle_radius, 1);
        knuckle1->rigidbody->setKinematic(true);
        knuckle1->rigidbody->setTag("color:0.8,0.8,0.3");
        world->addRigidBody(knuckle1->rigidbody);
        lower_finger1->children.push_back(knuckle1);
        auto knuckle2 = new Component();
        knuckle2->rigidbody = createSphereRigidBody(pe::Transform::Identity(), _knuckle_radius, 1);
        knuckle2->rigidbody->setKinematic(true);
        knuckle2->rigidbody->setTag("color:0.8,0.8,0.3");
        world->addRigidBody(knuckle2->rigidbody);
        lower_finger2->children.push_back(knuckle2);
        auto knuckle3 = new Component();
        knuckle3->rigidbody = createSphereRigidBody(pe::Transform::Identity(), _knuckle_radius, 1);
        knuckle3->rigidbody->setKinematic(true);
        knuckle3->rigidbody->setTag("color:0.8,0.8,0.3");
        world->addRigidBody(knuckle3->rigidbody);
        lower_finger3->children.push_back(knuckle3);
        auto knuckle4 = new Component();
        knuckle4->rigidbody = createSphereRigidBody(pe::Transform::Identity(), _knuckle_radius, 1);
        knuckle4->rigidbody->setKinematic(true);
        knuckle4->rigidbody->setTag("color:0.8,0.8,0.3");
        world->addRigidBody(knuckle4->rigidbody);
        lower_finger4->children.push_back(knuckle4);
        auto upper_finger1 = new Component();
        upper_finger1->rigidbody = createBoxRigidBody(pe::Transform::Identity(), _upper_finger_size, 1);
        upper_finger1->rigidbody->setKinematic(true);
        upper_finger1->rigidbody->setTag("color:0.8,0.3,0.3");
        world->addRigidBody(upper_finger1->rigidbody);
        knuckle1->children.push_back(upper_finger1);
        auto upper_finger2 = new Component();
        upper_finger2->rigidbody = createBoxRigidBody(pe::Transform::Identity(), _upper_finger_size, 1);
        upper_finger2->rigidbody->setKinematic(true);
        upper_finger2->rigidbody->setTag("color:0.8,0.3,0.3");
        world->addRigidBody(upper_finger2->rigidbody);
        knuckle2->children.push_back(upper_finger2);
        auto upper_finger3 = new Component();
        upper_finger3->rigidbody = createBoxRigidBody(pe::Transform::Identity(), _upper_finger_size, 1);
        upper_finger3->rigidbody->setKinematic(true);
        upper_finger3->rigidbody->setTag("color:0.8,0.3,0.3");
        world->addRigidBody(upper_finger3->rigidbody);
        knuckle3->children.push_back(upper_finger3);
        auto upper_finger4 = new Component();
        upper_finger4->rigidbody = createBoxRigidBody(pe::Transform::Identity(), _upper_finger_size, 1);
        upper_finger4->rigidbody->setKinematic(true);
        upper_finger4->rigidbody->setTag("color:0.8,0.3,0.3");
        world->addRigidBody(upper_finger4->rigidbody);
        knuckle4->children.push_back(upper_finger4);
        auto finger_tip1 = new Component();
        finger_tip1->rigidbody = createSphereRigidBody(pe::Transform::Identity(), _finger_tip_radius, 1);
        finger_tip1->rigidbody->setKinematic(true);
        finger_tip1->rigidbody->setTag("color:0.8,0.8,0.3");
        world->addRigidBody(finger_tip1->rigidbody);
        upper_finger1->children.push_back(finger_tip1);
        auto finger_tip2 = new Component();
        finger_tip2->rigidbody = createSphereRigidBody(pe::Transform::Identity(), _finger_tip_radius, 1);
        finger_tip2->rigidbody->setKinematic(true);
        finger_tip2->rigidbody->setTag("color:0.8,0.8,0.3");
        world->addRigidBody(finger_tip2->rigidbody);
        upper_finger2->children.push_back(finger_tip2);
        auto finger_tip3 = new Component();
        finger_tip3->rigidbody = createSphereRigidBody(pe::Transform::Identity(), _finger_tip_radius, 1);
        finger_tip3->rigidbody->setKinematic(true);
        finger_tip3->rigidbody->setTag("color:0.8,0.8,0.3");
        world->addRigidBody(finger_tip3->rigidbody);
        upper_finger3->children.push_back(finger_tip3);
        auto finger_tip4 = new Component();
        finger_tip4->rigidbody = createSphereRigidBody(pe::Transform::Identity(), _finger_tip_radius, 1);
        finger_tip4->rigidbody->setKinematic(true);
        finger_tip4->rigidbody->setTag("color:0.8,0.8,0.3");
        world->addRigidBody(finger_tip4->rigidbody);
        upper_finger4->children.push_back(finger_tip4);
    }

public:
    explicit MechanicalArm(pe_intf::World* world) {
        initializeArm(world);
        initializeParams();
        updateRotation();
        updateKinematic(bottom, _transform);
    }

    ~MechanicalArm() {
        pe::Array<Component*> stack;
        stack.push_back(bottom);
        while (!stack.empty()) {
            auto component = stack.back();
            stack.pop_back();
            for (auto& child : component->children) {
                stack.push_back(child);
            }
            delete component;
        }
    }

    void setTransform(const pe::Transform& transform) {
        _transform = transform;
        updateKinematic(bottom, _transform);
    }

    void rotateBottom(pe::Real delta_angle) {
        _bottom_rotation += delta_angle;
        if (_bottom_rotation > PE_PI * 2) _bottom_rotation -= PE_PI * 2;
        if (_bottom_rotation < 0) _bottom_rotation += PE_PI * 2;
        updateRotation();
        updateKinematic(bottom, _transform);
    }

    void rotateBottomNeck(pe::Real delta_angle) {
        pe::Real new_rot = _bottom_neck_rotation + delta_angle;
        if (new_rot < PE_PI && new_rot > 0) _bottom_neck_rotation = new_rot;
        updateRotation();
        updateKinematic(bottom, _transform);
    }

    void rotateMiddleNeck(pe::Real delta_angle) {
        pe::Real new_rot = _middle_neck_rotation + delta_angle;
        if (new_rot < PE_PI * PE_R(1.25) && new_rot > PE_PI / 4) _middle_neck_rotation = new_rot;
        updateRotation();
        updateKinematic(bottom, _transform);
    }

    void rotateTopNeck(pe::Real delta_angle) {
        pe::Real new_rot = _top_neck_rotation + delta_angle;
        if (new_rot < PE_PI * PE_R(0.75) && new_rot > 0) _top_neck_rotation = new_rot;
        updateRotation();
        updateKinematic(bottom, _transform);
    }

    void rotatePalm(pe::Real delta_angle) {
        pe::Real new_rot = _lower_finger_rotation + delta_angle;
        if (new_rot < PE_PI / 3 && new_rot > PE_PI / 9) _lower_finger_rotation = new_rot;
        updateRotation();
        updateKinematic(bottom, _transform);
    }

    void rotateKnuckle(pe::Real delta_angle) {
        pe::Real new_rot = _upper_finger_rotation + delta_angle;
        if (new_rot < PE_PI / 2 && new_rot > 0) _upper_finger_rotation = new_rot;
        updateRotation();
        updateKinematic(bottom, _transform);
    }

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
};

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class MechArmSimulator : public pe_intf::Simulator {
protected:
    MechanicalArm* _arm;

public:
    MechArmSimulator() {}
    virtual ~MechArmSimulator() { delete _arm; }

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, PE_R(-9.8), 0));
        // _world.setSleepLinVel2Threshold(pe::Real(0.01)); // linear velocity threshold for sleep
        // _world.setSleepAngVel2Threshold(pe::Real(0.01)); // angular velocity threshold for sleep
        // _world.setSleepTimeThreshold(pe::Real(1.0));     // sleep time threshold

        // add a ground
        auto rb = MechanicalArm::createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, -5, 0)),
                                      pe::Vector3(50, 10, 50), 10000);
        rb->setKinematic(true);
        _world.addRigidBody(rb); // a rigidbody must be added into the _world to perform physical effects

        // add a mechanical arm
        _arm = new MechanicalArm(&_world);

        // add container 1
        createContainer({7, 0, 0}, 6, 1, 0.3);
        // add container 2
        createContainer({-7, 0, 0}, 6, 1, 0.3);

        // add some items
        rb = MechanicalArm::createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(8, 0.8, 1)),
                                          pe::Vector3(1, 1, 1), PE_R(0.1));
        _world.addRigidBody(rb);
        rb = MechanicalArm::createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(8, 0.8, -1)),
                                          pe::Vector3(1, 1, 1), PE_R(0.1));
        _world.addRigidBody(rb);
        rb = MechanicalArm::createSphereRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(6, 0.8, 1)),
                                            PE_R(0.5), PE_R(0.1));
        _world.addRigidBody(rb);
        rb = MechanicalArm::createSphereRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(6, 0.8, -1)),
                                            PE_R(0.5), PE_R(0.1));
        _world.addRigidBody(rb);
    }

    void step() override {
        /* Called every frame to update the physics world */
        static pe::Real bottom_speed = PE_R(0.2);
        static pe::Real bottom_neck_speed = PE_R(0.3);
        static pe::Real middle_neck_speed = PE_R(0.3);
        static pe::Real top_neck_speed = PE_R(0.3);
        static int frame = 0;

        if (frame < 98) {
            _arm->rotateBottom(-PE_PI / 400);
        }
        if (frame >= 20 && frame < 80) {
            _arm->rotateBottomNeck( -PE_PI / 400);
        }
        if (frame >= 50 && frame < 100) {
            _arm->rotateMiddleNeck(PE_PI / 600);
        }
        if (frame >= 45 && frame < 140) {
            _arm->rotateTopNeck(PE_PI / 200);
        }
        if (frame >= 80 && frame < 160) {
            _arm->rotatePalm(PE_PI / 800);
            _arm->rotateKnuckle(-PE_PI / 800);
        }
        if (frame >= 160 && frame < 220) {
            _arm->rotateBottomNeck(-PE_PI / 1600);
            _arm->rotateMiddleNeck(PE_PI / 800);
            _arm->rotateTopNeck(-PE_PI / 1000);
        }
        if (frame >= 220 && frame < 300) {
            _arm->rotatePalm(-PE_PI / 1200);
            _arm->rotateKnuckle(PE_PI / 1200);
        }
        if (frame >= 340 && frame < 440) {
            _arm->rotateBottomNeck(PE_PI / 800);
        }
        if (frame >= 400 && frame < 1000) {
            _arm->rotateBottom(PE_PI / 1200);
        }
        if (frame >= 960 && frame < 1060) {
            _arm->rotateBottomNeck(-PE_PI / 800);
        }
        if (frame >= 1100 && frame < 1180) {
            _arm->rotatePalm(PE_PI / 1200);
            _arm->rotateKnuckle(-PE_PI / 1200);
        }
        if (frame >= 1220 && frame < 1320) {
            _arm->rotateBottomNeck(PE_PI / 800);
        }
        if (frame >= 1280 && frame < 1680) {
            _arm->rotateBottom(-PE_PI / 1200);
        }

        // update the arm
        if (pe_intf::Viewer::getKeyState('g') == 0) {
            _arm->rotateBottom(_world.getDt() * bottom_speed);
        } else if (pe_intf::Viewer::getKeyState('l') == 0) {
            _arm->rotateBottom(-_world.getDt() * bottom_speed);
        } else if (pe_intf::Viewer::getKeyState('y') == 0) {
            _arm->rotateBottomNeck(_world.getDt() * bottom_neck_speed);
        } else if (pe_intf::Viewer::getKeyState('h') == 0) {
            _arm->rotateBottomNeck(-_world.getDt() * bottom_neck_speed);
        } else if (pe_intf::Viewer::getKeyState('u') == 0) {
            _arm->rotateMiddleNeck(-_world.getDt() * middle_neck_speed);
        } else if (pe_intf::Viewer::getKeyState('j') == 0) {
            _arm->rotateMiddleNeck(_world.getDt() * middle_neck_speed);
        } else if (pe_intf::Viewer::getKeyState('i') == 0) {
            _arm->rotateTopNeck(-_world.getDt() * top_neck_speed);
        } else if (pe_intf::Viewer::getKeyState('k') == 0) {
            _arm->rotateTopNeck(_world.getDt() * top_neck_speed);
        } else if (pe_intf::Viewer::getKeyState('m') == 0) {
            _arm->rotatePalm(-_world.getDt() * top_neck_speed);
        } else if (pe_intf::Viewer::getKeyState(',') == 0) {
            _arm->rotatePalm(_world.getDt() * top_neck_speed);
        } else if (pe_intf::Viewer::getKeyState('.') == 0) {
            _arm->rotateKnuckle(_world.getDt() * top_neck_speed);
        } else if (pe_intf::Viewer::getKeyState('/') == 0) {
            _arm->rotateKnuckle(-_world.getDt() * top_neck_speed);
        }

        frame++;
    }

    void createContainer(const pe::Vector3& origin, pe::Real w, pe::Real h, pe::Real t) {
        auto rb = MechanicalArm::createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), {origin.x(), origin.y() +t/2, origin.z() }),
            pe::Vector3(w, t, w), 1);
        rb->setKinematic(true);
        rb->setTag("color:0.7,0.5,0.3");
        _world.addRigidBody(rb);
        rb = MechanicalArm::createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), {origin.x() +t/2-w/2, origin.y() +t/2+h/2, origin.z() }),
            pe::Vector3(t, h - t, w), 1);
        rb->setKinematic(true);
        _world.addRigidBody(rb);
        rb->setTag("color:0.7,0.5,0.3");
        rb = MechanicalArm::createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), {origin.x() -t/2+w/2, origin.y() +t/2+h/2, origin.z() }),
            pe::Vector3(t, h - t, w), 1);
        rb->setKinematic(true);
        _world.addRigidBody(rb);
        rb->setTag("color:0.7,0.5,0.3");
        rb = MechanicalArm::createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), {origin.x(), origin.y() +t/2+h/2, origin.z() + t / 2 - w / 2}),
            pe::Vector3(w, h - t, t), 1);
        rb->setKinematic(true);
        _world.addRigidBody(rb);
        rb->setTag("color:0.7,0.5,0.3");
        rb = MechanicalArm::createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), {origin.x(), origin.y() +t/2+h/2, origin.z() -t/2+w/2}),
            pe::Vector3(w, h - 0.3, 0.3), 1);
        rb->setKinematic(true);
        rb->setTag("color:0.7,0.5,0.3");
        _world.addRigidBody(rb);
    }
};

// Simulator class, Target frame rate
PE_CUSTOM_MAIN(MechArmSimulator, 100)
