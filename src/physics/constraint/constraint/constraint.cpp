#include "constraint.h"

namespace pe_physics_constraint {

std::atomic<uint32_t> Constraint::_globalIdCounter(0);

void Constraint::setObjectA(pe_physics_object::RigidBody *obj) {
    _object_a = obj == nullptr ? getStaticBody() : obj;
}

void Constraint::setObjectB(pe_physics_object::RigidBody *obj) {
    _object_b = obj == nullptr ? getStaticBody() : obj;
}

Constraint::Constraint(): _global_id(++_globalIdCounter) {}

void Constraint::getSkewSymmetricMatrix(const pe::Vector3& v, pe::Matrix3& m) {
    m[0][0] = 0; m[0][1] = -v.z; m[0][2] = v.y;
    m[1][0] = v.z; m[1][1] = 0; m[1][2] = -v.x;
    m[2][0] = -v.y; m[2][1] = v.x; m[2][2] = 0;
}

pe_physics_object::RigidBody* Constraint::getStaticBody() {
    static pe_physics_object::RigidBody static_body;
    static_body.setKinematic(true);
    return &static_body;
}

} // namespace pe_physics_constraint
