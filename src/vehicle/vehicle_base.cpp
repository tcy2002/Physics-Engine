#include "vehicle_base.h"

namespace pe_vehicle {

VehicleBase::~VehicleBase() {
    for (auto wheel : _wheels) {
        delete wheel;
    }
    for (auto suspension : _suspensions) {
        delete suspension;
    }
}

void VehicleBase::setWheelInfo(int index, AxleType type, bool motor, pe::Real radius, pe::Real mass, pe::Real friction, pe::Real anchor_offset_y,
                               pe::Real anchor_offset_z, pe::Real suspension_rest_length, pe::Real suspension_stiffness, pe::Real suspension_damping) {
    if (index < 0 || index >= _wheel_count_per_side * 2) {
        throw std::out_of_range("Invalid wheel index in VehicleBase::setWheelInfo: " + std::to_string(index));
    }
    if (_wheel_info.empty()) _wheel_info.resize(_wheel_count_per_side * 2);
    _wheel_info[index].type = type;
    _wheel_info[index].motor = motor;
    _wheel_info[index].radius = radius;
    _wheel_info[index].mass = mass;
    _wheel_info[index].friction = friction;
    _wheel_info[index].anchor_offset_y = anchor_offset_y;
    _wheel_info[index].anchor_offset_z = anchor_offset_z;
    _wheel_info[index].suspension_rest_length = suspension_rest_length;
    _wheel_info[index].suspension_stiffness = suspension_stiffness;
    _wheel_info[index].suspension_damping = suspension_damping;
}

void VehicleBase::setSteerAngle(int index, pe::Real angle) {
    if (index < 0 || index >= PE_I(_wheels.size())) {
        throw std::out_of_range("Invalid wheel index in VehicleBase::setSteerAngle: " + std::to_string(index));
    }
    // only front wheels can steer
    if (_wheel_info[index].type != AxleType::AT_FRONT) return;
    _suspensions[index]->setSteerAngle(angle - PE_PI / PE_R(2.0));
}

void VehicleBase::init(pe_interface::World* phys_world) {
    if (_chassis) {
        _chassis->init(phys_world);
    }
    if (_engine) {
        // Engine is not a physical object
    }

    if (_wheel_info.empty() || !_chassis) return;

    const pe::Real wheel_gap = _wheel_count_per_side > 1 ? (_wheel_region_length / PE_R(_wheel_count_per_side - 1)) : PE_R(0.0);
    const pe::Real half_wheel_region_width = _wheel_region_width / PE_R(2.0);
    const pe::Real half_wheel_region_length = _wheel_region_length / PE_R(2.0);
    const pe::Transform chassis_transform = _chassis->getTransform();
    for (int i = 0; i < _wheel_count_per_side * 2; i++) {
        auto& wi = _wheel_info[i];
        if (wi.type == AxleType::AT_NONE) continue; // ignore the wheel not properly set
        const pe::Real x = (i < _wheel_count_per_side ? -half_wheel_region_width : half_wheel_region_width) + _wheel_region_offset.x;
        const pe::Real y = _wheel_region_offset.y;
        const pe::Real z = (_wheel_count_per_side > 1 ? (-half_wheel_region_length + PE_R(i % _wheel_count_per_side) * wheel_gap) : PE_R(0.0)) + _wheel_region_offset.z;
        auto* wheel = new Wheel(WheelType::WT_Sphere, wi.radius, _wheel_width, wi.mass, wi.friction);
        const pe::Transform wheel_local_transform = pe::Transform(pe::Matrix3::identity(), pe::Vector3(x, y + wi.anchor_offset_y - wi.suspension_rest_length, z + wi.anchor_offset_z));
        wheel->setTransform(chassis_transform * wheel_local_transform);
        wheel->getBody()->addIgnoreCollisionId(_chassis->getBasePart().body->getGlobalId());
        _wheels.push_back(wheel);
        auto* suspension = new Suspension(_chassis, wheel, wi.type, wi.suspension_rest_length, wi.suspension_stiffness,
                                          wi.suspension_damping, pe::Vector3(x, y + wi.anchor_offset_y, z + wi.anchor_offset_z));
        _suspensions.push_back(suspension);
    }

    for (auto& wheel : _wheels) {
        wheel->init(phys_world);
    }
    for (auto& suspension : _suspensions) {
        suspension->init(phys_world);
    }
}

void VehicleBase::step(pe::Real dt) {
    if (_chassis) {
        _chassis->step(dt);
    }
    if (_engine) {
        _engine->setGear(_gear);
        pe::Real total_angular_velocity = PE_R(0.0);
        int motored_wheel_count = 0;
        for (int i = 0; i < PE_I(_suspensions.size()); i++) {
            if (_wheel_info[i].motor) {
                total_angular_velocity += _wheels[i]->getAngularVelocity().dot(-_wheels[i]->getTransform().getAxis(0));
                motored_wheel_count++;
            }
        }
        if (motored_wheel_count != 0) {
            const pe::Real avg_angular_velocity = total_angular_velocity / PE_R(motored_wheel_count);
            const pe::Real torque = _engine->getTorque(PE_ABS(avg_angular_velocity)) * _throttle;
            const pe::Real wheel_torque = torque / PE_R(motored_wheel_count);
            for (int i = 0; i < PE_I(_suspensions.size()); i++) {
                if (_wheel_info[i].motor) {
                    _wheels[i]->applyTorque(torque);
                }
            }
        }
    }
    for (auto& wheel : _wheels) {
        wheel->step(dt);
    }
    _brake = PE_MAX(_brake, PE_R(0.0));
    for (auto& suspension : _suspensions) {
        if (_brake > PE_EPS) {
            const pe::Real current_speed = suspension->getWheel()->getBody()->getAngularVelocity().dot(suspension->getWheel()->getTransform().getAxis(0));
            suspension->setTargetWheelSpeed(PE_MAX((PE_R(1.0) - _brake) * current_speed, PE_R(0.0)));
        }
        else suspension->releaseTargetWheelSpeed();
        suspension->step(dt);
    }
}

void VehicleBase::setTransform(const pe::Transform& trans) {
    if (!_chassis) return;
    const pe::Transform delta_trans = _chassis->getTransform().inverse() * trans;
    _chassis->setTransform(trans);
    for (auto wheel : _wheels) {
        wheel->setTransform(delta_trans * wheel->getTransform());
    }
}

pe::Transform VehicleBase::getTransform() const {
    if (!_chassis) return pe::Transform::identity();
    return _chassis->getTransform();
}

} // namespace pe_vehicle
