#include "vehicle/tracked_vehicle/tracked_vehicle.h"

namespace pe_vehicle {

TrackedVehicle::~TrackedVehicle() {
    delete _left_track;
    delete _right_track;
}

static COMMON_FORCE_INLINE pe::Real getTangentLength(pe::Real height1, pe::Real height2, pe::Real gap, pe::Real radius1, pe::Real radius2) {
    const pe::Real h_d = height2 - height1;
    const pe::Real r_d = radius2 - radius1;
    return PE_SQRT(gap * gap + h_d * h_d - r_d * r_d);
}

static COMMON_FORCE_INLINE pe::Real getTangentSlope(pe::Real height1, pe::Real height2, pe::Real gap, pe::Real radius1, pe::Real radius2) {
    const pe::Real h_d = height2 - height1;
    const pe::Real r_d = radius2 - radius1;
    const pe::Real tan1 = h_d / gap;
    const pe::Real tan2 = r_d / PE_SQRT(gap * gap + h_d * h_d - r_d * r_d);
    return (tan1 + tan2) / (PE_R(1.0) - tan1 * tan2);
}

static COMMON_FORCE_INLINE pe::Real getBoundingLength(const pe::Array<VehicleBase::WheelInfo>& wheels, int start_index, int end_index, pe::Real wheel_ragion_length) {
    const pe::Real wheel_gap = (end_index - start_index > 0) ? (wheel_ragion_length / PE_R(end_index - start_index)) : PE_R(0.0);
    
    // from the first wheel to last wheel, calculate the co-tangent line pair by pair
    std::vector<pe::KV<pe::Vector3, pe::Real>> stack; // <wheel center, wheel radius, slope>
    for (int i = start_index; i <= end_index; i++) {
        if (wheels[i].type == AxleType::AT_NONE) continue; // ignore the wheel not properly set
        const pe::Vector3 o(wheel_gap * i + wheels[i].anchor_offset_z, wheels[i].anchor_offset_y - wheels[i].suspension_rest_length, wheels[i].radius);
        pe::Real k = PE_REAL_MAX;
        while (!stack.empty()) {
            const auto& top = stack.back();
            k = getTangentSlope(top.first.y, o.y, o.x - top.first.x, top.first.z, o.z);
            if (k <= top.second) break;
            stack.pop_back();
        }
        stack.push_back(pe::KV<pe::Vector3, pe::Real>(o, k));
    }
    // traverse the stack to get the length
    pe::Real length = PE_R(0.0);
    for (size_t i = 1; i < stack.size(); ++i) {
        const auto& prev = stack[i - 1];
        const auto& curr = stack[i];
        const pe::Real segment = getTangentLength(prev.first.y, curr.first.y, curr.first.x - prev.first.x, prev.first.z, curr.first.z);
        length += segment;
    }

    // from the last wheel to first wheel, calculate the co-tangent line pair by pair
    std::vector<pe::KV<pe::Vector3, pe::Real>> stack2;
    for (int i = end_index; i >= start_index; i--) {
        if (wheels[i].type == AxleType::AT_NONE) continue; // ignore the wheel not properly set
        const pe::Vector3 o(wheel_gap * i + wheels[i].anchor_offset_z, wheels[i].anchor_offset_y - wheels[i].suspension_rest_length, wheels[i].radius);
        pe::Real k = PE_REAL_MAX;
        while (!stack2.empty()) {
            const auto& top = stack2.back();
            k = getTangentSlope(top.first.y, o.y, o.x - top.first.x, top.first.z, o.z);
            if (k <= top.second) break;
            stack2.pop_back();
        }
        stack2.push_back(pe::KV<pe::Vector3, pe::Real>(o, k));
    }
    // traverse the stack to get the length
    for (size_t i = 1; i < stack2.size(); ++i) {
        const auto& prev = stack2[i - 1];
        const auto& curr = stack2[i];
        const pe::Real segment = getTangentLength(prev.first.y, curr.first.y, curr.first.x - prev.first.x, prev.first.z, curr.first.z);
        length += segment;
    }

    // get the wheels' rest circumference part from stack's info
    if (stack.size() >= 2) {
        stack.front().second = stack2.back().second;
        for (int i = 0; i < PE_I(stack.size()) - 1; i++) {
            const pe::Real atan1 = PE_ATAN(stack[i].second);
            const pe::Real atan2 = PE_ATAN(stack[i + 1].second);
            const pe::Real arc = stack[i].first.z * (i == 0 ? (PE_PI + atan1 - atan2) : (atan1 - atan2));
            length += arc;
        }
        stack2.front().second = stack.back().second;
        for (int i = 0; i < PE_I(stack2.size()) - 1; i++) {
            const pe::Real atan1 = PE_ATAN(stack2[i].second);
            const pe::Real atan2 = PE_ATAN(stack2[i + 1].second);
            const pe::Real arc = stack2[i].first.z * (i == 0 ? (PE_PI + atan1 - atan2) : (atan1 - atan2));
            length += arc;
        }
    } else if (!stack.empty()) {
        length += PE_R(2.0) * PE_PI * stack.front().first.z;
    }

    return length;
}

static void getBoundingInfo(const pe::Array<VehicleBase::WheelInfo>& wheels, int start_index, int end_index, pe::Real wheel_ragion_length,
                            pe::Real& length, pe::Real& radius, pe::Real& tightening_ratio, pe::Real& offset_x, pe::Real& offset_y) {
    const pe::Real wheel_gap = (end_index - start_index > 0) ? (wheel_ragion_length / PE_R(end_index - start_index)) : PE_R(0.0);
    
    // get the radius
    pe::Real top = PE_REAL_MIN, down = PE_REAL_MAX;
    int first_valid_wheel = -1, last_valid_wheel = -1;
    for (int i = start_index; i <= end_index; i++) {
        if (wheels[i].type == AxleType::AT_NONE) continue; // ignore the wheel not properly set
        if (first_valid_wheel == -1) first_valid_wheel = i;
        last_valid_wheel = i;
        top = PE_MAX(wheels[i].anchor_offset_y - wheels[i].suspension_rest_length + wheels[i].radius, top);
        down = PE_MIN(wheels[i].anchor_offset_y - wheels[i].suspension_rest_length - wheels[i].radius, down);
    }
    radius = (top - down) / PE_R(2.0);

    if (first_valid_wheel == last_valid_wheel && first_valid_wheel != -1) {
        length = PE_R(0.0);
        offset_x = wheels[first_valid_wheel].anchor_offset_z;
        offset_y = wheels[first_valid_wheel].anchor_offset_y + wheels[first_valid_wheel].suspension_rest_length;
        tightening_ratio = PE_R(1.0);
        return;
    }

    // get the offset_y
    offset_y = (down + top) / PE_R(2.0);

    // get the length
    const pe::Real first_wheel_diff = radius - wheels[first_valid_wheel].radius;
    const pe::Real first_wheel_offset = wheels[first_valid_wheel].anchor_offset_y - wheels[first_valid_wheel].suspension_rest_length + offset_y;
    const pe::Real last_wheel_diff = radius - wheels[last_valid_wheel].radius;
    const pe::Real last_wheel_offset = wheels[last_valid_wheel].anchor_offset_y - wheels[last_valid_wheel].suspension_rest_length + offset_y;
    const pe::Real former_part = -PE_SQRT(PE_MAX(PE_SQR(first_wheel_diff) - PE_SQR(first_wheel_offset), PE_R(0.0)));
    const pe::Real latter_part = -PE_SQRT(PE_MAX(PE_SQR(last_wheel_diff) - PE_SQR(last_wheel_offset), PE_R(0.0)));
    length = (last_valid_wheel - first_valid_wheel) * wheel_gap + former_part + latter_part;

    // get the offset_x
    offset_x = (latter_part - former_part) / PE_R(2.0);

    // get the tightening ratio, which needs the real track length
    const pe::Real track_length = PE_PI * radius * PE_R(2.0) + PE_R(2.0) * length;
    const pe::Real real_track_length = getBoundingLength(wheels, start_index, end_index, wheel_ragion_length);
    tightening_ratio = real_track_length / track_length;
}

void TrackedVehicle::init(pe_interface::World* phys_world) {
    VehicleBase::init(phys_world);

    if (!_chassis) return;

    // calculate the bounding capsule the can contains all the wheel on left side
    pe::Real length_l, radius_l , tightening_ratio_l, offset_x_l, offset_y_l;
    getBoundingInfo(_wheel_info, 0, _wheel_count_per_side - 1, _wheel_region_length,
                    length_l, radius_l, tightening_ratio_l, offset_x_l, offset_y_l);

    // calculate the bounding capsule the can contains all the wheel on right side
    pe::Real length_r, radius_r, tightening_ratio_r, offset_x_r, offset_y_r;
    getBoundingInfo(_wheel_info, _wheel_count_per_side, _wheel_count_per_side * 2 - 1, _wheel_region_length,
                    length_r, radius_r, tightening_ratio_r, offset_x_r, offset_y_r);

    // create left track
    _left_track = new Track(_chassis, pe::Vector3(-_wheel_region_width / PE_R(2.0), offset_y_l, offset_x_l) + _wheel_region_offset,
                            length_l, radius_l, _wheel_width, 30, PE_R(0.1), PE_R(0.8), 10, PE_R(1.0), tightening_ratio_l * PE_R(0.98));
    _left_track->init(phys_world);

    // create right track
    _right_track = new Track(_chassis, pe::Vector3(_wheel_region_width / PE_R(2.0), offset_y_r, offset_x_r) + _wheel_region_offset,
                            length_r, radius_r, _wheel_width, 30, PE_R(0.1), PE_R(0.8), 10, PE_R(1.0), tightening_ratio_r * PE_R(0.98));
    _right_track->init(phys_world);
}

void TrackedVehicle::step(pe::Real dt) {
    // override the super class's step function to control the wheels separately by side
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
            const pe::Real torque = _engine->getTorque(PE_ABS(avg_angular_velocity));
            const pe::Real wheel_torque = torque / PE_R(motored_wheel_count);
            for (int i = 0; i < PE_I(_suspensions.size()); i++) {
                if (_wheel_info[i].motor) {
                    _wheels[i]->applyTorque(i < _wheel_count_per_side ? (torque * _left_throttle) : (torque * _right_throttle));
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
            const pe::Real current_speed = -suspension->getWheel()->getBody()->getAngularVelocity().dot(suspension->getWheel()->getTransform().getAxis(0));
            suspension->setTargetWheelSpeed(PE_MAX((PE_R(1.0) - _brake), PE_R(0.0)) * current_speed);
        }
        else suspension->releaseTargetWheelSpeed();
        suspension->step(dt);
    }

    if (_left_track) {
        _left_track->step(dt);
    }
    if (_right_track) {
        _right_track->step(dt);
    }
}

} // namespace pe_vehicle
