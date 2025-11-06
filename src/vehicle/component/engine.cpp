#include "engine.h"
#include "utils/logger.h"

namespace pe_vehicle {

void Engine::setGear(int gear) {
    if (gear < -1 || gear > _gear_count) {
        throw std::out_of_range("Invalid gear in Engine::setGear: " + std::to_string(gear));
    }
    _gear = gear;
}

void Engine::setGearRatio(int gear_index, pe::Real ratio) {
    if (gear_index < -1 || gear_index > _gear_count) {
        throw std::out_of_range("Invalid gear index in Engine::setGearRatio: " + std::to_string(gear_index));
    }
    if (gear_index == 0) {
        return; // P
    }
    if (gear_index == -1) {
        gear_index = 0; // R
    }
    _gear_ratios[gear_index] = ratio;
}

pe::Real Engine::getGearRatio(int gear_index) const {
    if (gear_index < -1 || gear_index > _gear_count) {
        throw std::out_of_range("Invalid gear index in Engine::getGearRatio: " + std::to_string(gear_index));
    }
    if (gear_index == 0) {
        return 0; // P
    }
    if (gear_index == -1) {
        gear_index = 0; // R
    }
    return _gear_ratios[gear_index];
}

void Engine::addTorquePoint(pe::Real rpm, pe::Real torque) {
    for (size_t i = 0; i < _torque_curve.size(); ++i) {
        if (PE_APPROX_EQUAL(_torque_curve[i].first, rpm)) {
            _torque_curve[i].second = torque;
            return;
        }
        if (_torque_curve[i].first > rpm) {
            _torque_curve.insert(_torque_curve.begin() + i, pe::KV<pe::Real, pe::Real>(rpm, torque));
            return;
        }
    }
    _torque_curve.push_back(pe::KV<pe::Real, pe::Real>(rpm, torque));
}

void Engine::clearTorqueCurve() {
    _torque_curve.clear();
}

static bool checkStringInt(const std::string& str, int& out_int) {
    try {
        size_t pos;
        out_int = std::stoi(str, &pos);
        return pos == str.size();
    } catch (...) {
        return false;
    }
}

pe::Real Engine::getTorque(pe::Real wheel_rpm) const {
    if (_gear == 0) {
        return 0; // P
    }
    const pe::Real gear_ratio = (_gear == -1) ? _gear_ratios[0] : _gear_ratios[_gear];
    const pe::Real ratio = gear_ratio * _final_drive_ratio;
    pe::Real engine_rpm = PE_ABS(wheel_rpm * ratio);
    if (_torque_curve.empty()) {
        return 0;
    }
    if (engine_rpm <= _torque_curve.front().first) {
        return _torque_curve.front().second * ratio;
    }
    if (engine_rpm >= _torque_curve.back().first) {
        return _torque_curve.back().second * ratio;
    }
    for (size_t i = 0; i < _torque_curve.size() - 1; ++i) {
        if (engine_rpm >= _torque_curve[i].first && engine_rpm <= _torque_curve[i + 1].first) {
            const pe::Real t1 = _torque_curve[i].second;
            const pe::Real t2 = _torque_curve[i + 1].second;
            const pe::Real r1 = _torque_curve[i].first;
            const pe::Real r2 = _torque_curve[i + 1].first;
            const pe::Real torque = t1 + (t2 - t1) * (engine_rpm - r1) / (r2 - r1);
            return wheel_rpm > 0 ? torque * ratio : -torque * ratio;
        }
    }
    return 0;
}

void Engine::loadConfigFromJson(const nlohmann::json& j) {
    if (j.contains("gear_count") && j["gear_count"].is_number_integer()) {
        _gear_count = j["gear_count"].get<int>();
        if (_gear_count < 1) {
            throw std::runtime_error("Engine config JSON 'gear_count' must be at least 1.");
        }
        _gear_ratios.clear();
        _gear_ratios.reserve(_gear_count + 1);
    } else {
        throw std::runtime_error("Engine config JSON must contain 'gear_count' field.");
    }
    if (j.contains("final_drive_ratio") && j["final_drive_ratio"].is_number()) {
        _final_drive_ratio = j["final_drive_ratio"].get<pe::Real>();
    }
    else {
        throw std::runtime_error("Engine config JSON must contain 'final_drive_ratio' field.");
    }
    if (j.contains("gear_ratios") && j["gear_ratios"].is_array()) {
        const auto& ratios = j["gear_ratios"];
        if (ratios.size() != _gear_count + 1) {
            throw std::runtime_error("Engine config JSON 'gear_ratios' size does not match gear count.");
        }
        for (const auto& ratio : ratios) {
            if (ratio.is_object() && ratio.size() == 1) {
                const std::string key = ratio.begin().key();
                if (!ratio[key].is_number()) {
                    throw std::runtime_error("Engine config JSON 'gear_ratios' must contain numeric values.");
                }
                int gear;
                if (key == "R") {
                    _gear_ratios[0] = ratio["R"].get<pe::Real>();
                } else if (checkStringInt(key, gear) && gear >= 1 && gear <= _gear_count) {
                    _gear_ratios[gear] = ratio[key].get<pe::Real>();
                } else {
                    throw std::runtime_error("Engine config JSON 'gear_ratios' has invalid gear key: " + key);
                }
            } else {
                throw std::runtime_error("Engine config JSON invalid 'gear_ratios'.");
            }
        }
    } else {
        throw std::runtime_error("Engine config JSON must contain 'gear_ratios' field.");
    }
    if (j.contains("torque_curve") && j["torque_curve"].is_array()) {
        const auto& curve = j["torque_curve"];
        clearTorqueCurve();
        for (const auto& point : curve) {
            if (point.is_object() && point.size() == 2 && point.contains("r") && point.contains("t")) {
                const pe::Real rpm = point["r"].get<pe::Real>();
                const pe::Real torque = point["t"].get<pe::Real>();
                addTorquePoint(rpm, torque);
            } else {
                throw std::runtime_error("Engine config JSON 'torque_curve' points must only contain 'r' and 't' fields.");
            }
        }
    } else {
        throw std::runtime_error("Engine config JSON must contain 'torque_curve' field.");
    }
}

} // namespace pe_vehicle