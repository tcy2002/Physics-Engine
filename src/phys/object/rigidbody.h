#pragma once

#include "collision_body.h"

namespace pe_phys_object {

class RigidBody : CollisionBody {
public:
    bool isDeformable() const override { return false; }
    virtual bool isFracturable() const { return false; }
};

} // namespace pe_phys_object
