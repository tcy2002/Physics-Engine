#pragma once

#include "rigidbody.h"

namespace pe_phys_object {

    class FracturableObject : public RigidBody {
        COMMON_MEMBER_SET_GET(pe::Real, threshold, Threshold)

    public:
        FracturableObject(): RigidBody() {}
        virtual ~FracturableObject() {}

        virtual bool isDeformable() const override { return false; }
        virtual bool isFracturable() const override { return true; }
    };

} // namespace pe_phys_fracture