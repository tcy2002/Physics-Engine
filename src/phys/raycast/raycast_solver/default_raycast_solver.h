#pragma once

#include "raycast_solver_base.h"

namespace pe_phys_raycast {
    
    class DefaultRaycastSolver : public RaycastSolverBase {
    public:
        DefaultRaycastSolver() {}
        virtual ~DefaultRaycastSolver() {}
        virtual bool performRaycast(const pe::Vector3& start, const pe::Vector3& direction, pe::Real length,
                                    const pe::Array<pe_phys_object::RigidBody*>& objects,
                                    const pe::Uint32HashList& ignores,
                                    RaycastResult& result) override;
    };
    
} // namespace pe_phys_ray