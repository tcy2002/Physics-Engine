#pragma once

#include "force_constraint/lorentz_circle_constraint.h"
#include "phys/collision/narrow_phase/contact_result.h"

namespace pe_phys_constraint {

    class NonSmoothForceBase {
    public:
        NonSmoothForceBase() {}
        virtual ~NonSmoothForceBase() {}

        virtual int dimensions() const = 0;
        virtual int constraintPerForce() const = 0;
        virtual void initForces(pe::VectorX& forces, pe::VectorX& lambda) = 0;
        virtual void preprocess(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                size_t contact_size,
                                const pe::Array<pe_phys_object::RigidBody*>& objects,
                                const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                const pe::VectorX& vel, pe::Real dt, pe::Real char_mass, pe::Real char_speed) = 0;
        virtual pe::Real surrogateDualGap(const pe::VectorX& lambda) {return -lambda.cwiseProduct(_constraint_val).mean();}
        virtual void nonSmoothResiduals(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                        size_t contact_size,
                                        const pe::Array<pe_phys_object::RigidBody*>& objects,
                                        const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                        const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda,
                                        bool use_stored_constraints,
                                        pe::Real mu, pe::VectorX& ru, pe::VectorX& rf, pe::VectorX& rl) = 0;
        virtual void calcConstraints(const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda) = 0;
        virtual pe::VectorX calcTangentWeight(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                              const pe::Array<pe_phys_object::RigidBody*>& objects,
                                              const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                              const pe::VectorX& vel, const pe::VectorX& forces,
                                              pe::Real char_mass) { return pe::VectorX::Ones(forces.size()); }
        virtual void linearSystemReserve(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                         const pe::Array<pe_phys_object::RigidBody*>& objects,
                                         const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                         pe::Set<pe::KV<size_t, size_t>>& obj_pairs,
                                         pe::Array<Eigen::Triplet<pe::Real>>& triplets);
        virtual void linearSystemAddition(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                          size_t contact_size,
                                          const pe::Array<pe_phys_object::RigidBody*>& objects,
                                          const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                          const pe::VectorX& lambda, const pe::VectorX& rf, const pe::VectorX& rl,
                                          pe::Real eps, pe::VectorX& y,
                                          pe::Map<pe::KV<size_t, size_t>, pe::Real*>& mat_pointers) = 0;
        virtual void retrieveNonSmoothForceInc(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                               size_t contact_size,
                                               const pe::Array<pe_phys_object::RigidBody*>& objects,
                                               const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                               const pe::VectorX& lambda, const pe::VectorX& du, const pe::VectorX& rf, const pe::VectorX& rl,
                                               pe::Real mu, pe::VectorX& df, pe::VectorX& dl) = 0;
        virtual bool filterLineSearch(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                      size_t contact_size,
                                      const pe::Array<pe_phys_object::RigidBody*>& objects,
                                      const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                      const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda,
                                      pe::Real mu, pe::Real char_mass, pe::VectorX& du, pe::VectorX& df, pe::VectorX& dl) = 0;
        virtual pe::VectorX ACVector(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                     const pe::Array<pe_phys_object::RigidBody*>& objects,
                                     const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                     const pe::VectorX& vel, const pe::VectorX& forces) {return pe::VectorX::Zero(forces.size());}

    protected:
        ForceConstraintBase* _fc;
        pe::VectorX _constraint_val;
    };

} // namespace pe_phys_constraint