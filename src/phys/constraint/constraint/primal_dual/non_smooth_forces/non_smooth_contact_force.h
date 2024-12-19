#pragma once

#include "non_smooth_force_base.h"
#include "force_constraint/lorentz_circle_constraint.h"
#include "utils/logger.h"

namespace pe_phys_constraint {

    class NonSmoothContactForce : public NonSmoothForceBase {
    public:
        NonSmoothContactForce() {_fc = new LorentzCircleConstraint(friction);}
        virtual ~NonSmoothContactForce() {}

        int dimensions() const override { return 3; }
        int constraintPerForce() const override { return 2; }

        void preprocess(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                        size_t contact_size,
                        const pe::Array<pe_phys_object::RigidBody*>& objects,
                        const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                        const pe::VectorX& vel, pe::Real dt, pe::Real char_mass, pe::Real char_speed) override;
        void initForces(pe::VectorX& forces, pe::VectorX& lambda) override;
        void nonSmoothResiduals(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                size_t contact_size,
                                const pe::Array<pe_phys_object::RigidBody*>& objects,
                                const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda,
                                bool use_stored_constraints,
                                pe::Real mu, pe::VectorX& ru, pe::VectorX& rf, pe::VectorX& rl) override;
        void calcConstraints(const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda) override;
        pe::VectorX calcTangentWeight(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                      const pe::Array<pe_phys_object::RigidBody*>& objects,
                                      const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                      const pe::VectorX& vel, const pe::VectorX& forces,
                                      pe::Real char_mass) override;
        void linearSystemAddition(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                  size_t contact_size,
                                  const pe::Array<pe_phys_object::RigidBody*>& objects,
                                  const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                  const pe::VectorX& lambda, const pe::VectorX& rf, const pe::VectorX& rl,
                                  pe::Real eps, pe::VectorX& y,
                                  pe::Map<pe::KV<size_t, size_t>, pe::Real*>& mat_pointers) override;
        void retrieveNonSmoothForceInc(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
            size_t contact_size,
            const pe::Array<pe_phys_object::RigidBody*>& objects,
            const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
            const pe::VectorX& lambda, const pe::VectorX& du, const pe::VectorX& rf, const pe::VectorX& rl,
            pe::Real mu, pe::VectorX& df, pe::VectorX& dl) override;
        bool filterLineSearch(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                              size_t contact_size,
                              const pe::Array<pe_phys_object::RigidBody*>& objects,
                              const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                              const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda,
                              pe::Real mu, pe::Real char_mass, pe::VectorX& du, pe::VectorX& df, pe::VectorX& dl) override;
        pe::VectorX ACVector(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                             const pe::Array<pe_phys_object::RigidBody*>& objects,
                             const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                             const pe::VectorX& vel, const pe::VectorX& forces) override;

        pe::Real restitution = 0.4;
        pe::Real friction = 0.5;
        bool spring_based_force = false;
        pe::Real spring_k = 1e4;
        pe::Real spring_d = 0.5;

    protected:
        pe::MatrixMN _gradients;
        pe::MatrixMN _hessians;
        pe::VectorX _non_smooth_k;
        pe::VectorX _inv_constraint_val;
        pe::VectorX _k_norm_inv;
        pe::VectorX _sf;
        pe::Real _compliance;
        pe::Real _reduction;
        pe::Array<pe::Matrix3> _weight_mat;
        pe::Array<Eigen::JacobiSVD<pe::Matrix2>> _k_mat_svd;
        pe::Real _current_avg_mass;
    };

} // namespace pe_phys_constraint