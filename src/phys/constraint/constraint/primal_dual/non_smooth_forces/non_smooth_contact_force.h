#pragma once

#include "non_smooth_force_base.h"

namespace pe_phys_constraint {

    class NonSmoothContactForce : public NonSmoothForceBase {
    public:
        NonSmoothContactForce() {}
        virtual ~NonSmoothContactForce() {}

        int dimensions() const override { return 3; }
        int constraintPerForce() const override { return 2; }

        void preprocess(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                        const pe::Array<pe_phys_object::RigidBody*>& objects,
                        const pe::VectorX& vel, pe::Real dt, pe::Real char_mass, pe::Real char_speed) override {
            const size_t n_contacts = contacts.size();
            _constraint_val.resize(n_contacts);
            _gradients.resize(n_contacts, 2);
            _hessians.resize(n_contacts * 2, 2);
            _non_smooth_k.resize(n_contacts * 3, 0);
            _k_norm_inv.resize(n_contacts);
            //_k_mat_svd.resize(n_contacts);
            _weight_mat.resize(n_contacts);
            _inv_constraint_val.resize(n_contacts * 2);
            _current_avg_mass = char_mass;
        }

        void initForces(pe::VectorX &forces, pe::VectorX &lambda) override {

        }

        void nonSmoothResiduals(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                const pe::Array<pe_phys_object::RigidBody*>& objects,
                                const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda,
                                pe::Real mu, pe::VectorX& ru, pe::VectorX& rf, pe::VectorX& rl) override {

        }

        void calcConstraints(const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda) override {

        }

        pe::VectorX calcTangentWeight(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                      const pe::Array<pe_phys_object::RigidBody*>& objects,
                                      const pe::VectorX& vel, const pe::VectorX& forces,
                                      pe::Real char_mass) override {
            return pe::VectorX::zeros(forces.size());
        }

        void linearSystemAddition(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                  const pe::Array<pe_phys_object::RigidBody*>& objects,
                                  const pe::VectorX& lambda, const pe::VectorX& rf, const pe::VectorX& rl,
                                  pe::Real eps, pe::VectorX& y,
                                  pe::HashMap<pe::KV<pe_phys_object::RigidBody*, pe_phys_object::RigidBody*>, pe::Real>& mat_pointers) override {

        }

        void retrieveNonSmoothForceInc(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                       const pe::Array<pe_phys_object::RigidBody*>& objects,
                                       const pe::VectorX& lambda, const pe::VectorX& rf, const pe::VectorX& rl,
                                       pe::Real mu, pe::VectorX& df, pe::VectorX& dl) override {

        }

        bool filterLineSearch(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                              const pe::Array<pe_phys_object::RigidBody*>& objects,
                              const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda,
                              pe::Real mu, pe::Real char_mass, pe::VectorX& du, pe::VectorX& df, pe::VectorX dl) override {
            return false;
        }

        pe::VectorX ACVector(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                             const pe::Array<pe_phys_object::RigidBody*>& objects,
                             const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda) {
            return pe::VectorX::zeros(forces.size());
        }

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
        //pe::Array<> _k_mat_svd;
        pe::Real _current_avg_mass;
    };

} // namespace pe_phys_constraint