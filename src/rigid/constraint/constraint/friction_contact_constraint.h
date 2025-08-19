#pragma once

#include "rigid/phys_general.h"
#include "rigid/collision/narrow_phase/contact_result.h"
#include "constraint.h"
#include "primal_dual/non_smooth_forces/non_smooth_force_base.h"

#define PE_MAX_CONTACT_POINT 8

namespace pe_phys_constraint {

    class FrictionContactConstraint : public Constraint {
        // for sequential impulse solver
    private:
        struct ConstraintInfo {
            pe::Vector3 r_a;
            pe::Vector3 r_b;
            pe::Vector3 n;
            pe::Vector3 t0;
            pe::Vector3 t1;
            pe::Real n_rhs = 0;
            pe::Real n_jmj_inv = 0;
            pe::Real t0_jmj_inv = 0;
            pe::Real t1_jmj_inv = 0;
            pe::Real n_applied_impulse = 0;
            pe::Real t0_applied_impulse = 0;
            pe::Real t1_applied_impulse = 0;
            pe::Real friction_coeff = 0;
        };
        pe::Array<ConstraintInfo> _cis;
        pe_phys_collision::ContactResult* _contact_result = nullptr;
    public:
        void setContactResult(pe_phys_collision::ContactResult& cr) { _contact_result = &cr; }

        // for primal-dual solver
    private:
        size_t _contact_size;
        size_t _n_objects;
        size_t _n_rigid_dof;
        size_t _n_force_dof;
        size_t _n_constraint_dof;
        size_t _n_unknowns;
        pe::VectorX _vel_old;
        pe::VectorX _vel;
        pe::VectorX _forces;
        pe::VectorX _lambda;
        pe::VectorX _rhs;
        pe::SparseMatrix _A;
        pe::SparseMatrix _mass_mat;
        pe::Map<pe::KV<size_t, size_t>, pe::Real*> _mat_pointers;
        pe::Real _char_mass;
        pe::Real _char_speed;
        pe::Real _dt;
        NonSmoothForceBase* _nsf;
        pe::Map<pe_phys_object::RigidBody*, size_t> _object2index;
        const pe::Array<pe_phys_object::RigidBody*>* _objects = nullptr;
        const pe::Array<pe_phys_collision::ContactResult*>* _contact_results = nullptr;
    public:
        void setObjects(const pe::Array<pe_phys_object::RigidBody*>* objs) { _objects = objs; }
        void setContactResults(const pe::Array<pe_phys_collision::ContactResult*>* crs) { _contact_results = crs; }

    public:
        ConstraintType getType() const override { return ConstraintType::CT_FRICTION_CONTACT; }

        FrictionContactConstraint() {}
        virtual ~FrictionContactConstraint() {}

        // for sequential impulse solver
        void initSequentialImpulse(const ConstraintParam& param) override;
        void iterateSequentialImpulse(int iter) override;

        // for primal-dual solver
        void initPrimalDual(const ConstraintParam& param) override;
        bool iteratePrimalDual(int iter, pe::LDLT& ldlt, pe::Real& hu,
                               pe::VectorX& du, pe::VectorX& df, pe::VectorX& dl,
                               pe::VectorX& ru, pe::VectorX& ru_add,
                               pe::VectorX& rf, pe::VectorX& wrf, pe::VectorX& rl,
                               pe::Real& exit_err, pe::Real tol) override;
        void afterPrimalDual() override;
    };

} // namespace pe_phys_constraint