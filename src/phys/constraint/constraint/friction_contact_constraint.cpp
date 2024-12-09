#include "friction_contact_constraint.h"
#include "primal_dual/primal_dual_utils.h"
#include "primal_dual/non_smooth_forces/non_smooth_contact_force.h"

//#include <amgcl/adapter/block_matrix.hpp>
//#include <amgcl/amg.hpp>
//#include <amgcl/backend/eigen.hpp>
//#include <amgcl/coarsening/smoothed_aggregation.hpp>
//#include <amgcl/make_solver.hpp>
//#include <amgcl/relaxation/as_preconditioner.hpp>
//#include <amgcl/relaxation/gauss_seidel.hpp>
//#include <amgcl/solver/cg.hpp>
//#include <amgcl/value_type/static_matrix.hpp>

// style-checked
namespace pe_phys_constraint {

    void FrictionContactConstraint::initSequentialImpulse(const ConstraintParam& param) {
        if (!_contact_result) return;
        const int point_size = PE_MIN(PE_MAX_CONTACT_POINT, _contact_result->getPointSize());
        _cis.resize(point_size);

        _object_a = _contact_result->getObjectA();
        _object_b = _contact_result->getObjectB();
        const pe::Transform& transform_a = _object_a->getTransform();
        const pe::Transform& transform_b = _object_b->getTransform();

        for (int i = 0; i < point_size; i++) {
            const pe_phys_collision::ContactPoint& cp = _contact_result->getContactPoint(i);
            ConstraintInfo& ci = _cis[i];

            const pe::Vector3 r_a = transform_a.getBasis() * cp.getLocalPosA();
            const pe::Vector3 r_b = transform_b.getBasis() * cp.getLocalPosB();

            //// setup ci
            ci.r_a = r_a;
            ci.r_b = r_b;
            ci.n = cp.getWorldNormal();
            ci.t0 = cp.getTangent(0);
            ci.t1 = cp.getTangent(1);

            const pe::Real inv_mass_sum = _object_a->getInvMass() + _object_b->getInvMass();
            const pe::Matrix3 rot_inv_inertia_a = _object_a->getWorldInvInertia();
            const pe::Matrix3 rot_inv_inertia_b = _object_b->getWorldInvInertia();

            //// normal denom
            {
                pe::Vector3 rxn_a = r_a.cross(ci.n);
                pe::Vector3 rxn_b = r_b.cross(ci.n);
                ci.n_denom_inv = PE_R(1.0) / (inv_mass_sum + (rot_inv_inertia_a * rxn_a).dot(rxn_a) +
                                        (rot_inv_inertia_b * rxn_b).dot(rxn_b));
            }
            //// tangent denom
            {
                pe::Vector3 rxn_a = r_a.cross(ci.t0);
                pe::Vector3 rxn_b = r_b.cross(ci.t0);
                ci.t0_denom_inv = PE_R(1.0) / (inv_mass_sum + (rot_inv_inertia_a * rxn_a).dot(rxn_a) +
                                         (rot_inv_inertia_b * rxn_b).dot(rxn_b));
            }
            {
                pe::Vector3 rxn_a = r_a.cross(ci.t1);
                pe::Vector3 rxn_b = r_b.cross(ci.t1);
                ci.t1_denom_inv = PE_R(1.0) / (inv_mass_sum + (rot_inv_inertia_a * rxn_a).dot(rxn_a) +
                                         (rot_inv_inertia_b * rxn_b).dot(rxn_b));
            }

            const pe::Vector3 vel_a = _object_a->getLinearVelocity() + _object_a->getAngularVelocity().cross(r_a);
            const pe::Vector3 vel_b = _object_b->getLinearVelocity() + _object_b->getAngularVelocity().cross(r_b);

            //// normal rhs
            {
                pe::Real rev_vel_r = -ci.n.dot(vel_a - vel_b);
                if (PE_ABS(rev_vel_r) < param.restitutionVelocityThreshold) {
                    rev_vel_r = 0;
                }
                rev_vel_r *= _contact_result->getRestitutionCoeff();

                pe::Real penetration = cp.getDistance();
                penetration = PE_MAX(penetration, -param.penetrationThreshold);
                ci.n_rhs = (rev_vel_r - param.kerp * penetration / param.dt) * ci.n_denom_inv;
            }

            ci.n_applied_impulse = 0;
            ci.t0_applied_impulse = 0;
            ci.t1_applied_impulse = 0;
            ci.friction_coeff = _contact_result->getFrictionCoeff();
        }
    }

    void FrictionContactConstraint::iterateSequentialImpulse(int iter) {
        for (auto& ci : _cis) {
            const pe::Vector3& r_a = ci.r_a;
            const pe::Vector3& r_b = ci.r_b;
            const pe::Vector3& n = ci.n;
            const pe::Vector3& t0 = ci.t0;
            const pe::Vector3& t1 = ci.t1;
            const pe::Vector3 vel_r = (_object_a->getTempLinearVelocity() +
                    _object_a->getTempAngularVelocity().cross(r_a))
                    - (_object_b->getTempLinearVelocity()
                    + _object_b->getTempAngularVelocity().cross(r_b));

            // compute impulse
            pe::Real n_impulse = ci.n_rhs - n.dot(vel_r) * ci.n_denom_inv;
            n_impulse = PE_MAX(n_impulse, -ci.n_applied_impulse);
            ci.n_applied_impulse += n_impulse;

#       if true
            pe::Real t0_total_impulse = ci.t0_applied_impulse - t0.dot(vel_r) * ci.t0_denom_inv;
            pe::Real max_friction = ci.friction_coeff * ci.n_applied_impulse;
            pe::Real t1_total_impulse = ci.t1_applied_impulse - t1.dot(vel_r) * ci.t1_denom_inv;
            t0_total_impulse = PE_MIN(t0_total_impulse, max_friction);
            t1_total_impulse = PE_MIN(t1_total_impulse, max_friction);
            t0_total_impulse = PE_MAX(t0_total_impulse, -max_friction);
            t1_total_impulse = PE_MAX(t1_total_impulse, -max_friction);
#       else
            pe::Real t0_total_impulse = ci.t0_applied_impulse - t0.dot(vel_r) * ci.t0_denom_inv;
            pe::Real t1_total_impulse = ci.t1_applied_impulse - t1.dot(vel_r) * ci.t1_denom_inv;
            const pe::Real scale = ci.friction_coeff * ci.n_applied_impulse /
                    PE_SQRT(PE_SQR(t0_total_impulse) + PE_SQR(t1_total_impulse));
            if (scale < 1) {
                t0_total_impulse *= scale;
                t1_total_impulse *= scale;
            }
#       endif

            pe::Vector3 impulse_vector = n_impulse * n + (t0_total_impulse - ci.t0_applied_impulse) * t0 +
                    (t1_total_impulse - ci.t1_applied_impulse) * t1;

            ci.t0_applied_impulse = t0_total_impulse;
            ci.t1_applied_impulse = t1_total_impulse;

            _object_a->applyTempImpulse(r_a, impulse_vector);
            _object_b->applyTempImpulse(r_b, -impulse_vector);
        }
    }

    void FrictionContactConstraint::initPrimalDual(const ConstraintParam &param) {
        _nsf = new NonSmoothContactForce;

        // map object to index
        _object2index.clear();
        for (size_t i = 0; i < _objects->size(); i++) {
            _object2index[(*_objects)[i]] = i;
        }

        // scalar values
        _contact_size = 0;
        for (const auto cr : *_contact_results) {
            _contact_size += cr->getPointSize();
        }
        _n_objects = _objects->size();
        _n_rigid_dof = _n_objects * 6;
        _n_force_dof = _contact_size * _nsf->dimensions();
        _n_constraint_dof = _contact_size * _nsf->constraintPerForce();
        _n_unknowns = _n_rigid_dof + _n_force_dof + _n_constraint_dof;

        // the unknowns are [vel, force, lambda]
        _vel_old = pe::VectorX(_n_rigid_dof);
        _vel = pe::VectorX(_n_rigid_dof);
        _forces = pe::VectorX(_n_force_dof, 0);
        _lambda = pe::VectorX(_n_constraint_dof, 0);

        // the linear systems
        _rhs = pe::VectorX(_n_rigid_dof);
        _A = pe::SparseMatrix(_n_rigid_dof, _n_rigid_dof);

        // property matrices
        _mass_mat = pe::SparseMatrix(_n_rigid_dof, _n_rigid_dof);

        // init the velocity vectors
        _dt = param.dt;
        PrimalDualUtils::initVelocity(*_objects, param.gravity, _dt, _vel_old, _vel);
        PrimalDualUtils::nonDimensionalParams(_dt, param.gravity, *_objects, *_contact_results, _char_mass, _char_speed);
        _vel_old /= _char_speed;
        _vel /= _char_speed;

        // init the mass matrix
        PrimalDualUtils::initMassMatrix(*_objects, _char_mass, _mass_mat);

        // init the sparse matrix structure
        pe::Set<pe::KV<size_t, size_t>> obj_pairs;
        pe::Array<Eigen::Triplet<pe::Real>> triplets;
        _nsf->linearSystemReserve(*_contact_results, *_objects, _object2index, obj_pairs, triplets);
        _A.setFromTriplets(triplets.begin(), triplets.end());
        for (int k = 0; k < _A.outerSize(); ++k) {
            for (pe::SparseMatrix::InnerIterator it(_A, k); it; ++it) {
                _mat_pointers[{it.row(), it.col()}] = &it.valueRef();
            }
        }

        // init forces and lambda
        _nsf->preprocess(*_contact_results, _contact_size, *_objects, _object2index, _vel, _dt, _char_mass, _char_speed);
        _nsf->initForces(_forces, _lambda);
    }

    bool FrictionContactConstraint::iteratePrimalDual(int iter, pe::LDLT& ldlt, pe::CG& cg, pe::Real& hu,
                                                      pe::VectorX& du, pe::VectorX& df, pe::VectorX& dl,
                                                      pe::VectorX& ru, pe::VectorX& ru_add,
                                                      pe::VectorX& rf, pe::VectorX& wrf, pe::VectorX& rl,
                                                      pe::Real& exit_err, pe::Real tol) {
        _nsf->calcConstraints(_vel, _forces, _lambda);
        const pe::Real s_dual_gap = _nsf->surrogateDualGap(_lambda);

        const pe::Real mu = s_dual_gap * PE_R(0.1);
        PrimalDualUtils::calcResiduals(true, *_contact_results, _contact_size, *_objects, _object2index,
                                       _vel, _forces, _lambda, _vel_old, _mass_mat, _char_mass, _char_speed,
                                       _dt, mu, ru, rf, wrf, rl, _nsf);

        const pe::VectorX ac_vec = _nsf->ACVector(*_contact_results, *_objects, _object2index, _vel, _forces);
        const pe::Real err = ru.squaredNorm() + wrf.squaredNorm() + rl.squaredNorm();
        const pe::Real s_err = ru.squaredNorm() + wrf.squaredNorm() + rl.squaredNorm();
        const pe::Real sac_err = ru.squaredNorm() + ac_vec.squaredNorm() + rl.squaredNorm();
        const pe::Real eps = PE_R(1e-4) * PE_POW(err, PE_R(0.333333));
        const pe::Real m_err = ru.squaredNorm() / _n_rigid_dof;
        exit_err = PE_MAX(m_err, wrf.norm() / _n_force_dof);
        exit_err = PE_MAX(exit_err, rl.norm() / _n_constraint_dof);
        exit_err = PE_MAX(exit_err, s_dual_gap);
        exit_err = PE_MIN(exit_err, PE_MAX(m_err, ac_vec.norm() / _contact_size));
        if (exit_err < tol) {
            return false;
        }

        _rhs.setConstant(0);
        for (int k = 0; k < _A.outerSize(); k++) {
            for (pe::SparseMatrix::InnerIterator it(_A, k); it; ++it) {
                it.valueRef() = 0;
            }
        }
        for (int k = 0; k < _mass_mat.outerSize(); k++) {
            for (pe::SparseMatrix::InnerIterator it(_mass_mat, k); it; ++it) {
                *_mat_pointers[{it.row(), it.col()}] += it.value();
                if (it.row() == it.col()) {
                    *_mat_pointers[{it.row(), it.col()}] += eps;
                }
            }
        }

        // calculate the linear system
        _nsf->linearSystemAddition(*_contact_results, _contact_size, *_objects, _object2index, _lambda, rf, rl, eps, _rhs, _mat_pointers);

        _rhs -= ru;

        pe::VectorX d(_A.rows());
        for (size_t i = 0; i < _n_objects; i++) {
            if ((*_objects)[i]->isKinematic()) {
                d.segment<6>(i * 6).setConstant(0);
            }
        }
        // scale linear system for better convergence
        for (int k = 0; k < _A.outerSize(); k++) {
            for (pe::SparseMatrix::InnerIterator it(_A, k); it; ++it) {
                if (it.row() == it.col()) {
                    d[k] = PE_R(1) / PE_SQRT(it.value());
                }
            }
        }
        for (int k = 0; k < _A.outerSize(); k++) {
            for (pe::SparseMatrix::InnerIterator it(_A, k); it; ++it) {
                it.valueRef() *= d[it.row()] * d[it.col()];
            }
        }
        _rhs = _rhs.cwiseProduct(d);

        size_t max_cg_it = 1000;
        size_t iters = 0;
        pe::Real lin_err;

        if (iter == 0) {
            ldlt.analyzePattern(_A);
        }
        ldlt.factorize(_A);
        if (ldlt.info() == Eigen::Success) {
            du = ldlt.solve(_rhs);
        }
        else {
            du.setZero();
        }
        
        /*if (_objects->size() < 2000) {
            if (iter == 0) {
                ldlt.analyzePattern(_A);
            }
            ldlt.factorize(_A);
            if (ldlt.info() == Eigen::Success) {
                du = ldlt.solve(_rhs);
            }
            else {
                du.setZero();
            }
        } else {
            typedef amgcl::static_matrix<pe::Real, 6, 6> dmat_type;
            typedef amgcl::static_matrix<pe::Real, 6, 1> dvec_type;
            typedef amgcl::backend::eigen<pe::Real> backend;
            typedef amgcl::make_solver <
                amgcl::amg<
                    backend,
                    amgcl::coarsening::smoothed_aggregation,
                    amgcl::relaxation::gauss_seidel
                >,
                amgcl::solver::cg<backend>
            > amg_solver;

            auto p_rhs = reinterpret_cast<dvec_type*>(_rhs.data());
            auto end_rhs = p_rhs + _objects->size();
            auto p_x = reinterpret_cast<dvec_type*>(du.data());
            auto end_x = p_x + _objects->size();
            auto b = amgcl::make_iterator_range<dvec_type*>(p_rhs, end_rhs);
            auto x = amgcl::make_iterator_range<dvec_type*>(p_x, end_x);
            amg_solver::params prm;
            auto a_b = amgcl::adapter::block_matrix<dmat_type>(_A);
            prm.solver.maxiter = max_cg_it;
            amg_solver amgs(a_b, prm);
            std::tie(iters, lin_err) = amgs(b, x);
            if (!std::isfinite(lin_err)) {
                du = cg.compute(_A).solve(_rhs);
                iters = cg.iterations();
                lin_err = cg.error();
            }
        }*/
        du = du.cwiseProduct(d);

        _nsf->retrieveNonSmoothForceInc(*_contact_results, _contact_size, *_objects, _object2index, _lambda, du, rf, rl, mu, df, dl);

        auto step_search = PrimalDualUtils::lineSearch(*_contact_results, _contact_size, *_objects, _object2index, _mass_mat, 
            _vel, _forces, _lambda, du, df, dl, _vel_old, s_err, sac_err, mu, _dt, _char_speed, _char_mass, 20, _nsf);

        bool use_gd = false;
        if (step_search.stepSize == 0) {
            use_gd = true;
            du = -ru;
            df - rf;
            dl = rl;
            step_search = PrimalDualUtils::lineSearch(*_contact_results, _contact_size, *_objects, _object2index, _mass_mat,
                _vel, _forces, _lambda, du, df, dl, _vel_old, s_err, sac_err, mu, _dt, _char_speed, _char_mass, 20, _nsf);
        }

        _vel = step_search.newU;
        _forces = step_search.newF;
        _lambda = step_search.newL;

        if (!use_gd && step_search.stepSize == 1) {
            if (iters == max_cg_it) {
                hu *= PE_R(1.3);
            } else {
                hu /= PE_R(4);
            }
        } else {
            hu *= PE_R(4);
        }
        hu = PE_MAX(hu, PE_R(1e-6));

        // apply the new velocity to the objects
        for (size_t i = 0; i < _n_objects; i++) {
            (*_objects)[i]->setTempLinearVelocity(_vel.segment<3>(i * 6));
            (*_objects)[i]->setTempAngularVelocity(_vel.segment<3>(i * 6 + 3));
        }
    }

} // namespace pe_phys_constraint