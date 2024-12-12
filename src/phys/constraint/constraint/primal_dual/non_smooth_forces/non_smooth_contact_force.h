#pragma once

#include "non_smooth_force_base.h"
#include "force_constraint/lorentz_circle_constraint.h"
#include "utils/logger.h"

namespace pe_phys_constraint {

    class NonSmoothContactForce : public NonSmoothForceBase {
    public:
        NonSmoothContactForce() {
            _fc = new LorentzCircleConstraint(friction);
        }
        virtual ~NonSmoothContactForce() {}

        int dimensions() const override { return 3; }
        int constraintPerForce() const override { return 2; }

        void preprocess(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                        size_t contact_size,
                        const pe::Array<pe_phys_object::RigidBody*>& objects,
                        const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                        const pe::VectorX& vel, pe::Real dt, pe::Real char_mass, pe::Real char_speed) override {
            // checked1
            _constraint_val.resize(2 * contact_size);
            _gradients.resize(contact_size, 2);
            _hessians.resize(contact_size * 2, 2);
            _non_smooth_k = pe::VectorX::Zero(contact_size * 3);
            _k_norm_inv.resize(contact_size);
            _k_mat_svd.resize(contact_size);
            _weight_mat.resize(contact_size);
            _inv_constraint_val = pe::VectorX::Zero(contact_size * 2);
            _current_avg_mass = char_mass;

            const pe::Real damp = spring_d * spring_k;
            const pe::Real tk = dt * spring_k;
            _compliance = PE_R(1.0) / (dt * (tk + damp));
            _reduction = tk / (tk + damp);

            size_t contact_i = 0;
            for (auto& contact : contacts) {
                const auto obj1 = object2index.at(contact->getObjectA());
                const auto obj2 = object2index.at(contact->getObjectB());
                for (int pi = 0; pi < contact->getPointSize(); pi++) {
                    const auto& cp = contact->getContactPoint(pi);
                    if (spring_based_force) {
                        const pe::Real red = -_reduction * cp.getDistanceNonNeg() / dt / char_speed;
                        _non_smooth_k[contact_i * 3] += red;
                    } else {
                        const pe::Vector3 hu = cp.toLocal(vel.segment<6>(obj1 * 6), vel.segment<6>(obj2 * 6));
                        _non_smooth_k[contact_i * 3] += hu[0] * restitution;
                    }
                    contact_i++;
                }
            }
            //PE_LOG_DEBUG << "non_smooth_k: " << _non_smooth_k.transpose() << std::endl;
        }

        void initForces(pe::VectorX& forces, pe::VectorX& lambda) override {
            // checked1
            forces.reshaped(3, forces.size() / 3).colwise() = pe::Vector3(1, 0, 0);
            lambda.setConstant(1e-2);
        }

        void nonSmoothResiduals(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                size_t contact_size,
                                const pe::Array<pe_phys_object::RigidBody*>& objects,
                                const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda,
                                bool use_stored_constraints,
                                pe::Real mu, pe::VectorX& ru, pe::VectorX& rf, pe::VectorX& rl) override {
            // checked1
            ru = pe::VectorX::Zero(objects.size() * 6);
            rf = pe::VectorX::Zero(contact_size * 3);
            rl = pe::VectorX::Zero(contact_size);

            size_t contact_i = 0;
            for (auto& contact : contacts) {
                const auto obj1 = object2index.at(contact->getObjectA());
                const auto obj2 = object2index.at(contact->getObjectB());
                const pe_phys_object::RigidBody* objs[2] = {contact->getObjectA(), contact->getObjectB()};
                const size_t obj_ids[2] = {obj1, obj2};
                //PE_LOG_DEBUG << "obj: " << obj1 << ", " << obj2 << ", " << !contact->getObjectA()->isKinematic() << ", " << !contact->getObjectB()->isKinematic() << PE_ENDL;
                for (int pi = 0; pi < contact->getPointSize(); pi++) {
                    const auto& cp = contact->getContactPoint(pi);
                    const pe::VectorX f = forces.segment<3>(contact_i * 3);
                    //PE_LOG_DEBUG << "f: " << f.transpose() << PE_ENDL;
                    rf.segment<3>(contact_i * 3) -= cp.toLocal(vel.segment<6>(obj1 * 6), vel.segment<6>(obj2 * 6));
                    for (int i = 0; i < 2; i++) {
                        pe::Vector6 htf = -cp.toGlobal(i, f);
                        //PE_LOG_DEBUG << "htf: " << htf.transpose() << PE_ENDL;
                        if (!objs[i]->isKinematic()) {
                            for (int rui = 0; rui < 6; rui++) {
                                ru[obj_ids[i] * 6 + rui] += htf[rui];
                            }
                        }
                    }
                    if (spring_based_force) {
                        rf[contact_i * 3] -= _compliance * _current_avg_mass * f[0];
                    }
                    rf[contact_i * 3] += lambda[contact_i * 2];
                    const pe::Vector2 grad = use_stored_constraints ?
                        _gradients.row(contact_i).transpose() :
                        _fc->gradient(f);
                    rf.segment<2>(contact_i * 3 + 1) -= lambda[contact_i * 2 + 1] * grad;

                    contact_i++;
                }
            }

            rf -= _non_smooth_k;
            if (use_stored_constraints) {
                rl = lambda.cwiseProduct(_constraint_val).array() + mu;
            }
            else {
                pe::VectorX c(contact_size * 2);
                for (size_t i = 0; i < contact_size; i++) {
                    const pe::VectorX f = forces.segment<3>(i * 3);
                    c[2 * i] = -f[0];
                    c[2 * i + 1] = _fc->constraint(f);
                }
                rl = lambda.cwiseProduct(c).array() + mu;
            }
        }

        void calcConstraints(const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda) override {
            // checked1
            for (size_t i = 0; i < forces.size() / 3; i++) {
                const pe::Vector3& f = forces.segment<3>(i * 3);
                _constraint_val[2 * i] = -f[0];
                _constraint_val[2 * i + 1] = _fc->constraint(f);
                _gradients.row(i) = _fc->gradient(f).transpose();
                _hessians.block<2, 2>(i * 2, 0) = _fc->hessian(f);
            }
        }

        pe::VectorX calcTangentWeight(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                      const pe::Array<pe_phys_object::RigidBody*>& objects,
                                      const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                      const pe::VectorX& vel, const pe::VectorX& forces,
                                      pe::Real char_mass) override {
            // checked1
            pe::VectorX f_weight = pe::VectorX::Ones(forces.size());
            pe::Real div = 0;
            size_t contact_i = 0;
            for (auto& contact : contacts) {
                const auto obj1 = object2index.at(contact->getObjectA());
                const auto obj2 = object2index.at(contact->getObjectB());
                const pe_phys_object::RigidBody* objs[2] = {contact->getObjectA(), contact->getObjectB()};
                for (int pi = 0; pi < contact->getPointSize(); pi++) {
                    auto& cp = contact->getContactPoint(pi);
                    pe::Vector2 tangent_vel = cp.toTangent(vel.segment<6>(obj1 * 6), vel.segment<6>(obj2 * 6));
                    pe::Real friction_weight = 0;
                    for (int i = 0; i < 2; i++) {
                        if (!objs[i]->isKinematic()) {
                            pe::Vector6 hh = -cp.toGlobalTangent(i, tangent_vel.tail<2>().normalized());
                            friction_weight += hh.dot(objs[i]->getInvMassMatrix6x6() * hh) * char_mass;
                        }
                    }
                    const pe::Real friction_radius = _fc->radius_at(forces[3 * contact_i]);
                    const pe::Real contact_weight = PE_MIN(friction_radius / (tangent_vel.norm() + PE_R(1e-8)) * friction_weight, PE_R(1));
                    f_weight.segment<2>(3 * contact_i + 1).array() = contact_weight;
                    div = PE_MAX(div, contact_weight);
                    contact_i++;
                }
            }
            return std::move(f_weight);
        }

        void linearSystemAddition(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                  size_t contact_size,
                                  const pe::Array<pe_phys_object::RigidBody*>& objects,
                                  const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                  const pe::VectorX& lambda, const pe::VectorX& rf, const pe::VectorX& rl,
                                  pe::Real eps, pe::VectorX& y,
                                  pe::Map<pe::KV<size_t, size_t>, pe::Real*>& mat_pointers) override {
            // checked1
            _sf = rf;
            pe::Array<pe::Matrix2> l_mats(contact_size, pe::Matrix2::Zero());
            pe::VectorX l_norms(contact_size);
            pe::VectorX sf_add(rf.size());
            sf_add.setZero();
            _inv_constraint_val = PE_R(1) / (_constraint_val.array() - eps * lambda.array());

            for (size_t i = 0; i < contact_size; i++) {
                const pe::Vector2& grad = _gradients.row(i).transpose();
                l_norms[i] = -lambda[2 * i] * _inv_constraint_val[2 * i];
                if (spring_based_force) {
                    l_norms[i] += _compliance * _current_avg_mass;
                }
                sf_add[3 * i] = -rl[2 * i] * _inv_constraint_val[2 * i];
                l_mats[i] = -lambda[2 * i + 1] * _inv_constraint_val[2 * i + 1] * grad * grad.transpose();
                sf_add.segment<2>(3 * i + 1) = grad * rl[2 * i + 1] * _inv_constraint_val[2 * i + 1];
            }
            _sf += sf_add;

            size_t contact_i = 0;
            for (auto& contact : contacts) {
                const auto obj1 = object2index.at(contact->getObjectA());
                const auto obj2 = object2index.at(contact->getObjectB());
                const pe_phys_object::RigidBody* objs[2] = { contact->getObjectA(), contact->getObjectB() };
                const size_t obj_ids[2] = { obj1, obj2 };
                for (int pi = 0; pi < contact->getPointSize(); pi++) {
                    auto& cp = contact->getContactPoint(pi);

                    pe::Real reg_n = eps + l_norms[contact_i];
                    _k_norm_inv[contact_i] = PE_R(1) / reg_n;
                    pe::Matrix2 reg = pe::Matrix2::Identity() * eps;
                    reg += l_mats[contact_i];
                    reg += lambda[2 * contact_i + 1] * _hessians.block<2, 2>(contact_i * 2, 0);
                    _k_mat_svd[contact_i].compute(reg, Eigen::ComputeFullU | Eigen::ComputeFullV);

                    pe::Vector3 k_inv_sf = pe::Vector3::Zero();
                    k_inv_sf[0] = _k_norm_inv[contact_i] * _sf[3 * contact_i];
                    k_inv_sf.tail<2>() = _k_mat_svd[contact_i].solve(_sf.segment<2>(3 * contact_i + 1));

                    for (int j = 0; j < 2; j++) {
                        if (objs[j]->isKinematic()) {
                            continue;
                        }
                        const auto& trans1 = cp.getTrans(j);
                        pe::Vector6 yadd_block = trans1 * k_inv_sf;
                        for (int k = 0; k < 6; k++) {
                            y[obj_ids[j] * 6 + k] += yadd_block[k];
                        }

                        for (int k = 0; k < 2; k++) {
                            if (objs[k]->isKinematic()) {
                                continue;
                            }
                            const auto& trans2 = cp.getTrans(k);
                            pe::Matrix6 mass_block = trans1.col(0) * _k_norm_inv[contact_i] * trans2.col(0).transpose();
                            mass_block += trans1.block<6, 2>(0, 1) * _k_mat_svd[contact_i].solve(trans2.block<6, 2>(0, 1).transpose());

                            for (int row = 0; row < 6; row++) {
                                for (int col = 0; col < 6; col++) {
                                    (*mat_pointers[{obj_ids[j] * 6 + row, obj_ids[k] * 6 + col}]) += mass_block(row, col);
                                }
                            }
                        }
                    }

                    contact_i++;
                }
            }
        }

        void retrieveNonSmoothForceInc(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                       size_t contact_size,
                                       const pe::Array<pe_phys_object::RigidBody*>& objects,
                                       const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                       const pe::VectorX& lambda, const pe::VectorX& du, const pe::VectorX& rf, const pe::VectorX& rl,
                                       pe::Real mu, pe::VectorX& df, pe::VectorX& dl) override {
            // checked1
            df = pe::VectorX::Zero(contact_size * 3);
            dl = pe::VectorX::Zero(contact_size * 2);
            size_t contact_i = 0;
            for (auto& contact : contacts) {
                const auto obj1 = object2index.at(contact->getObjectA());
                const auto obj2 = object2index.at(contact->getObjectB());
                for (int pi = 0; pi < contact->getPointSize(); pi++) {
                    const auto& cp = contact->getContactPoint(pi);
                    pe::VectorX dfi = _sf.segment<3>(contact_i * 3);
                    dfi -= cp.toLocal(du.segment<6>(obj1 * 6), du.segment<6>(obj2 * 6));
                    dfi[0] = _k_norm_inv[contact_i] * dfi[0];
                    dfi.tail<2>() = _k_mat_svd[contact_i].solve(dfi.tail<2>());
                    df.segment<3>(contact_i * 3) = dfi;
                    dl(2 * contact_i) = -_inv_constraint_val[2 * contact_i] * (-lambda[2 * contact_i] * dfi[0] + rl[2 * contact_i]);
                    dl(2 * contact_i + 1) = -_inv_constraint_val[2 * contact_i + 1] * 
                        (lambda[2 * contact_i + 1] * _gradients.row(contact_i).dot(dfi.tail(2)) + rl[2 * contact_i + 1]);
                    contact_i++;
                }
            }
        }

        bool filterLineSearch(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                              size_t contact_size,
                              const pe::Array<pe_phys_object::RigidBody*>& objects,
                              const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                              const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda,
                              pe::Real mu, pe::Real char_mass, pe::VectorX& du, pe::VectorX& df, pe::VectorX& dl) override {
            // checked1
            bool projected = false;
            for (size_t i = 0; i < contact_size; i++) {
                pe::VectorX cf = forces.segment<3>(i * 3);
                pe::VectorX dfi = df.segment<3>(i * 3);
                pe::VectorX tf = cf + dfi;

                if (tf[0] < 0) {
                    tf[0] = 0;
                    df.segment<3>(i * 3) = tf - cf;
                    projected = true;
                }

                const pe::Real co = _fc->constraint(tf);
                if (co > 0) {
                    pe::VectorX pf = _fc->project(tf, tf);
                    df.segment<3>(i * 3) = pf - cf;
                    projected = true;
                }
            }

            for (size_t i = 0; i < 2 * contact_size; i++) {
                if (dl[i] < 0 && lambda[i] + dl[i] < 0) {
                    dl[i] -= lambda[i];
                }
            }

            return projected;
        }

        pe::VectorX ACVector(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                             const pe::Array<pe_phys_object::RigidBody*>& objects,
                             const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                             const pe::VectorX& vel, const pe::VectorX& forces) override {
            // checked1
            pe::VectorX ac = pe::VectorX::Zero(forces.size());
            size_t contact_i = 0;
            for (auto& contact : contacts) {
                const auto obj1 = object2index.at(contact->getObjectA());
                const auto obj2 = object2index.at(contact->getObjectB());
                for (int pi = 0; pi < contact->getPointSize(); pi++) {
                    const auto& cp = contact->getContactPoint(pi);
                    const pe::VectorX local_u = cp.toLocal(vel.segment<6>(obj1 * 6), vel.segment<6>(obj2 * 6));
                    const pe::Real to_project_n = forces[3 * contact_i] - (local_u[0] + _non_smooth_k[3 * contact_i] +
                        spring_based_force * _compliance * _current_avg_mass * forces[3 * contact_i]);
                    ac[3 * contact_i] = PE_MAX(to_project_n, 0) - forces[3 * contact_i];
                    const pe::Real r = _fc->radius_at(forces[3 * contact_i]);
                    const pe::Vector2 local_tangent = forces.segment<2>(3 * contact_i + 1);
                    const pe::Vector2 t_diff = local_tangent - local_u.tail<2>();
                    const pe::Vector2 proj = (t_diff.norm() < r) ? t_diff : t_diff * r / t_diff.norm();
                    ac.segment<2>(3 * contact_i + 1) = proj - local_tangent;
                    contact_i++;
                }
            }
            return std::move(ac);
        }

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