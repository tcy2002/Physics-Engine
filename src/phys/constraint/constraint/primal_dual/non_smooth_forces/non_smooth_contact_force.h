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
                        const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
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

            const pe::Real damp = spring_d * spring_k;
            const pe::Real tk = dt * spring_k;
            _compliance = R(1.0) / (dt * (tk + damp));
            _reduction = tk / (tk + damp);
        }

        void initForces(pe::VectorX &forces, pe::VectorX &lambda) override {
            for (size_t i = 0; i < forces.size(); i += 3) {
                forces[i] = 1;
            }
            lambda.setValue(1e-2);
        }

        void nonSmoothResiduals(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                size_t contact_size,
                                const pe::Array<pe_phys_object::RigidBody*>& objects,
                                const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda,
                                bool use_stored_constraints,
                                pe::Real mu, pe::VectorX& ru, pe::VectorX& rf, pe::VectorX& rl) override {
            ru.resize(objects.size() * 6, 0);
            rf.resize(contact_size * 3, 0);
            rl.resize(contact_size, 0);

            size_t contact_i = 0;
            for (auto& contact : contacts) {
                const auto obj1 = object2index.at(contact->getObjectA());
                const auto obj2 = object2index.at(contact->getObjectB());
                const pe_phys_object::RigidBody* objs[2] = {contact->getObjectA(), contact->getObjectB()};
                const size_t obj_ids[2] = {obj1, obj2};
                for (int pi = 0; pi < contact->getPointSize(); pi++) {
                    const auto& cp = contact->getContactPoint(pi);
                    const pe::VectorX f = forces.getSubVector(3, contact_i * 3);
                    const pe::VectorX local_u = cp.toLocal(vel.getSubVector(6, obj1), vel.getSubVector(6, obj2));
                    rf.getRefSubVector(3, contact_i * 3) -= local_u;
                    for (int i = 0; i < 2; i++) {
                        pe::VectorX htf = -cp.toGlobal(i, f);
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
                    const pe::VectorX grad = use_stored_constraints ?
                        _gradients.getSubRowVector(2, contact_i, 0) :
                        _fc->gradient(f);
                    rf.getRefSubVector(2, contact_i * 3 + 1) -= lambda[contact_i * 2 + 1] * grad;
                    contact_i++;
                }

                rf -= _non_smooth_k;
                if (use_stored_constraints) {
                    rl = lambda.mult(_constraint_val) + pe::VectorX::ones(contact_size) * mu;
                } else {
                    pe::VectorX c(contact_size * 2);
                    for (size_t i = 0; i < contact_size; i++) {
                        const pe::VectorX f = forces.getSubVector(3, i * 3);
                        c[2 * i] = -f[0];
                        c[2 * i + 1] = _fc->constraint(f);
                    }
                    rl = lambda.mult(c) + pe::VectorX::ones(contact_size) * mu;
                }
            }
        }

        void calcConstraints(const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda) override {
            for (size_t i = 0; i < forces.size() / 3; i++) {
                const pe::VectorX f = forces.getSubVector(3, i * 3);
                _constraint_val[2 * i] = -f[0];
                _constraint_val[2 * i + 1] = _fc->constraint(f);
                _gradients.setSubVector(_fc->gradient(f), i, 0, true);
                _hessians.setSubMatrix(_fc->hessian(f), i * 2, 0);
            }
        }

        pe::VectorX calcTangentWeight(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                      const pe::Array<pe_phys_object::RigidBody*>& objects,
                                      const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                      const pe::VectorX& vel, const pe::VectorX& forces,
                                      pe::Real char_mass) override {
            pe::VectorX f_weight = pe::VectorX::ones(forces.size());
            pe::Real div = 0;
            size_t contact_i = 0;
            for (auto& contact : contacts) {
                const auto obj1 = object2index.at(contact->getObjectA());
                const auto obj2 = object2index.at(contact->getObjectB());
                const pe_phys_object::RigidBody* objs[2] = {contact->getObjectA(), contact->getObjectB()};
                for (int pi = 0; pi < contact->getPointSize(); pi++) {
                    auto& cp = contact->getContactPoint(pi);
                    auto tangent_vel = cp.toTangent(vel.getSubVector(6, obj1), vel.getSubVector(6, obj2));
                    pe::Real friction_weight = 0;
                    for (int i = 0; i < 2; i++) {
                        if (!contact->getObjectA()->isKinematic()) {
                            pe::VectorX hh = -cp.toGlobalTangent(i, tangent_vel.normalized());
                            friction_weight += hh.dot(objs[i]->getInvMassMatrix6x6() * hh) * char_mass;
                        }
                    }
                    const pe::Real friction_radius = _fc->radius_at(forces[3 * contact_i]);
                    const pe::Real contact_weight = PE_MIN(friction_radius / (tangent_vel.norm() + R(1e-8)) * friction_weight, R(1));
                    f_weight[3 * contact_i + 1] = contact_weight;
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
                                  pe::HashMap<pe::KV<pe_phys_object::RigidBody*, pe_phys_object::RigidBody*>, pe::Real>& mat_pointers) override {
            _sf = rf;
            pe::Array<pe::MatrixMN> l_mats(contact_size, pe::MatrixMN::zeros(2, 2));
            pe::VectorX l_norms(contact_size);
            pe::VectorX sf_add(0, rf.size());
            _inv_constraint_val = pe::VectorX::ones(contact_size).div(_constraint_val - lambda * eps);

            for (size_t i = 0; i < contact_size; i++) {
                pe::VectorX grad = _gradients.getSubRowVector(2, i, 0);
                l_norms[i] = -lambda[2 * i] * _inv_constraint_val[2 * i];
                if (spring_based_force) {
                    l_norms[i] += _compliance * _current_avg_mass;
                }
                sf_add[3 * i] = -rl[2 * i] * _inv_constraint_val[2 * i];
                pe::MatrixMN l_mat(2, 2);
                l_mat[0][0] = grad[0] * grad[0];
                l_mat[0][1] = l_mat[1][0] = grad[0] * grad[1];
                l_mat[1][1] = grad[1] * grad[1];
                l_mats[i] = -lambda[2 * i + 1] * _inv_constraint_val[2 * i + 1] * l_mat;
                sf_add.getRefSubVector(2, 3 * i + 1) = grad * rl[2 * i + 1] * _inv_constraint_val[2 * i + 1];
            }
            _sf += sf_add;

            for (size_t i = 0; i < contact_size; i++) {
                pe::Real reg_n = eps + l_norms[i];
                _k_norm_inv[i] = R(1) / reg_n;
                auto reg = pe::MatrixMN::identity(2) * eps;
                reg += l_mats[i];
                reg += lambda[2 * i + 1] * _hessians.getSubMatrix(2, 2, i * 2, 0);
                // TODO: solve svd
            }
        }

        void retrieveNonSmoothForceInc(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                       size_t contact_size,
                                       const pe::Array<pe_phys_object::RigidBody*>& objects,
                                       const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                       const pe::VectorX& lambda, const pe::VectorX& du, const pe::VectorX& rf, const pe::VectorX& rl,
                                       pe::Real mu, pe::VectorX& df, pe::VectorX& dl) override {
            df.resize(contact_size * 3, 0);
            dl.resize(contact_size * 2, 0);
            size_t contact_i = 0;
            for (auto& contact : contacts) {
                const auto obj1 = object2index.at(contact->getObjectA());
                const auto obj2 = object2index.at(contact->getObjectB());
                for (int pi = 0; pi < contact->getPointSize(); pi++) {
                    const auto& cp = contact->getContactPoint(pi);
                    pe::VectorX dfi = _sf.getSubVector(3, contact_i * 3);
                    dfi -= cp.toLocal(du.getSubVector(6, obj1), du.getSubVector(6, obj2));
                    dfi[0] = _k_norm_inv[contact_i] * dfi[0];
                    // TODO: solve svd
                    contact_i++;
                }
            }
        }

        bool filterLineSearch(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                              size_t contact_size,
                              const pe::Array<pe_phys_object::RigidBody*>& objects,
                              const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                              const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda,
                              pe::Real mu, pe::Real char_mass, pe::VectorX& du, pe::VectorX& df, pe::VectorX dl) override {
            bool projected = false;
            for (size_t i = 0; i < contact_size; i++) {
                pe::VectorX cf = forces.getSubVector(3, i * 3);
                pe::VectorX dfi = df.getSubVector(3, i * 3);
                pe::VectorX tf = cf + dfi;

                if (tf[0] < 0) {
                    tf[0] = 0;
                    df.getRefSubVector(3, i * 3) = tf - cf;
                    projected = true;
                }

                const pe::Real co = _fc->constraint(tf);
                if (co > 0) {
                    pe::VectorX pf = _fc->project(tf, tf);
                    df.getRefSubVector(3, i * 3) = pf - cf;
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
                             const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda) override {
            pe::VectorX ac(forces.size());
            int contact_index = 0;
            for (auto& contact : contacts) {
                const auto obj1 = object2index.at(contact->getObjectA());
                const auto obj2 = object2index.at(contact->getObjectB());
                for (int pi = 0; pi < contact->getPointSize(); pi++) {
                    const auto& cp = contact->getContactPoint(pi);
                    const pe::VectorX local_u = cp.toLocal(vel.getSubVector(6, obj1), vel.getSubVector(6, obj2));
                    const int index = 3 * contact_index;
                    const pe::Real to_project_n = forces[index] - (local_u[0] + _non_smooth_k[index] +
                        spring_based_force * _compliance * _current_avg_mass * forces[index]);
                    ac[index] = PE_MAX(to_project_n, 0) - forces[index];
                    const pe::Real r = _fc->radius_at(forces[index]);
                    const pe::Real tangent0 = forces[index + 1], tangent1 = forces[index + 2];
                    const pe::Real t_diff0 = tangent0 - local_u[1], t_diff1 = tangent1 - local_u[2];
                    const pe::Real r_t = PE_SQRT(t_diff0 * t_diff0 + t_diff1 * t_diff1);
                    const pe::Real proj0 = r_t < r ? t_diff0 : t_diff0 * r / r_t, proj1 = r_t < r ? t_diff1 : t_diff1 * r / r_t;
                    ac[index + 1] = proj0 - tangent0;
                    ac[index + 2] = proj1 - tangent1;
                    ++contact_index;
                }
            }
            return std::move(ac);
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
        //pe::Array<JacobiSVD> _k_mat_svd;
        pe::Real _current_avg_mass;
    };

} // namespace pe_phys_constraint