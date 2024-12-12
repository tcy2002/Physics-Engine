#pragma once

#include "utils/logger.h"

namespace pe_phys_constraint {

    class PrimalDualUtils {
    public:
        static void initVelocity(const pe::Array<pe_phys_object::RigidBody*>& objects,
                                 const pe::Vector3& gravity, pe::Real dt, pe::VectorX& vel_old, pe::VectorX& vel) {
            // checked1
            for (size_t i = 0; i < objects.size(); i++) {
                auto obj = objects[i];
                const pe::Vector3 acc = gravity * dt;
                vel.segment<6>(i * 6) << obj->getLinearVelocity(), obj->getAngularVelocity();
                vel_old.segment<6>(i * 6) = vel.segment<6>(i * 6);
                if (!obj->isKinematic()) {
                    vel_old.segment<3>(i * 6) += acc;
                }
            }
        }

        static void nonDimensionalParams(
                pe::Real dt,
                const pe::Vector3& gravity,
                const pe::Array<pe_phys_object::RigidBody*>& objects,
                const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                const pe::Array<pe_phys_collision::ContactResult*>& contact_results,
                pe::Real& char_mass, pe::Real& char_speed) {
            // checked1
            char_mass = 0;
            int active_objects = 0;
            for (const auto obj : objects) {
                if (!obj->isKinematic()) {
                    char_mass += obj->getMassMatrix6x6().trace();
                    active_objects++;
                }
            }
            char_mass = active_objects > 0 ? char_mass / active_objects / 6 : 1;

            pe::VectorX obj_speed = pe::VectorX::Zero(objects.size());
            for (const auto contact : contact_results) {
                const auto obj1 = object2index.at(contact->getObjectA());
                const auto obj2 = object2index.at(contact->getObjectB());
                const pe_phys_object::RigidBody* objs[2] = { contact->getObjectA(), contact->getObjectB() };
                const size_t obj_ids[2] = { obj1, obj2 };
                for (int pi = 0; pi < contact->getPointSize(); pi++) {
                    auto& cp = contact->getContactPoint(pi);
                    bool has_static = false;
                    pe::Vector3 rel_vel = pe::Vector3::Zero();
                    for (int j = 0; j < 2; j++) {
                        pe::Vector6 vel_ext;
                        vel_ext << objs[j]->getLinearVelocity(), objs[j]->getAngularVelocity();
                        if (!objs[j]->isKinematic()) {
                            vel_ext.head<3>() += gravity * dt;
                        } else {
                            has_static = true;
                        }
                        pe::Vector3 rel_vel_half = cp.toLocal(j, vel_ext);
                        rel_vel += rel_vel_half;
                        obj_speed[obj_ids[j]] = PE_MAX(obj_speed[obj_ids[j]], rel_vel_half.norm());
                    }
                    if (has_static) {
                        for (int j = 0; j < 2; ++j) {
                            obj_speed[obj_ids[j]] = PE_MAX(obj_speed[obj_ids[j]], rel_vel.norm());
                        }
                    }
                }
            }

            char_speed = 0;
            pe::Real mass_sum = 0;
            for (size_t i = 0; i < objects.size(); i++) {
                if (!objects[i]->isKinematic()) {
                    char_speed += obj_speed[i] * objects[i]->getMass();
                    mass_sum += objects[i]->getMass();
                }
            }
            char_speed /= mass_sum;
            char_speed = PE_MAX(char_speed, gravity.norm() * dt * PE_R(0.25));
            char_speed = PE_MAX(char_speed, PE_R(1e-4));
        }

        static void initMassTriplets(const pe::Array<pe_phys_object::RigidBody*>& objects,
                                   pe::Real char_mass, pe::Array<Eigen::Triplet<pe::Real>>& triplets) {
            // checked1
            for (int i = 0; i < PE_I(objects.size()); i++) {
                if (objects[i]->isKinematic()) {
                    for (int j = 0; j < 6; j++) {
                        triplets.push_back({ i * 6 + j, i * 6 + j, 1 });
                    }
                } else {
                    pe::Matrix6 masses = objects[i]->getMassMatrix6x6() / char_mass;
                    for (int j = 0; j < 6; j++) {
                        for (int k = 0; k < 6; k++) {
                            if (masses(j, k)) {
                                triplets.push_back({ i * 6 + j, i * 6 + k, masses(j, k) });
                            }
                        }
                    }
                }
            }
        }

        static void calcResiduals(
                bool use_stored_constraints,
                const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                size_t contact_size,
                const pe::Array<pe_phys_object::RigidBody*>& objects,
                const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda, const pe::VectorX& vel_old,
                const pe::MatrixMN& mass_mat, pe::Real char_mass, pe::Real char_speed, pe::Real dt, pe::Real mu,
                pe::VectorX& ru, pe::VectorX& rf, pe::VectorX& wrf, pe::VectorX& rl,
                NonSmoothForceBase* _nsf) {
            // checked1
            pe::VectorX ru_add;
            ru = mass_mat * (vel - vel_old);
            //PE_LOG_DEBUG << "ru: " << ru.transpose() << PE_ENDL;
            for (size_t i = 0; i < objects.size(); i++) {
                if (objects[i]->isKinematic()) {
                    ru.segment<6>(i * 6).setZero();
                }
            }
            pe::VectorX f_weight = _nsf->calcTangentWeight(contacts, objects, object2index, vel, forces, char_mass);
            _nsf->nonSmoothResiduals(contacts, contact_size, objects, object2index,
                vel, forces, lambda, use_stored_constraints, mu, ru_add, rf.derived(), rl.derived());
            //PE_LOG_DEBUG << "ru_add: " << ru_add.transpose() << PE_ENDL;
            ru += ru_add;

            wrf = rf.cwiseProduct(f_weight);
        }

        struct LineSearchResult {
            pe::VectorX newU;
            pe::VectorX newF;
            pe::VectorX newL;
            pe::Real stepSize;
            pe::Real sErr;
            pe::Real acErr;
        };

        static LineSearchResult lineSearch(
                const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                size_t contact_size,
                const pe::Array<pe_phys_object::RigidBody*>& objects,
                const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                const pe::SparseMatrix& mass_mat,
                const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda,
                const pe::VectorX& du, const pe::VectorX& df, const pe::VectorX& dl,
                const pe::VectorX& vels_old, pe::Real s_err, pe::Real sac_err, pe::Real mu, pe::Real dt,
                pe::Real char_speed, pe::Real char_mass, int max_linear_search,
                NonSmoothForceBase* _nsf) {
            // checked1
            pe::Real step = PE_R(1.0);
            LineSearchResult res;
            res.newU = vel;
            res.newF = forces;
            res.newL = lambda;
            res.sErr = s_err;
            res.acErr = sac_err;
            res.stepSize = 0;
            for (int step_it = 0; step_it < max_linear_search; step_it++) {
                pe::VectorX du_in = step * du;
                pe::VectorX df_in = step * df;
                pe::VectorX dl_in = step * dl;

                _nsf->filterLineSearch(contacts, contact_size, objects, object2index, vel, forces, lambda, mu, char_mass, du_in, df_in, dl_in);
                
                const pe::VectorX l_inner = lambda + PE_R(0.99) * dl_in;
                const pe::VectorX v_inner = vel + PE_R(0.99) * du_in;
                const pe::VectorX f_inner = forces + PE_R(0.99) * df_in;

                pe::VectorX ru_in, rf_in, wrf_in, rl_in;
                calcResiduals(false, contacts, contact_size, objects, object2index, v_inner, f_inner, l_inner, vels_old, mass_mat, 
                    char_mass, char_speed, dt, mu, ru_in, rf_in, wrf_in, rl_in, _nsf);

                const pe::Real in_err = ru_in.squaredNorm() + wrf_in.squaredNorm() + rl_in.squaredNorm();
                const pe::VectorX ac_vec_in = _nsf->ACVector(contacts, objects, object2index, v_inner, f_inner);
                const pe::Real sac_err_in = ru_in.squaredNorm() + ac_vec_in.squaredNorm() + rl_in.squaredNorm();
                const pe::Real g_err = -du_in.dot(ru_in) + df_in.dot(rf_in) + dl_in.cwiseQuotient(l_inner).dot(rl_in);

                bool in_err_acpt = in_err < s_err;
                bool ac_err_acpt = sac_err_in < sac_err;
                bool acpt = in_err_acpt || ac_err_acpt;
                if ((acpt && g_err > 0) || (step_it == max_linear_search - 1)) {
                    if (acpt) {
                        res.newU = v_inner;
                        res.newF = f_inner;
                        res.newL = l_inner;
                        res.stepSize = step;
                        res.sErr = in_err;
                        res.acErr = sac_err_in;
                        return res;
                    }
                    break;
                }
                step *= PE_R(0.5);
            }
            return res;
        }
    };

} // namespace pe_phys_constraint