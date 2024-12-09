#pragma once

namespace pe_phys_constraint {

    class PrimalDualUtils {
    public:
        static void initVelocity(const pe::Array<pe_phys_object::RigidBody*>& objects,
                                 const pe::Vector3& gravity, pe::Real dt, pe::VectorX& vel_old, pe::VectorX& vel) {
            for (size_t i = 0; i < objects.size(); i++) {
                auto obj = objects[i];
                const size_t offset = i * 6;
                const auto& vel_ = obj->getTempLinearVelocity();
                const auto& ang_vel = obj->getTempAngularVelocity();
                const pe::Vector3 acc = gravity * dt;
                vel_old[offset] = vel[offset] = vel_.x();
                vel_old[offset + 1] = vel[offset + 1] = vel_.y();
                vel_old[offset + 2] = vel[offset + 2] = vel_.z();
                vel_old[offset + 3] = vel[offset + 3] = ang_vel.x();
                vel_old[offset + 4] = vel[offset + 4] = ang_vel.y();
                vel_old[offset + 5] = vel[offset + 5] = ang_vel.z();
                vel_old[offset] += acc.x();
                vel_old[offset + 1] += acc.y();
                vel_old[offset + 2] += acc.z();
            }
        }

        static void nonDimensionalParams(
                pe::Real dt,
                const pe::Vector3& gravity,
                const pe::Array<pe_phys_object::RigidBody*>& objects,
                const pe::Array<pe_phys_collision::ContactResult*>& contact_results,
                pe::Real& char_mass, pe::Real& char_speed) {
            char_mass = 0;
            int active_objects = 0;
            for (const auto obj : objects) {
                if (!obj->isKinematic()) {
                    char_mass += obj->getMass() * 3;
                    const auto& inertia = obj->getWorldInertia();
                    char_mass += inertia.trace();
                    active_objects++;
                }
            }
            char_mass = active_objects > 0 ? char_mass / active_objects : 1;

            pe::HashMap<pe_phys_object::RigidBody*, pe::Real> obj_speed;
            for (const auto cr : contact_results) {
                for (int i = 0; i < cr->getPointSize(); i++) {
                    auto& p = cr->getContactPoint(i).getWorldPos();
                    auto& trans_a = cr->getObjectA()->getTransform();
                    auto& trans_b = cr->getObjectB()->getTransform();
                    auto vel_a = cr->getObjectA()->getLinearVelocityAtLocalPoint(trans_a.inverseTransform(p));
                    vel_a += gravity * dt;
                    auto vel_b = cr->getObjectB()->getLinearVelocityAtLocalPoint(trans_b.inverseTransform(p));
                    vel_b += gravity * dt;
                    obj_speed[cr->getObjectA()] = PE_MAX(obj_speed[cr->getObjectA()], vel_a.norm());
                    obj_speed[cr->getObjectB()] = PE_MAX(obj_speed[cr->getObjectB()], vel_b.norm());
                }
            }

            char_speed = 0;
            pe::Real mass_sum = 0;
            for (const auto& kv : obj_speed) {
                if (!kv.first->isKinematic()) {
                    char_speed += kv.second;
                    mass_sum += kv.first->getMass();
                }
            }
            char_speed /= mass_sum;
            char_speed = PE_MAX(char_speed, gravity.norm() * dt * PE_R(0.25));
            char_speed = PE_MAX(char_speed, PE_R(1e-4));
        }

        static void initMassMatrix(const pe::Array<pe_phys_object::RigidBody*>& objects,
                                   pe::Real char_mass, pe::SparseMatrix& m) {
            pe::Array<Eigen::Triplet<pe::Real>> mass_triplets;
            for (int i = 0; i < PE_I(objects.size()); i++) {
                if (objects[i]->isKinematic()) {
                    for (int j = 0; j < 6; j++) {
                        mass_triplets.push_back({ i * 6 + j, i * 6 + j, 1 });
                    }
                } else {
                    pe::Matrix6 masses = objects[i]->getMassMatrix6x6() / char_mass;
                    for (int j = 0; j < 6; j++) {
                        for (int k = 0; k < 6; k++) {
                            if (masses(j, k)) {
                                mass_triplets.push_back({ i * 6 + j, i * 6 + k, masses(j, k) });
                            }
                        }
                    }
                }
            }
            m.setFromTriplets(mass_triplets.begin(), mass_triplets.end());
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
            pe::VectorX ru_add;
            ru = mass_mat * (vel - vel_old);
            for (size_t i = 0; i < objects.size(); i++) {
                if (objects[i]->isKinematic()) {
                    ru.segment<6>(i * 6).setConstant(0);
                }
            }
            pe::VectorX f_weight = _nsf->calcTangentWeight(contacts, objects, object2index, vel, forces, char_mass);
            _nsf->nonSmoothResiduals(contacts, contact_size, objects, object2index,
                vel, forces, lambda, use_stored_constraints, mu, ru_add, rf, rl);
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
            pe::Real step = 1.0;
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