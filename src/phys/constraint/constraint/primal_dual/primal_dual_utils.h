#pragma once

namespace pe_phys_constraint {

    class PrimalDualUtils {
    public:
        static void initVelocity(const pe::Array<pe_phys_object::RigidBody*>& objects,
                                 const pe::Vector3& gravity, pe::Real dt, pe::VectorX& vel_old, pe::VectorX& vel) {
            for (size_t i = 0; i < objects.size(); i++) {
                const pe_phys_object::RigidBody* obj = objects[i];
                const size_t offset = i * 6;
                const auto vel_ = obj->getLinearVelocity();
                const auto ang_vel = obj->getAngularVelocity();
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
            char_speed = PE_MAX(char_speed, gravity.norm() * dt * R(0.25));
            char_speed = PE_MAX(char_speed, R(1e-4));
        }

        static void initMassMatrix(const pe::Array<pe_phys_object::RigidBody*>& objects,
                                   pe::Real char_mass, pe::MatrixMN& m) {
            for (size_t i = 0; i < objects.size(); i++) {
                if (objects[i]->isKinematic()) {
                    for (int j = 0; j < 6; j++) {
                        const size_t index = i * 6 + j;
                        m(index, index) = 1;
                    }
                } else {
                    const size_t offset = i * 6;
                    const pe::Real mass = objects[i]->getMass();
                    m(offset, offset) = m(offset + 1, offset + 1) = m(offset + 2, offset + 2) = mass / char_mass;
                    const auto& inertia = objects[i]->getWorldInertia();
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            m(offset + 3 + i, offset + 3 + j) = inertia(i, j) / char_mass;
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
    };

} // namespace pe_phys_constraint