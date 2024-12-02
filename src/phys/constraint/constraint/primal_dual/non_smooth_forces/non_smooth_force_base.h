#pragma once

#include "force_constraint/lorentz_circle_constraint.h"

namespace pe_phys_constraint {

    class NonSmoothForceBase {
    public:
        NonSmoothForceBase(): _fc(new LorentzCircleConstraint(0.5)) {}
        virtual ~NonSmoothForceBase() {}

        virtual int dimensions() const = 0;
        virtual int constraintPerForce() const = 0;
        virtual void initForces(pe::VectorX& forces, pe::VectorX& lambda) = 0;
        virtual void preprocess(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                const pe::Array<pe_phys_object::RigidBody*>& objects,
                                const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                const pe::VectorX& vel, pe::Real dt, pe::Real char_mass, pe::Real char_speed) = 0;
        virtual pe::Real surrogateDualGap(const pe::VectorX& lambda) {
            return -lambda.mult(_constraint_val).mean();
        }
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
                                              pe::Real char_mass) { return pe::VectorX::ones(forces.size()); }
        virtual void linearSystemReserve(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                         const pe::Array<pe_phys_object::RigidBody*>& objects,
                                         const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                         pe::Set<pe::KV<size_t, size_t>>& obj_pairs,
                                         pe::Array<pe::KV<pe::KV<size_t, size_t>, bool>>& triplets) {
            for (const auto contact : contacts) {
                auto obj1 = object2index.at(contact->getObjectA());
                auto obj2 = object2index.at(contact->getObjectB());
                if (obj1 > obj2) PE_SWAP(obj1, obj2);
                if (obj_pairs.count({obj1, obj2})) continue;
                obj_pairs.insert({obj1, obj2});

                for (int row = 0; row < 6; row++) {
                    for (int col = 0; col < 6; col++) {
                        if (!objects[obj1]->isKinematic()) {
                            triplets.push_back({{obj1 * 6 + row, obj1 * 6 * col}, true});
                        }
                        if (!objects[obj2]->isKinematic()) {
                            triplets.push_back({{obj2 * 6 + row, obj2 * 6 * col}, true});
                        }
                        if (!objects[obj1]->isKinematic() && !objects[obj2]->isKinematic()) {
                            triplets.push_back({{obj1 * 6 + row, obj2 * 6 * col}, true});
                            triplets.push_back({{obj2 * 6 + row, obj1 * 6 * col}, true});
                        }
                    }
                }
            }
        }
        virtual void linearSystemAddition(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                          size_t contact_size,
                                          const pe::Array<pe_phys_object::RigidBody*>& objects,
                                          const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                          const pe::VectorX& lambda, const pe::VectorX& rf, const pe::VectorX& rl,
                                          pe::Real eps, pe::VectorX& y,
                                          pe::HashMap<pe::KV<pe_phys_object::RigidBody*, pe_phys_object::RigidBody*>, pe::Real>& mat_pointers) = 0;
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
                                      pe::Real mu, pe::Real char_mass, pe::VectorX& du, pe::VectorX& df, pe::VectorX dl) = 0;
        virtual pe::VectorX ACVector(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
                                     const pe::Array<pe_phys_object::RigidBody*>& objects,
                                     const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
                                     const pe::VectorX& vel, const pe::VectorX& forces, const pe::VectorX& lambda) {
            return pe::VectorX::zeros(forces.size());
        }

        pe::Real restitution = 0;
        bool spring_based_force = false;
        pe::Real spring_k = 1e4;
        pe::Real spring_d = 0.5;

    protected:
        ForceConstraintBase* _fc;
        pe::VectorX _constraint_val;
    };

} // namespace pe_phys_constraint