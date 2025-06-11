#include "compound_compound_collision_algorithm.h"
#include "rigid/shape/compound_shape.h"
#include "rigid/collision/narrow_phase/narrow_phase_base.h"

// style-checked.
namespace pe_phys_collision {

    bool CompoundCompoundCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                              pe::Transform trans_a, pe::Transform trans_b,
                                                              const pe::Real refScale, ContactResult& result) {
        if (shape_b->getType() == pe_phys_shape::ShapeType::ST_Compound) {
            std::swap(shape_a, shape_b);
            std::swap(trans_a, trans_b);
            result.setObjects(result.getObjectB(), result.getObjectA());
        }
        if (shape_a->getType() != pe_phys_shape::ShapeType::ST_Compound) {
            return false;
        }

        bool has_contact = false;

        for (auto& s : dynamic_cast<pe_phys_shape::CompoundShape *>(shape_a)->getShapes()) {
            pe::Transform trans_a_w = trans_a * s.local_transform;
            if (shape_b->getType() == pe_phys_shape::ShapeType::ST_Compound) {
                const auto compound_b = dynamic_cast<pe_phys_shape::CompoundShape *>(shape_b);
                for (auto& s_b : compound_b->getShapes()) {
                    pe::Transform trans_b_w = trans_b * s_b.local_transform;
                    has_contact |= processSubCollision(s.shape, s_b.shape,
                                                       trans_a_w, trans_b_w, refScale, result);
                }
            } else {
                has_contact |= processSubCollision(s.shape, shape_b,
                                                   trans_a_w, trans_b, refScale, result);
            }
        }

        return has_contact;
    }

    bool CompoundCompoundCollisionAlgorithm::processSubCollision(pe_phys_shape::Shape *shape_a,
                                                                 pe_phys_shape::Shape *shape_b,
                                                                 const pe::Transform& trans_a,
                                                                 const pe::Transform& trans_b,
                                                                 const pe::Real refScale, ContactResult &result) {
        const auto algo = NarrowPhaseBase::getAlgorithm(shape_a->getType(), shape_b->getType());
        if (algo == nullptr) return false;
        return algo->processCollision(shape_a, shape_b, trans_a, trans_b, refScale, result);
    }

} // pe_phys_collision