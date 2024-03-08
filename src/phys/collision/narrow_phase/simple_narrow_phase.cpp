#include "simple_narrow_phase.h"
#include "core/viewer.h"

namespace pe_phys_collision {

    void SimpleNarrowPhase::calcContactResults(const pe::Array<CollisionPair>& pairs) {
        // TODO: map pairs to collision algorithms
        for (auto& pair : pairs) {
            ContactResult result;
            pe::Vector3 overlap_min, overlap_max;
            auto type_a = pair.first->getCollisionShape()->getType();
            auto type_b = pair.second->getCollisionShape()->getType();
            bool ret = false;
            if (type_a == pe_phys_shape::ShapeType::Box &&
                type_b == pe_phys_shape::ShapeType::Box) {
                ret = _algos[0]->processCollision(pair.first, pair.second, result,
                                                  overlap_min, overlap_max);
            } else if (type_a == pe_phys_shape::ShapeType::ConvexMesh &&
                       type_b == pe_phys_shape::ShapeType::ConvexMesh) {
                ret = _algos[1]->processCollision(pair.first, pair.second, result,
                                                  overlap_min, overlap_max);
            }
            if (ret) {
                _contact_results.push_back(result);

//                static pe::Array<int> debug_points;
//                for (auto id : debug_points) {
//                    pe_core::Viewer::removeCube(id);
//                }
//                debug_points.clear();
//
//                for (int i = 0; i < result.getPointSize(); i++) {
//                    auto& p = result.getContactPoint(i);
//                    std::cout << i << ": " << p.getDistance() << " ";
//                    std::cout << p.getWorldPos();
//                    std::cout << p.getWorldNormal() << std::endl;
//                    int id = pe_core::Viewer::addCube({0.02, 0.02, 0.02});
//                    pe_core::Viewer::updateCubeTransform(id, pe::Transform(pe::Matrix3::identity(), p.getWorldPos()));
//                    pe_core::Viewer::updateCubeColor(id, {1, 0, 0});
//                    debug_points.push_back(id);
//                }
            }
        }
    }

    void SimpleNarrowPhase::clearContactResults() {
        _contact_results.clear();
    }

    pe::Array<ContactResult>& SimpleNarrowPhase::getContactResults() {
        return _contact_results;
    }

} // namespace pe_phys_collision