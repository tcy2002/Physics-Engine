#include "simple_narrow_phase.h"
#include "utils/thread_pool.h"

namespace pe_phys_collision {

    void SimpleNarrowPhase::calcContactResults(const pe::Array<CollisionPair>& pairs,
                                               pe::Array<ContactResult*>& results) {
        // clear old contact results
        static pe::Real total_time = 0;
        static int frame = 0;
        frame++;
        auto start = COMMON_GetMicroseconds();

        const int old_size = (int)results.size();
        const int new_size = (int)pairs.size();
        if (old_size < new_size) {
            results.resize(new_size);
            for (int i = old_size; i < new_size; i++) {
                results[i] = _cr_pool.create();
            }
        } else {
            for (int i = new_size; i < old_size; i++) {
				_cr_pool.destroy(results[i]);
            }
            results.resize(new_size);
        }

#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forBatchedLoop(pairs.size(), 0, [&](int i) {
            auto shape_a = pairs[i].first->getCollisionShape();
            auto shape_b = pairs[i].second->getCollisionShape();
            auto trans_a = pairs[i].first->getTransform();
            auto trans_b = pairs[i].second->getTransform();
            auto type_a = shape_a->getType();
            auto type_b = shape_b->getType();
            results[i]->clearContactPoints();
            results[i]->setObjects(pairs[i].first, pairs[i].second);
            getAlgorithm(type_a, type_b)->processCollision(shape_a, shape_b, trans_a, trans_b, *results[i]);
            results[i]->sortContactPoints();
        });
        utils::ThreadPool::join();
#   else
        for (int i = 0; i < (int)pairs.size(); i++) {
            auto shape_a = pairs[i].first->getCollisionShape();
            auto shape_b = pairs[i].second->getCollisionShape();
            auto trans_a = pairs[i].first->getTransform();
            auto trans_b = pairs[i].second->getTransform();
            auto type_a = shape_a->getType();
            auto type_b = shape_b->getType();
            results[i]->clearContactPoints();
            results[i]->setObjects(pairs[i].first, pairs[i].second);
            getAlgorithm(type_a, type_b)->processCollision(shape_a, shape_b, trans_a, trans_b, *results[i]);
            results[i]->sortContactPoints();
        }
#   endif
        
        // remove empty contact results and update dynamic/static count
        for (int i = (int)results.size() - 1; i >= 0; i--) {
            if (results[i]->getPointSize() == 0) {
				/*delete results[i];
                results.erase(results.begin() + i);*/
            } else {
                auto obj_a = results[i]->getObjectA();
                auto obj_b = results[i]->getObjectB();
                if (!obj_b->isKinematic()) {
                    if (obj_a->isSleep() || obj_a->isKinematic()) {
                        obj_b->incStaticCount();
                    } else {
                        obj_b->incDynamicCount();
                    }
                }
                if (!obj_a->isKinematic()) {
                    if (obj_b->isSleep() || obj_b->isKinematic()) {
                        obj_a->incStaticCount();
                    } else {
                        obj_a->incDynamicCount();
                    }
                }
            }
        }

        auto end = COMMON_GetMicroseconds();
        total_time += end - start;
        if (frame >= 2000) {
            std::cout << "narrow phase calc: " << total_time / pe::Real(1000) << "ms" << std::endl;
        }
    }

} // namespace pe_phys_collision