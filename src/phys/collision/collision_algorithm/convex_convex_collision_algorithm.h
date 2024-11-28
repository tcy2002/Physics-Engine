#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    typedef pe::Array<pe::Vector3> VertexArray;

    class ConvexConvexCollisionAlgorithm : public CollisionAlgorithm {
    public:
        bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                              pe::Transform trans_a, pe::Transform trans_b,
                              pe::Real refScale, ContactResult& result) override;

        static bool getClosestPoints(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                    const pe::Mesh& mesh_a, const pe::Mesh& mesh_b,
                                    const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& unique_edges_a,
                                    const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& unique_edges_b,
                                    const pe::Transform& trans_a, const pe::Transform& trans_b,
                                    pe::Real margin, pe::Real refScale, ContactResult& result);
    };

} // pe_phys_collision