#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    typedef pe::Array<pe::Vector3> VertexArray;

    class ConvexConvexCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      pe::Real refScale, ContactResult& result) override;

        static void clipHullAgainstHull(const pe::Vector3& sep_normal1,
                                        const pe::Mesh& mesh_a, const pe::Mesh& mesh_b,
                                        const pe::Transform& trans_a, const pe::Transform& trans_b,
                                        pe::Real min_dist, pe::Real max_dist,
                                        VertexArray& world_vertices_b1, VertexArray& world_vertices_b2,
                                        pe::Real margin, ContactResult& result);
        static bool findSeparatingAxis(const pe_phys_shape::Shape* shape_a,
                                       const pe_phys_shape::Shape* shape_b,
                                       const pe::Mesh& mesh_a, const pe::Mesh& mesh_b,
                                       const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& unique_edges_a,
                                       const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& unique_edges_b,
                                       const pe::Transform& trans_a, const pe::Transform& trans_b,
                                       pe::Vector3& sep, pe::Real margin, ContactResult& result);
        static void clipFaceAgainstHull(const pe::Vector3& sep_normal,
                                        const pe::Mesh& mesh_a, const pe::Transform& trans_a,
                                        VertexArray& world_vertices_b1, VertexArray& world_vertices_b2,
                                        pe::Real min_dist, pe::Real max_dist,
                                        pe::Real margin, ContactResult& result);
        static void clipFace(const VertexArray& p_in, VertexArray& p_out,
                             const pe::Vector3& plane_normal_w, pe::Real plane_eq_w);
    };

} // pe_phys_collision