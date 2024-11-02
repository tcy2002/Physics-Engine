#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    typedef pe::Array<pe::Vector3> VertexArray;

    class ConvexConvexCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      pe::Real refScale, ContactResult& result) override;

        static void clipHullAgainstHull(const pe::Vector3& separatingNormal1,
                                        const pe::Mesh& meshA, const pe::Mesh& meshB,
                                        const pe::Transform& transA, const pe::Transform& transB,
                                        pe::Real minDist, pe::Real maxDist,
                                        VertexArray& worldVertsB1, VertexArray& worldVertsB2,
                                        pe::Real margin, ContactResult& result);
        static bool findSeparatingAxis(const pe_phys_shape::Shape* shapeA,
                                       const pe_phys_shape::Shape* shapeB,
                                       const pe::Mesh& meshA, const pe::Mesh& meshB,
                                       const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& uniqueEdgesA,
                                       const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& uniqueEdgesB,
                                       const pe::Transform& transA, const pe::Transform& transB,
                                       pe::Vector3& sep, pe::Real margin, ContactResult& result);
        static void clipFaceAgainstHull(const pe::Vector3& separatingNormal,
                                        const pe::Mesh& meshA, const pe::Transform& transA,
                                        VertexArray& worldVertsB1, VertexArray& worldVertsB2,
                                        pe::Real minDist, pe::Real maxDist,
                                        pe::Real margin, ContactResult& resultOut);
        static void clipFace(const VertexArray& pVtxIn, VertexArray& ppVtxOut,
                             const pe::Vector3& planeNormalWS, pe::Real planeEqWS);
    };

} // pe_phys_collision