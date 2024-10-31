#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    typedef pe::Array<pe::Vector3> VertexArray;

    class ConvexConvexCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      ContactResult& result) override;

        static void clipHullAgainstHull(const pe::Vector3& separatingNormal1,
                                        const pe::Mesh& meshA, const pe::Mesh& meshB,
                                        const pe::Transform& transA, const pe::Transform& transB,
                                        pe::Real minDist, pe::Real maxDist,
                                        VertexArray& worldVertsB1, VertexArray& worldVertsB2,
                                        pe::Real margin, ContactResult& result);
        static void segmentsClosestPoints(pe::Vector3& ptsVector,
                                          pe::Vector3& offsetA,
                                          pe::Vector3& offsetB,
                                          pe::Real& tA, pe::Real& tB,
                                          const pe::Vector3& translation,
                                          const pe::Vector3& dirA, pe::Real hLenA,
                                          const pe::Vector3& dirB, pe::Real hLenB);
        static bool testSepAxis(const pe_phys_shape::Shape* object_a,
                                const pe_phys_shape::Shape* object_b,
                                const pe::Transform& transA, const pe::Transform& transB,
                                const pe::Vector3& sep_axis, pe::Real& depth,
                                pe::Vector3& witnessPointA, pe::Vector3& witnessPointB);
        static bool findSeparatingAxis(const pe_phys_shape::Shape* shapeA,
                                       const pe_phys_shape::Shape* shapeB,
                                       const pe::Mesh& meshA, const pe::Mesh& meshB,
                                       const pe::Array<pe::Vector3>& uniqueEdgesA,
                                       const pe::Array<pe::Vector3>& uniqueEdgesB,
                                       const pe::Transform& transA, const pe::Transform& transB,
                                       pe::Vector3& sep, pe::Real margin, ContactResult& resultOut);
        static void clipFaceAgainstHull(const pe::Vector3& separatingNormal,
                                        const pe::Mesh& meshA, const pe::Transform& transA,
                                        VertexArray& worldVertsB1, VertexArray& worldVertsB2,
                                        pe::Real minDist, pe::Real maxDist,
                                        pe::Real margin, ContactResult& resultOut);
        static void clipFace(const VertexArray& pVtxIn, VertexArray& ppVtxOut,
                             const pe::Vector3& planeNormalWS, pe::Real planeEqWS);
    };

} // pe_phys_collision