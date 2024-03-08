#pragma once

#include "collision_algorithm.h"
#include "phys/shape/convex_mesh_shape.h"

namespace pe_phys_collision {

    typedef pe::Array<pe::Vector3> VertexArray;

    class MeshMeshCollisionAlgorithm : public CollisionAlgorithm {
    private:
        VertexArray _world_verts_b1;
        VertexArray _world_verts_b2;

    public:
        virtual bool processCollision(pe_phys_object::RigidBody* object_a, pe_phys_object::RigidBody* object_b,
                                      ContactResult& result, pe::Vector3 overlapMin, pe::Vector3 overlapMax) override;

        static void clipHullAgainstHull(const pe::Vector3& separatingNormal1,
                                        const pe_phys_shape::ConvexMeshShape* object_a,
                                        const pe_phys_shape::ConvexMeshShape* object_b,
                                        const pe::Transform& transA, const pe::Transform& transB,
                                        pe::Real minDist, pe::Real maxDist,
                                        VertexArray& worldVertsB1, VertexArray& worldVertsB2,
                                        pe::Real margin, ContactResult& resultOut);
        static void clipFaceAgainstHull(const pe::Vector3& separatingNormal,
                                        const pe_phys_shape::ConvexMeshShape* object_a,
                                        const pe::Transform& transA,
                                        VertexArray& worldVertsB1, VertexArray& worldVertsB2,
                                        pe::Real minDist, pe::Real maxDist,
                                        pe::Real margin, ContactResult& resultOut);
        static void clipFace(const VertexArray& pVtxIn, VertexArray& ppVtxOut,
                             const pe::Vector3& planeNormalWS, pe::Real planeEqWS);
        static bool findSeparatingAxis(const pe_phys_object::RigidBody* object_a,
                                       const pe_phys_object::RigidBody* object_b,
                                       const pe::Transform& transA, const pe::Transform& transB,
                                       pe::Vector3& sep, pe::Real margin, ContactResult& resultOut);
    };

} // pe_phys_collision