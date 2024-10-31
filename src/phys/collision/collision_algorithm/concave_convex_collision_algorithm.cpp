#include "concave_convex_collision_algorithm.h"
#include <phys/shape/concave_mesh_shape.h>
#include <phys/shape/convex_mesh_shape.h>
#include "convex_convex_collision_algorithm.h"

namespace pe_phys_collision {

    bool ConcaveConvexCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                           pe::Transform trans_a, pe::Transform trans_b,
                                                           ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh &&
            shape_b->getType() == pe_phys_shape::ShapeType::ConcaveMesh) ||
            (shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh &&
                shape_b->getType() == pe_phys_shape::ShapeType::ConvexMesh))) {
            return false;
        }

        auto shape_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? shape_a : shape_b;
        auto shape_convex = shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? shape_a : shape_b;
        auto trans_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? trans_a : trans_b;
        auto trans_convex = shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? trans_a : trans_b;
        auto& mesh_concave = ((pe_phys_shape::ConcaveMeshShape*)shape_concave)->getMesh();
        auto& mesh_convex = ((pe_phys_shape::ConvexMeshShape*)shape_convex)->getMesh();

        pe::Vector3 sep;
        pe::Real margin = PE_MARGIN;

        VertexArray world_verts_b1;
        VertexArray world_verts_b2;

        pe::Transform trans_convex_rel2concave = trans_concave.inverse() * trans_convex;
        pe::Vector3 convex_AA, convex_BB;
        shape_convex->getAABB(trans_convex_rel2concave, convex_AA, convex_BB);
        pe::Array<int> intersect;
        ((pe_phys_shape::ConvexMeshShape*)shape_concave)->getIntersetFaces(convex_AA, convex_BB, intersect);

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh);
        for (auto fi : intersect) {
            auto& f = mesh_concave.faces[fi];
            pe::Array<pe::Vector3> concave_unique_edges;
            getUniqueEdges(mesh_concave, f, concave_unique_edges);
            if (!findSeparatingAxis(f.normal, shape_convex, mesh_concave, f, mesh_convex,
                                    concave_unique_edges,
                                    ((pe_phys_shape::ConvexMeshShape*)shape_convex)->getUniqueEdges(),
                                    trans_concave, trans_convex, sep, margin, result)) {
                continue;
            }

            world_verts_b1.resize(0);
            for (auto e0 : f.indices) {
                world_verts_b1.push_back(trans_concave * mesh_concave.vertices[e0].position);
            }
            ConvexConvexCollisionAlgorithm::clipFaceAgainstHull(sep, mesh_convex, trans_convex,
                world_verts_b1, world_verts_b2,
                PE_REAL_MIN, margin, margin, result);
        }

        return true;
    }

    static void projectFace(const pe::Mesh& mesh, const pe::Mesh::Face& face,
                            const pe::Transform& trans, const pe::Vector3 &axis,
                            pe::Real &minProj, pe::Real &maxProj,
                            pe::Vector3& minPoint, pe::Vector3& maxPoint) {
        pe::Matrix3 rot = trans.getBasis();
        pe::Vector3 pos = trans.getOrigin();
        pe::Vector3 local_axis = rot.transposed() * axis;
        pe::Real offset = pos.dot(axis);

        minProj = PE_REAL_MAX;
        maxProj = PE_REAL_MIN;
        for (auto fi: face.indices) {
            auto& p = mesh.vertices[fi].position;
            auto v = p.dot(local_axis);
            if (v < minProj) {
                minProj = v;
                minPoint = p;
            } else if (v > maxProj) {
                maxProj = v;
                maxPoint = p;
            }
        }

        minProj += offset;
        maxProj += offset;
        minPoint = trans * minPoint;
        maxPoint = trans * maxPoint;
    }

    void ConcaveConvexCollisionAlgorithm::getUniqueEdges(const pe::Mesh& mesh, const pe::Mesh::Face& face,
                                                         pe::Array<pe::Vector3>& uniqueEdges) {
        int count = (int)face.indices.size();
        for (int i = 0; i < count; i++) {
            auto v1 = mesh.vertices[face.indices[i]].position;
            auto v2 = mesh.vertices[face.indices[(i + 1) % count]].position;
            uniqueEdges.push_back((v2 - v1).normalized());
        }
        uniqueEdges.push_back(face.normal);
    }

    bool ConcaveConvexCollisionAlgorithm::testSepAxis(
            const pe::Mesh& meshA, const pe::Mesh::Face& faceA,
            const pe_phys_shape::Shape* object_b,
            const pe::Transform& transA, const pe::Transform& transB,
            const pe::Vector3& sep_axis, pe::Real& depth,
            pe::Vector3& witnessPointA, pe::Vector3& witnessPointB) {
        pe::Real Min0, Max0;
        pe::Real Min1, Max1;
        pe::Vector3 witnessPtMinA, witnessPtMaxA;
        pe::Vector3 witnessPtMinB, witnessPtMaxB;

        projectFace(meshA, faceA, transA, sep_axis, Min0, Max0, witnessPtMinA, witnessPtMaxA);
        object_b->project(transB, sep_axis, Min1, Max1, witnessPtMinB, witnessPtMaxB);

        if (Max0 < Min1 || Max1 < Min0) return false;

        pe::Real d0 = Max0 - Min1;
        pe::Real d1 = Max1 - Min0;
        if (d0 < d1) {
            depth = d0;
            witnessPointA = witnessPtMaxA;
            witnessPointB = witnessPtMinB;
        } else {
            depth = d1;
            witnessPointA = witnessPtMinA;
            witnessPointB = witnessPtMaxB;
        }

        return true;
    }

    bool ConcaveConvexCollisionAlgorithm::findSeparatingAxis(
            const pe::Vector3& normA,
            const pe_phys_shape::Shape* shapeB,
            const pe::Mesh& meshA, const pe::Mesh::Face& faceA, const pe::Mesh& meshB,
            const pe::Array<pe::Vector3>& uniqueEdgesA,
            const pe::Array<pe::Vector3>& uniqueEdgesB,
            const pe::Transform& transA, const pe::Transform& transB,
            pe::Vector3& sep, pe::Real margin, ContactResult& result) {
        const pe::Vector3 c0 = transA.getOrigin();
        const pe::Vector3 c1 = transB.getOrigin();
        const pe::Vector3 DeltaC2 = c0 - c1;

        pe::Real dMin = PE_REAL_MAX;

        // Test normal of triA
        {
            pe::Vector3 faceANormalWS = transA.getBasis() * normA;
            if (DeltaC2.dot(faceANormalWS) < 0) {
                faceANormalWS *= pe::Real(-1.0);
            }

            pe::Real d;
            pe::Vector3 wA, wB;
            if (!testSepAxis(meshA, faceA, shapeB, transA, transB,
                             faceANormalWS, d, wA, wB)) {
                return false;
            }

            if (d < dMin) {
                dMin = d;
                sep = faceANormalWS;
            }
        }

        // Test normals from hullB
        for (int i = 0; i < (int)meshB.faces.size(); i++) {
            const pe::Vector3 Normal = meshB.faces[i].normal;
            pe::Vector3 WorldNormal = transB.getBasis() * Normal;
            if (DeltaC2.dot(WorldNormal) < 0) {
                WorldNormal *= pe::Real(-1.0);
            }

            pe::Real d;
            pe::Vector3 wA, wB;
            if (!testSepAxis(meshA, faceA, shapeB, transA, transB,
                             WorldNormal, d, wA, wB)) {
                return false;
                             }

            if (d < dMin) {
                dMin = d;
                sep = WorldNormal;
            }
        }

        // no need to test corner case

        int edgeA = -1;
        int edgeB = -1;
        pe::Vector3 worldEdgeA;
        pe::Vector3 worldEdgeB;
        pe::Vector3 witnessPointA(0, 0, 0), witnessPointB(0, 0, 0);

        // Test edges
        for (int e0 = 0; e0 < (int)uniqueEdgesA.size(); e0++) {
            const pe::Vector3 edge0 = uniqueEdgesA[e0];
            const pe::Vector3 WorldEdge0 = transA.getBasis() * edge0;
            for (int e1 = 0; e1 < (int)uniqueEdgesB.size(); e1++) {
                const pe::Vector3 edge1 = uniqueEdgesB[e1];
                const pe::Vector3 WorldEdge1 = transB.getBasis() * edge1;

                pe::Vector3 Cross = WorldEdge0.cross(WorldEdge1);
                if (!PE_APPROX_EQUAL(Cross.norm(), 0)) {
                    Cross = Cross.normalized();
                    if (DeltaC2.dot(Cross) < 0) {
                        Cross *= pe::Real(-1.0);
                    }

                    pe::Real dist;
                    pe::Vector3 wA, wB;
                    if (!testSepAxis(meshA, faceA, shapeB, transA, transB,
                                     Cross, dist, wA, wB)) {
                        return false;
                                     }

                    if (dist < dMin) {
                        dMin = dist;
                        sep = Cross;
                        edgeA = e0;
                        edgeB = e1;
                        worldEdgeA = WorldEdge0;
                        worldEdgeB = WorldEdge1;
                        witnessPointA = wA;
                        witnessPointB = wB;
                    }
                }
            }
        }

        if (edgeA >= 0 && edgeB >= 0) {
            //add an edge-edge contact
            pe::Vector3 ptsVector;
            pe::Vector3 offsetA;
            pe::Vector3 offsetB;
            pe::Real tA;
            pe::Real tB;

            pe::Vector3 translation = witnessPointB - witnessPointA;

            pe::Vector3 dirA = worldEdgeA;
            pe::Vector3 dirB = worldEdgeB;

            pe::Real hLenB = PE_REAL_MAX;
            pe::Real hLenA = PE_REAL_MAX;

            ConvexConvexCollisionAlgorithm::segmentsClosestPoints(ptsVector, offsetA,
                offsetB, tA, tB, translation, dirA, hLenA, dirB, hLenB);

            pe::Real nlSqrt = ptsVector.norm2();
            if (nlSqrt > PE_EPS) {
                pe::Real nl = std::sqrt(nlSqrt);
                ptsVector *= pe::Real(1.0) / nl;
                if (ptsVector.dot(DeltaC2) < 0.f) {
                    ptsVector *= pe::Real(-1.0);
                }
                pe::Vector3 ptOnB = witnessPointB + offsetB;
                pe::Real distance = nl;

                result.addContactPoint(ptsVector, ptOnB - ptsVector * margin,
                                       -distance + margin * 2);
            }
        }

        if (DeltaC2.dot(sep) < 0) {
            sep = -sep;
        }

        return true;
    }

} // pe_phys_collision