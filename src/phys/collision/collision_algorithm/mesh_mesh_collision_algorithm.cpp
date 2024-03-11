#include "mesh_mesh_collision_algorithm.h"

namespace pe_phys_collision {

    // sat convex collision (bullet)
    bool MeshMeshCollisionAlgorithm::processCollision(pe_phys_object::RigidBody *object_a,
                                                      pe_phys_object::RigidBody *object_b,
                                                      ContactResult &result,
                                                      pe::Vector3 overlapMin, pe::Vector3 overlapMax) {
        if (object_a->getCollisionShape()->getType() != pe_phys_shape::ShapeType::ConvexMesh ||
            object_b->getCollisionShape()->getType() != pe_phys_shape::ShapeType::ConvexMesh) {
            return false;
        }
        auto shape_a = (pe_phys_shape::ConvexMeshShape*)object_a->getCollisionShape();
        auto shape_b = (pe_phys_shape::ConvexMeshShape*)object_b->getCollisionShape();

        auto& mesh_a = shape_a->getMesh();
        auto& mesh_b = shape_b->getMesh();
        auto transA = object_a->getTransform();
        auto transB = object_b->getTransform();

        pe::Vector3 sep;
        pe::Vector3 margin = (object_a->getMargin() + object_b->getMargin()) / 2;
        margin = PE_MAX_VEC(margin, pe::Vector3(0.005, 0.005, 0.005));
        result.setObjects(object_a, object_b);

        VertexArray world_verts_b1;
        VertexArray world_verts_b2;

        result.cleanContactPointFlag();
        if (!findSeparatingAxis(object_a, object_b, transA, transB, sep, margin, result)) {
            return false;
        }
        clipHullAgainstHull(sep, shape_a, shape_b, transA, transB,
                            PE_REAL_MIN, 0, world_verts_b1, world_verts_b2,
                            margin, result);
        result.sortContactPoints();
        return result.getPointSize() > 0;
    }

    void MeshMeshCollisionAlgorithm::clipHullAgainstHull(const pe::Vector3 &separatingNormal1,
                                                         const pe_phys_shape::ConvexMeshShape* object_a,
                                                         const pe_phys_shape::ConvexMeshShape* object_b,
                                                         const pe::Transform &transA, const pe::Transform &transB,
                                                         pe::Real minDist, pe::Real maxDist,
                                                         VertexArray &worldVertsB1, VertexArray &worldVertsB2,
                                                         pe::Vector3 margin, ContactResult &result) {
        pe::Vector3 separatingNormal = separatingNormal1.normalized();
        auto& mesh_b = object_b->getMesh();

        int closestFaceB = -1;
        pe::Real dMax = PE_REAL_MIN;
        {
            for (int face = 0; face < mesh_b.faces.size(); face++) {
                const pe::Vector3 Normal = mesh_b.faces[face].normal;
                const pe::Vector3 WorldNormal = transB.getBasis() * Normal;
                pe::Real d = WorldNormal.dot(separatingNormal);
                if (d > dMax) {
                    dMax = d;
                    closestFaceB = face;
                }
            }
        }
        if (closestFaceB < 0) {
            return;
        }

        worldVertsB1.resize(0);
        {
            const auto& polyB = mesh_b.faces[closestFaceB];
            const int numVertices = (int)polyB.indices.size();
            for (int e0 = 0; e0 < numVertices; e0++) {
                pe::Vector3 b = mesh_b.vertices[polyB.indices[e0]].position;
                worldVertsB1.push_back(transB * b);
            }
        }

        if (closestFaceB >= 0) {
            clipFaceAgainstHull(separatingNormal, object_a, transA, worldVertsB1, worldVertsB2,
                                minDist, maxDist, margin, result);
        }
    }

    void MeshMeshCollisionAlgorithm::clipFaceAgainstHull(const pe::Vector3 &separatingNormal,
                                                         const pe_phys_shape::ConvexMeshShape* object_a,
                                                         const pe::Transform &transA,
                                                         VertexArray &worldVertsB1, VertexArray &worldVertsB2,
                                                         pe::Real minDist, pe::Real maxDist,
                                                         pe::Vector3 margin, ContactResult &result) {
        worldVertsB2.resize(0);
        VertexArray* pVtxIn = &worldVertsB1;
        VertexArray* pVtxOut = &worldVertsB2;
        pVtxOut->reserve(pVtxIn->size());
        auto& mesh_a = object_a->getMesh();

        int closestFaceA = -1;
        {
            pe::Real dMin = PE_REAL_MAX;
            for (int face = 0; face < mesh_a.faces.size(); face++) {
                const pe::Vector3 Normal = mesh_a.faces[face].normal;
                const pe::Vector3 faceANormalWS = transA.getBasis() * Normal;

                pe::Real d = faceANormalWS.dot(separatingNormal);
                if (d < dMin) {
                    dMin = d;
                    closestFaceA = face;
                }
            }
        }
        if (closestFaceA < 0) {
            return;
        }

        const auto& polyA = mesh_a.faces[closestFaceA];

        // clip polygon to back of planes of all faces of hull A that are adjacent to witness face
        int numVerticesA = (int)polyA.indices.size();
        pe::Vector3 worldPlaneANormal1 = transA.getBasis() * polyA.normal;
        for (int e0 = 0; e0 < numVerticesA; e0++) {
            pe::Vector3 a = mesh_a.vertices[polyA.indices[e0]].position;
            pe::Vector3 b = mesh_a.vertices[polyA.indices[(e0 + 1) % numVerticesA]].position;
            const pe::Vector3 edge0 = a - b;
            const pe::Vector3 WorldEdge0 = transA.getBasis() * edge0;

            pe::Vector3 planeNormalWS1 = -WorldEdge0.cross(worldPlaneANormal1);  //.cross(WorldEdge0);
            pe::Vector3 worldA1 = transA * a;
            pe::Real planeEqWS1 = -worldA1.dot(planeNormalWS1);

            //clip face
            clipFace(*pVtxIn, *pVtxOut, planeNormalWS1, planeEqWS1);
            std::swap(pVtxIn, pVtxOut);
            pVtxOut->resize(0);
        }

        // only keep points that are behind the witness face
        {
            pe::Vector3 planeV = mesh_a.vertices[polyA.indices[0]].position;
            pe::Vector3 localPlaneNormal = polyA.normal;
            pe::Vector3 margin_vec = separatingNormal.mult(margin);
            pe::Real margin_real = (std::abs(margin_vec.x) +
                    std::abs(margin_vec.y) + std::abs(margin_vec.z)) / 3;

            for (int i = 0; i < pVtxIn->size(); i++) {
                pe::Vector3 vtx = pVtxIn->at(i);
                pe::Real depth = localPlaneNormal.dot(transA.inverseTransform(vtx) - planeV);
                if (depth <= minDist) {
                    depth = minDist;
                }

                if (depth <= maxDist) {
                    pe::Vector3 point = pVtxIn->at(i);
                    result.addContactPoint(separatingNormal,
                                           point - separatingNormal.mult(margin),
                                           depth + margin_real * 2);
                }
            }
            result.sortContactPoints();
        }
    }

    void MeshMeshCollisionAlgorithm::clipFace(const VertexArray &pVtxIn, VertexArray &ppVtxOut,
                                              const pe::Vector3 &planeNormalWS, pe::Real planeEqWS) {
        int ve;
        pe::Real ds, de;
        int numVerts = (int)pVtxIn.size();
        if (numVerts < 2) return;

        pe::Vector3 firstVertex = pVtxIn[pVtxIn.size() - 1];
        pe::Vector3 endVertex;

        ds = planeNormalWS.dot(firstVertex) + planeEqWS;

        for (ve = 0; ve < numVerts; ve++) {
            endVertex = pVtxIn[ve];

            de = planeNormalWS.dot(endVertex) + planeEqWS;

            if (ds < 0) {
                if (de < 0) {
                    // Start < 0, end < 0, so output endVertex
                    ppVtxOut.push_back(endVertex);
                } else {
                    // Start < 0, end >= 0, so output intersection
                    ppVtxOut.push_back(firstVertex.lerp(endVertex, pe::Real(ds * 1.f / (ds - de))));
                }
            } else {
                if (de < 0) {
                    // Start >= 0, end < 0 so output intersection and end
                    ppVtxOut.push_back(firstVertex.lerp(endVertex, pe::Real(ds * 1.f / (ds - de))));
                    ppVtxOut.push_back(endVertex);
                }
            }
            firstVertex = endVertex;
            ds = de;
        }
    }

    void segmentsClosestPoints(
            pe::Vector3& ptsVector,
            pe::Vector3& offsetA,
            pe::Vector3& offsetB,
            pe::Real& tA, pe::Real& tB,
            const pe::Vector3& translation,
            const pe::Vector3& dirA, pe::Real hLenA,
            const pe::Vector3& dirB, pe::Real hLenB) {
        // compute the parameters of the closest points on each line segment

        pe::Real dirA_dot_dirB = dirA.dot(dirB);
        pe::Real dirA_dot_trans = dirA.dot(translation);
        pe::Real dirB_dot_trans = dirB.dot(translation);

        pe::Real denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;

        if (denom == 0.0f) {
            tA = 0.0f;
        } else {
            tA = (dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB) / denom;
            if (tA < -hLenA) tA = -hLenA;
            else if (tA > hLenA) tA = hLenA;
        }

        tB = tA * dirA_dot_dirB - dirB_dot_trans;

        if (tB < -hLenB) {
            tB = -hLenB;
            tA = tB * dirA_dot_dirB + dirA_dot_trans;

            if (tA < -hLenA) tA = -hLenA;
            else if (tA > hLenA) tA = hLenA;
        } else if (tB > hLenB) {
            tB = hLenB;
            tA = tB * dirA_dot_dirB + dirA_dot_trans;

            if (tA < -hLenA) tA = -hLenA;
            else if (tA > hLenA) tA = hLenA;
        }

        // compute the closest points relative to segment centers.

        offsetA = dirA * tA;
        offsetB = dirB * tB;

        ptsVector = translation - offsetA + offsetB;
    }

    static bool testSepAxis(const pe_phys_shape::ConvexMeshShape* object_a,
                            const pe_phys_shape::ConvexMeshShape* object_b,
                            const pe::Transform& transA, const pe::Transform& transB,
                            const pe::Vector3& sep_axis, pe::Real& depth,
                            pe::Vector3& witnessPointA, pe::Vector3& witnessPointB) {
        pe::Real Min0, Max0;
        pe::Real Min1, Max1;
        pe::Vector3 witnessPtMinA, witnessPtMaxA;
        pe::Vector3 witnessPtMinB, witnessPtMaxB;

        object_a->project(transA, sep_axis, Min0, Max0, witnessPtMinA, witnessPtMaxA);
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

    bool MeshMeshCollisionAlgorithm::findSeparatingAxis(const pe_phys_object::RigidBody* object_a,
                                                        const pe_phys_object::RigidBody* object_b,
                                                        const pe::Transform &transA, const pe::Transform &transB,
                                                        pe::Vector3 &sep, pe::Vector3 margin, ContactResult &result) {
        auto shape_a = (pe_phys_shape::ConvexMeshShape*)object_a->getCollisionShape();
        auto shape_b = (pe_phys_shape::ConvexMeshShape*)object_b->getCollisionShape();
        auto& mesh_a = shape_a->getMesh();
        auto& mesh_b = shape_b->getMesh();
        const pe::Vector3 c0 = transA.getOrigin();
        const pe::Vector3 c1 = transB.getOrigin();
        const pe::Vector3 DeltaC2 = c0 - c1;

        pe::Real dMin = PE_REAL_MAX;
        pe::Vector3 dMinPt;
        bool ptOnA = false;

        // Test normals from hullA
        for (int i = 0; i < (int)mesh_a.faces.size(); i++) {
            const pe::Vector3 Normal = mesh_a.faces[i].normal;
            pe::Vector3 faceANormalWS = transA.getBasis() * Normal;
            if (DeltaC2.dot(faceANormalWS) < 0)
                faceANormalWS *= -1.f;

            pe::Real d;
            pe::Vector3 wA, wB;
            if (!testSepAxis(shape_a, shape_b, transA, transB, faceANormalWS, d, wA, wB)) {
                return false;
            }

            if (d < dMin) {
                dMin = d;
                dMinPt = wB;
                ptOnA = false;
                sep = faceANormalWS;
            }
        }

        // Test normals from hullB
        for (int i = 0; i < (int)mesh_b.faces.size(); i++) {
            const pe::Vector3 Normal = mesh_b.faces[i].normal;
            pe::Vector3 WorldNormal = transB.getBasis() * Normal;
            if (DeltaC2.dot(WorldNormal) < 0) {
                WorldNormal *= -1.f;
            }

            pe::Real d;
            pe::Vector3 wA, wB;
            if (!testSepAxis(shape_a, shape_b, transA, transB,
                             WorldNormal, d, wA, wB)) {
                return false;
            }

            if (d < dMin) {
                dMin = d;
                dMinPt = wA;
                ptOnA = true;
                sep = WorldNormal;
            }
        }

        if ((ptOnA && object_b->getCollisionShape()->localIsInside(transB.inverseTransform(dMinPt))) ||
            (!ptOnA && object_a->getCollisionShape()->localIsInside(transA.inverseTransform(dMinPt)))) {
            pe::Vector3 margin_vec = sep.mult(margin);
            pe::Real margin_real = (std::abs(margin_vec.x) +
                                    std::abs(margin_vec.y) + std::abs(margin_vec.z)) / 3;
            result.addContactPoint(sep, dMinPt - margin_vec,
                                   -dMin + margin_real * 2);
        }

        int edgeA = -1;
        int edgeB = -1;
        pe::Vector3 worldEdgeA;
        pe::Vector3 worldEdgeB;
        pe::Vector3 witnessPointA(0, 0, 0), witnessPointB(0, 0, 0);

        // Test edges
        auto& unique_edges_a = shape_a->getUniqueEdges();
        auto& unique_edges_b = shape_b->getUniqueEdges();
        for (int e0 = 0; e0 < unique_edges_a.size(); e0++) {
            const pe::Vector3 edge0 = unique_edges_a[e0];
            const pe::Vector3 WorldEdge0 = transA.getBasis() * edge0;
            for (int e1 = 0; e1 < unique_edges_b.size(); e1++) {
                const pe::Vector3 edge1 = unique_edges_b[e1];
                const pe::Vector3 WorldEdge1 = transB.getBasis() * edge1;

                pe::Vector3 Cross = WorldEdge0.cross(WorldEdge1);
                if (!PE_APPROX_EQUAL(Cross.norm(), 0)) {
                    Cross = Cross.normalized();
                    if (DeltaC2.dot(Cross) < 0) {
                        Cross *= -1.f;
                    }

                    pe::Real dist;
                    pe::Vector3 wA, wB;
                    if (!testSepAxis(shape_a, shape_b, transA, transB,
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

            segmentsClosestPoints(ptsVector, offsetA, offsetB, tA, tB,
                                  translation, dirA, hLenA, dirB, hLenB);

            pe::Real nlSqrt = ptsVector.norm2();
            if (nlSqrt > PE_EPS) {
                pe::Real nl = std::sqrt(nlSqrt);
                ptsVector *= 1.f / nl;
                if (ptsVector.dot(DeltaC2) < 0.f) {
                    ptsVector *= -1.f;
                }
                pe::Vector3 ptOnB = witnessPointB + offsetB;
                pe::Real distance = nl;

                pe::Vector3 margin_vec = ptsVector.mult(margin);
                pe::Real margin_real = (std::abs(margin_vec.x) +
                                        std::abs(margin_vec.y) + std::abs(margin_vec.z)) / 3;
                result.addContactPoint(ptsVector, ptOnB - margin_vec,
                                       -distance + margin_real * 2);
            }
        }

        if ((DeltaC2.dot(sep)) < 0.0f) {
            sep = -sep;
        }

        return true;
    }

} // pe_phys_collision