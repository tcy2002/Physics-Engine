#include "sphere_convex_collision_algorithm.h"
#include "phys/shape/sphere_shape.h"
#include "phys/shape/convex_mesh_shape.h"

namespace pe_phys_collision {

    bool SphereConvexCollisionAlgorithm::processCollision(pe_phys_object::RigidBody* object_a,
                                                          pe_phys_object::RigidBody* object_b,
                                                          ContactResult& result) {
        if (object_a->getCollisionShape()->getType() == pe_phys_shape::ShapeType::ConvexMesh) {
            std::swap(object_a, object_b);
        }
        if (object_a->getCollisionShape()->getType() != pe_phys_shape::ShapeType::Sphere ||
            object_b->getCollisionShape()->getType() != pe_phys_shape::ShapeType::ConvexMesh) {
            return false;
        }

        auto shape_b = (pe_phys_shape::ConvexMeshShape*)object_b->getCollisionShape();
        auto& mesh_b = shape_b->getMesh();
        auto& trans_b = object_b->getTransform();

        pe::Vector3 vertices[3];
        result.setObjects(object_a, object_b);
        for (auto& f : mesh_b.faces) {
            for (int i = 0; i < f.indices.size() - 2; i++) {
                vertices[0] = mesh_b.vertices[f.indices[0]].position;
                vertices[1] = mesh_b.vertices[f.indices[i + 1]].position;
                vertices[2] = mesh_b.vertices[f.indices[i + 2]].position;
                getClosestPoints(object_a, vertices, trans_b, result, false);
            }
        }

        result.sortContactPoints();
        return result.getPointSize() > 0;
    }

    void SphereConvexCollisionAlgorithm::getClosestPoints(pe_phys_object::RigidBody *object_a,
                                                          const pe::Vector3 vertices[],
                                                          const pe::Transform& transTri,
                                                          ContactResult &result, bool shouldSwap) {
        const pe::Transform& transSph = object_a->getTransform();

        pe::Vector3 point, normal;
        pe::Real depth = 0;
        pe::Real margin = 0.005;
        //move sphere into triangle space
        pe::Vector3 sphereInTr = transTri.inverseTransform(transSph.getOrigin());
        pe::Real sphereRadius = ((pe_phys_shape::SphereShape*)object_a->getCollisionShape())->getRadius();

        if (collideSphereTriangle(sphereInTr, sphereRadius, vertices,
                                  point, normal, depth, margin)) {
            if (shouldSwap) {
                pe::Vector3 normalOnB = transTri.getBasis() * normal;
                pe::Vector3 normalOnA = -normalOnB;
                pe::Vector3 pointOnA = transTri * point + normalOnB * depth;
                result.addContactPoint(normalOnA, pointOnA, depth);
            } else {
                result.addContactPoint(transTri.getBasis() * normal, transTri * point, depth);
            }
        }
    }

    static pe::Real segmentSqrDistance(const pe::Vector3& from, const pe::Vector3& to,
                                       const pe::Vector3& p, pe::Vector3& nearest) {
        pe::Vector3 diff = p - from;
        pe::Vector3 v = to - from;
        pe::Real t = v.dot(diff);

        if (t > 0) {
            pe::Real dotVV = v.dot(v);
            if (t < dotVV) {
                t /= dotVV;
                diff -= t * v;
            } else {
                t = 1;
                diff -= v;
            }
        } else {
            t = 0;
        }

        nearest = from + t * v;
        return diff.dot(diff);
    }

    bool SphereConvexCollisionAlgorithm::collideSphereTriangle(const pe::Vector3& sphereCenter, pe::Real radius,
                                                               const pe::Vector3 vertices[], pe::Vector3& point,
                                                               pe::Vector3& resultNormal, pe::Real& depth,
                                                               pe::Real margin) {
        pe::Real radiusWithThreshold = radius + margin;
        pe::Vector3 normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
        pe::Real l2 = normal.norm2();
        bool hasContact = false;
        pe::Vector3 contactPoint;

        if (l2 >= PE_EPS * PE_EPS)
        {
            normal /= std::sqrt(l2);

            pe::Vector3 p1ToCentre = sphereCenter - vertices[0];
            pe::Real distanceFromPlane = p1ToCentre.dot(normal);

            if (distanceFromPlane < pe::Real(0.))
            {
                //triangle facing the other way
                distanceFromPlane *= pe::Real(-1.);
                normal *= pe::Real(-1.);
            }

            bool isInsideContactPlane = distanceFromPlane < radiusWithThreshold;

            // Check for contact / intersection

            if (isInsideContactPlane) {
                if (faceContains(sphereCenter, vertices, normal)) {
                    // Inside the contact wedge - touches a point on the shell plane
                    hasContact = true;
                    contactPoint = sphereCenter - normal * distanceFromPlane;
                } else {
                    // Could be inside one of the contact capsules
                    pe::Real contactCapsuleRadiusSqr = radiusWithThreshold * radiusWithThreshold;
                    pe::Real minDistSqr = contactCapsuleRadiusSqr;
                    pe::Vector3 nearestOnEdge;
                    for (int i = 0; i < 3; i++) {
                        pe::Vector3 pa = vertices[i];
                        pe::Vector3 pb = vertices[(i + 1) % 3];

                        pe::Real distanceSqr = segmentSqrDistance(pa, pb, sphereCenter, nearestOnEdge);
                        if (distanceSqr < minDistSqr) {
                            // Yep, we're inside a capsule, and record the capsule with the smallest distance
                            minDistSqr = distanceSqr;
                            hasContact = true;
                            contactPoint = nearestOnEdge;
                        }
                    }
                }
            }
        }

        if (hasContact) {
            pe::Vector3 contactToCentre = sphereCenter - contactPoint;
            pe::Real distanceSqr = contactToCentre.norm2();

            if (distanceSqr < radiusWithThreshold * radiusWithThreshold) {
                if (distanceSqr > PE_EPS * PE_EPS) { // TODO: check if this is correct
                    pe::Real distance = std::sqrt(distanceSqr);
                    resultNormal = contactToCentre;
                    resultNormal.normalize();
                    point = contactPoint;
                    depth = -(radius - distance);
                } else {
                    resultNormal = normal;
                    point = contactPoint;
                    depth = -radius;
                }
                return true;
            }
        }

        return false;
    }

    bool SphereConvexCollisionAlgorithm::pointInTriangle(const pe::Vector3 *vertices, const pe::Vector3 &normal,
                                                         pe::Vector3 *p) {
        const pe::Vector3* p1 = &vertices[0];
        const pe::Vector3* p2 = &vertices[1];
        const pe::Vector3* p3 = &vertices[2];

        pe::Vector3 edge1(*p2 - *p1);
        pe::Vector3 edge2(*p3 - *p2);
        pe::Vector3 edge3(*p1 - *p3);

        pe::Vector3 p1_to_p(*p - *p1);
        pe::Vector3 p2_to_p(*p - *p2);
        pe::Vector3 p3_to_p(*p - *p3);

        pe::Vector3 edge1_normal(edge1.cross(normal));
        pe::Vector3 edge2_normal(edge2.cross(normal));
        pe::Vector3 edge3_normal(edge3.cross(normal));

        pe::Real r1, r2, r3;
        r1 = edge1_normal.dot(p1_to_p);
        r2 = edge2_normal.dot(p2_to_p);
        r3 = edge3_normal.dot(p3_to_p);
        if ((r1 > 0 && r2 > 0 && r3 > 0) || (r1 <= 0 && r2 <= 0 && r3 <= 0)) {
            return true;
        }
        return false;
    }

    bool SphereConvexCollisionAlgorithm::faceContains(const pe::Vector3 &p, const pe::Vector3 *vertices, pe::Vector3 &normal) {
        pe::Vector3 l_p(p);
        pe::Vector3 l_normal(normal);
        return pointInTriangle(vertices, l_normal, &l_p);
    }

} // pe_phys_collision