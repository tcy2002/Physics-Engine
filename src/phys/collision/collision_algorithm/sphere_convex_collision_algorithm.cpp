#include "sphere_convex_collision_algorithm.h"
#include "phys/shape/sphere_shape.h"
#include "phys/shape/convex_mesh_shape.h"

// sphere-triangle collision (from bullet)
// style-checked.
namespace pe_phys_collision {

    bool SphereConvexCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                          pe::Transform trans_a, pe::Transform trans_b,
                                                          pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::Sphere &&
               shape_b->getType() == pe_phys_shape::ShapeType::ConvexMesh) ||
              (shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh &&
               shape_b->getType() == pe_phys_shape::ShapeType::Sphere))) {
            return false;
        }

        const auto shape_mesh = dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? shape_a : shape_b);
        const auto shape_sph = dynamic_cast<pe_phys_shape::SphereShape *>(shape_a->getType() == pe_phys_shape::ShapeType::Sphere ? shape_a : shape_b);
        auto& mesh = shape_mesh->getMesh();
        const auto& trans_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? trans_a : trans_b;
        const auto& trans_sph = shape_a->getType() == pe_phys_shape::ShapeType::Sphere ? trans_a : trans_b;

        const pe::Vector3 sph_rel2mesh = trans_mesh.inverseTransform(trans_sph.getOrigin());
        const pe::Real radius = shape_sph->getRadius();
        const pe::Vector3 sph_AA = sph_rel2mesh - pe::Vector3(radius, radius, radius);
        const pe::Vector3 sph_BB = sph_rel2mesh + pe::Vector3(radius, radius, radius);
        pe::Array<int> intersect;
        shape_mesh->getIntersectFaces(sph_AA, sph_BB, intersect);

        pe::Vector3 vertices[3];
        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh);
        for (const auto fi : intersect) {
            auto& f = mesh.faces[fi];
            for (int i = 0; i < (int)f.indices.size() - 2; i++) {
                vertices[0] = mesh.vertices[f.indices[0]].position;
                vertices[1] = mesh.vertices[f.indices[i + 1]].position;
                vertices[2] = mesh.vertices[f.indices[i + 2]].position;
                getClosestPoints(shape_sph, trans_sph, vertices, trans_mesh, result);
            }
        }
        result.setSwapFlag(false);

        return true;
    }

    void SphereConvexCollisionAlgorithm::getClosestPoints(pe_phys_shape::Shape* shape_a,
                                                          const pe::Transform& trans_a,
                                                          const pe::Vector3 vertices[],
                                                          const pe::Transform& transTri,
                                                          ContactResult &result) {
        pe::Vector3 point, normal;
        pe::Real depth = 0;
        constexpr auto margin = PE_MARGIN;

        //move sphere into triangle space
        const pe::Vector3 sphereInTr = transTri.inverseTransform(trans_a.getOrigin());
        const pe::Real sphereRadius = dynamic_cast<pe_phys_shape::SphereShape *>(shape_a)->getRadius();

        if (collideSphereTriangle(sphereInTr, sphereRadius, vertices,
                                  point, normal, depth)) {
            normal = transTri.getBasis() * normal;
            point = transTri * point;
            result.addContactPoint(normal, point - normal * margin, depth + 2 * margin);
        }
    }

    static pe::Real segmentSqrDistance(const pe::Vector3& from, const pe::Vector3& to,
                                       const pe::Vector3& p, pe::Vector3& nearest) {
        pe::Vector3 diff = p - from;
        const pe::Vector3 v = to - from;
        pe::Real t = v.dot(diff);

        if (t > 0) {
            const pe::Real dotVV = v.dot(v);
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

    bool SphereConvexCollisionAlgorithm::collideSphereTriangle(const pe::Vector3& sphereCenter, pe::Real sphereRadius,
                                                               const pe::Vector3 vertices[], pe::Vector3& point,
                                                               pe::Vector3& resultNormal, pe::Real& depth) {
        const pe::Real radiusWithThreshold = sphereRadius;
        pe::Vector3 normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
        const pe::Real l2 = normal.norm2();
        bool hasContact = false;
        pe::Vector3 contactPoint;

        if (l2 >= PE_EPS * PE_EPS) {
            normal /= PE_SQRT(l2);

            pe::Vector3 p1ToCentre = sphereCenter - vertices[0];
            pe::Real distanceFromPlane = p1ToCentre.dot(normal);

            if (distanceFromPlane < pe::Real(0.)) {
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

                        const pe::Real distanceSqr = segmentSqrDistance(pa, pb, sphereCenter, nearestOnEdge);
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
            const pe::Vector3 contactToCentre = sphereCenter - contactPoint;
            const pe::Real distanceSqr = contactToCentre.norm2();

            if (distanceSqr < radiusWithThreshold * radiusWithThreshold) {
                if (distanceSqr > PE_EPS * PE_EPS) {
                    const pe::Real distance = PE_SQRT(distanceSqr);
                    resultNormal = contactToCentre;
                    resultNormal.normalize();
                    point = contactPoint;
                    depth = -(sphereRadius - distance);
                } else {
                    resultNormal = normal;
                    point = contactPoint;
                    depth = -sphereRadius;
                }
                return true;
            }
        }

        return false;
    }

    bool SphereConvexCollisionAlgorithm::pointInTriangle(const pe::Vector3* vertices, const pe::Vector3& normal,
                                                         const pe::Vector3* p) {
        const pe::Vector3* p1 = &vertices[0];
        const pe::Vector3* p2 = &vertices[1];
        const pe::Vector3* p3 = &vertices[2];

        const pe::Vector3 edge1(*p2 - *p1);
        const pe::Vector3 edge2(*p3 - *p2);
        const pe::Vector3 edge3(*p1 - *p3);

        const pe::Vector3 p1_to_p(*p - *p1);
        const pe::Vector3 p2_to_p(*p - *p2);
        const pe::Vector3 p3_to_p(*p - *p3);

        const pe::Vector3 edge1_normal(edge1.cross(normal));
        const pe::Vector3 edge2_normal(edge2.cross(normal));
        const pe::Vector3 edge3_normal(edge3.cross(normal));

        const pe::Real r1 = edge1_normal.dot(p1_to_p);
        const pe::Real r2 = edge2_normal.dot(p2_to_p);
        const pe::Real r3 = edge3_normal.dot(p3_to_p);
        if ((r1 > 0 && r2 > 0 && r3 > 0) || (r1 <= 0 && r2 <= 0 && r3 <= 0)) {
            return true;
        }
        return false;
    }

    bool SphereConvexCollisionAlgorithm::faceContains(const pe::Vector3& p, const pe::Vector3* vertices,
                                                      const pe::Vector3& normal) {
        const pe::Vector3 l_p(p);
        const pe::Vector3 l_normal(normal);
        return pointInTriangle(vertices, l_normal, &l_p);
    }

} // pe_phys_collision